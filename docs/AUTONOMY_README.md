# DroneHacks 2026 autonomy stack (ROS 2 Humble)

> Current status: `AUTONOMY_STATUS.md` and `docs/LOCAL_VALIDATION.md` (paths relative to repository root). Physical flight is prohibited. Dry run is the default; nothing in the historical flight/simulation results below is hardware qualification. Use only `docs/JETSON_VERIFICATION.md` for the later stationary procedure. The tube is illuminated.


Fully autonomous, GPS-denied flight through the Stanford DroneHacks 2026 course on the issued "Sky Beta" drone (Jetson Orin Nano + Pixhawk 6C/PX4 v1.16 + OAK-D S2). The route is **planned from the official course file at start-up**, not typed in. The drone follows it with a lag-compensated tracker and corrects drift by matching depth scans against the known course. It lands and disarms on the pad and, where allowed, re-arms and flies the return over the barrier.

> **Read first:**
> - `STRATEGY.md` explains why it is built this way: the scoring math, the speed limits, why not to hard-code, and the simulation evidence.
> - `../stanford_drone_hacks_research_handoff.md` has the full competition research.
>
> **Rule R9.1 says the autonomy stack is built during the event.** Confirm with the organizers before using pre-written code.

## How it works

```
course OBJ ──► dh_core.course   (slice at cruise height, detect apertures/launch/pad/guideline)
           ──► dh_core.route    (clearance A* → shortcut → clothoid fillets → time-optimal speed profile;
                                  gate | window | auto; optional return leg over the walls)
cuVSLAM odom ─► mission_node: MapAlignment (course←odom from the launch mark + heading)
                              TrajectoryFollower (feed-forward with 0.3 s lead + P, reference clock governor)
                              state machine: WAIT_START → TAKEOFF → COURSE → LAND → LANDED → RETURN → RETURN_LAND → DONE
                              backends: skybeta (organizers' /skybeta/setpoint API) | px4 (px4_msgs offboard)
stereo pair ──► stereo_scan_node (SGBM depth → horizontal pseudo-scan, forward clearance)
            ──► localizer_node   (scan ↔ signed distance field of the course → /dh/map_fix)
mono image ───► line_node        (floor guideline → lateral correction in the maze)
```

## Repository layout

| Path | What |
|---|---|
| `ros2_ws/src/dh_core/` | Pure-numpy library (no ROS): `course.py`, `planner_astar.py`, `planner_mpl.py` (motion-primitive planner with the LQMT heuristic, Liu et al. 2017), `planner_sfc.py` (safe flight corridor + min-jerk QP, Liu et al. RA-L 2017), `timing.py`, `tracker.py`, `localize.py`, `camera_geom.py`, `scoring.py`, `sim.py`; unit tests in `test/` |
| `ros2_ws/src/dh_autonomy/` | ROS 2 nodes: `mission`, `localizer`, `stereo_scan`, `line`, `sitl_odom`; `launch/autonomy.launch.py`; `config/competition.yaml` |
| `sim/make_gazebo_world.py` | Builds a Gazebo Harmonic world containing the official course mesh, for PX4 SITL |
| `tools/` | Offline analysis: `plan_course.py`, `evaluate_*.py` (Monte Carlo), `compare_planners.py`, `test_localizer.py`, `generality_test.py`, `export_waypoints.py`, `sil/` (software-in-the-loop runs of the real nodes with a fake ROS layer and fake PX4) |
| `docs/` | Results tables, figures, research notes |

## Build (on the drone, inside the organizers' container, or on an Ubuntu 22.04 + Humble laptop)

```bash
# organizers' layout: team code lives in ~/team_ws/src (bind-mounted at /root/team_ws)
cp -r ros2_ws/src/dh_core ros2_ws/src/dh_autonomy ~/team_ws/src/
mkdir -p ~/team_ws/course && cp /path/to/dronehacks-course-v3.obj ~/team_ws/course/   # from the Challenge page
team_build.sh            # in the skybeta container (or: colcon build --symlink-install)
```

The packages need numpy, scipy (optional, for speed) and OpenCV (`python3-opencv`); all are in the Isaac ROS base image. `px4_msgs` is needed only for `backend:=px4`; use the image's own copy and **do not** add `px4_msgs` to the team workspace.

## Stationary verification only

Follow `docs/JETSON_VERIFICATION.md` after a separate authorization for hardware access. No SSH, hardware connection, flight battery, propeller, arming or hardware command was used during local development. Use `dry_run:=true`, measured optical-to-body TF and fresh VO status. The Sky Beta backend cannot land or disarm; its pad state is a hover, not CP5 completion. Raw PX4 code remains simulation-only.

## Simulate

1. **No ROS needed** (any laptop with numpy and matplotlib). Set `DH_COURSE_OBJ` to the official OBJ path, then:

   ```bash
   python tools/plan_course.py docs/figures/course_plan.png
   python tools/sil/run_sil.py --route window --localizer --v_dark 0.85 --plot sil.png
   python tools/sil/run_sil.py --backend px4 --with_return --localizer --v_dark 0.85
   python ros2_ws/src/dh_core/test/test_core.py
   python tools/sil/test_stereo_node.py --plot stereo.png   # needs opencv-python
   python tools/sil/test_line_node.py --plot line.png
   ```

2. **PX4 SITL + Gazebo Harmonic** (Ubuntu 22.04, PX4 v1.16):

   ```bash
   python sim/make_gazebo_world.py dronehacks-course-v3.obj ~/PX4-Autopilot/Tools/simulation/gz/worlds/dronehacks.sdf
   cd ~/PX4-Autopilot && PX4_GZ_MODEL_POSE='-2.515,0.914,0.1,0,0,1.5708' PX4_GZ_WORLD=dronehacks make px4_sitl gz_x500_depth
   MicroXRCEAgent udp4 -p 8888
   ros2 run dh_autonomy sitl_odom
   ros2 launch dh_autonomy autonomy.launch.py backend:=px4 odom_topic:=/dh/sim_odom enable_return:=true use_stereo:=false use_line:=false
   ros2 topic pub --once /dh/start std_msgs/msg/Empty {}
   ```

## Historical results (superseded for the current revision by `docs/LOCAL_VALIDATION.md`)

- **Auto-planned route from the official mesh:** window 16.20 m / 20.2 s; Gate B 16.75 m / 21.5 s at 0.85 m/s. Both include a straight pass along the tube axis and the landing descent ramp. Planning takes about 1.5 s on a laptop.
- **SIL of the real nodes, 10 seeds per row** (`docs/sil_results_final.md`):
  - window route, localizer on: **10/10** clean, 22.2 s, pad error 1.9 cm;
  - gate route, localizer on: **10/10** clean, 23.5 s, pad error 3.5 cm;
  - with the localizer off: 6/10 (window) and 1/10 (gate), pad error about 23 cm.
- **SIL, px4 backend with return:**
  - finish (disarm on the pad) ≈ 25.0 s;
  - pad error ≈ 4 cm;
  - return completed by about 38 s;
  - landing within 1 cm of the launch mark.
- **Generality:** the unchanged planner succeeded on 15/15 modified courses (12 random ±15 cm / ±5° perturbations, a mirrored layout, a bricked-up window). See `docs/figures/generality.png`.
- **`stereo_scan_node` end-to-end** on ray-cast stereo pairs (`tools/sil/test_stereo_node.py`, `docs/stereo_node_test.md`):
  - wall distance within 1–3 % at 0.8–2.5 m, with rectification from camera_info or from TF;
  - a 1° mis-rotated pair treated as a pure baseline reads 15–40 % short. That is why the node uses TF extrinsics when camera_info has no stereo calibration.
- **`line_node` end-to-end** on rendered corridors (`tools/sil/test_line_node.py`, `docs/line_node_test.md`):
  - guideline lateral offset within 2 mm and heading within 0.2°, including weak contrast and dark walls;
  - no false lines when none is in view (this test found and fixed a dark-wall false positive).
- **Professor's papers** (`docs/paper_notes.md`, `docs/planner_comparison.md`):
  - Safe Flight Corridor planner: works, but flies 27.2 s against 20.2 s;
  - LQMT heuristic: motion-primitive search with 2.7–3.5× fewer expansions;
  - default stays the fillet planner.
- **Unit tests** (11): distance transform, the official scoring examples, speed-profile limits, frame alignment, follower, scan matcher, stereo and floor-line camera geometry, stereo rectification from extrinsics, safe-flight-corridor safety, and the LQMT heuristic.
- **Biggest identified risk:** drift in the dark tube. Mitigations:
  - the scan-to-map localizer;
  - flying the tube's axis, at full speed;
  - slowing after the exit until a fresh fix arrives;
  - best of all, an auxiliary 2D LiDAR.

Other docs:
- `STRATEGY.md`: the points, speed and hard-coding analysis.
- `docs/presentation_outline.md`: a slide plan.
- `docs/waypoints_v3.md`: generated waypoints, for reference only.
- `docs/research_notes.md`: notes on MPL, ORB-SLAM3, VGGT and code sources.
- `docs/paper_notes.md`: the professor's three papers, what was built from each, and the measured results.

## Not verified yet (needs the real drone)

- `stereo_scan_node` on real images: what the OAK driver actually puts in camera_info and TF, SGBM quality on the real wall finish and lighting. The logic is tested only on rendered pairs.
- `line_node` on real images: `dark_delta` on the real floor and line colours. It is tested only on rendered corridors (`docs/line_node_test.md`).
- Real PX4 behaviour: land detector, re-arm checks, `MPC_XY_VEL_MAX`.
- Real cuVSLAM drift and tunnel behaviour.
- Timing on the Orin Nano CPU (SGBM at 512×320 is expected to be about 10 Hz but is unmeasured).

## Credits

- A* (Hart, Nilsson & Raphael 1968) with greedy line-of-sight post-smoothing ("A* + post-smoothing", the baseline in Nash et al.'s Theta* paper, 2007; not Theta* itself).
- Corner fillets: circular-arc waypoint transitions (Beard & McLain, *Small Unmanned Aircraft*, 2012, ch. 11), now replaced by clothoid (Euler-spiral) transitions for continuous curvature (cf. Fraichard & Scheuer, IEEE T-RO 2004).
- Speed profile: forward/backward integration under a velocity-limit curve (Bobrow, Dubowsky & Gibson 1985; Shin & McKay 1985), with a combined (friction-circle) acceleration limit.
- Time-optimal path parameterization (cf. TOPP-RA, Pham & Pham 2018).
- Motion-primitive planning and the LQMT heuristic (S. Liu, N. Atanasov, K. Mohta, V. Kumar, IROS 2017, arXiv 1709.05401; github.com/sikang/motion_primitive_library).
- Safe Flight Corridors (S. Liu et al., IEEE RA-L 2(3), 2017; github.com/sikang/DecompUtil); ADMM QP after OSQP (Stellato et al., 2020).
- Sim-to-real residual idea (E. Kaufmann et al., "Champion-level drone racing using deep reinforcement learning", Nature 620, 2023).
- DLL direct localization (Caballero & Merino, IROS 2021).
- Distance transform (Felzenszwalb & Huttenlocher 2012).
- SGM stereo (Hirschmüller 2008).
- UAV line following (Brandão et al., ICINCO 2015).
- Organizers' interface: github.com/Artash-N/skybeta.
