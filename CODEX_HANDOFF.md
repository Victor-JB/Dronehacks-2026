# CODEX_HANDOFF: DroneHacks 2026 autonomy stack (Claude Code → Codex)

Handoff date: 2026-11-06. Author: Claude Code. Target owner: Codex, on branch `saketh/autonomy`.
Claude Code is moving to Gazebo work and has **not** started it.

> **Hard constraints, unchanged:**
> - The Jetson Orin Nano and sensors may be powered independently; **physical flight is prohibited**.
> - Nobody has SSH'd into the drone. No command has been sent to any flight controller.
> - Do not arm, do not fly, and do not publish to `/skybeta/*` or `/fmu/*` on hardware.
> - Do not change PX4 firmware or parameters.
> - `dry_run` is the default and must stay on until the organizers authorize flight.

---

## 1. Repository, branch, commit, working tree

| Item | Value |
|---|---|
| Local repository | `dronehacks_autonomy/`, inside a **temporary Claude Code session folder** (deleted when that session is deleted). A portable copy is exported as a git bundle; see §16. |
| Remote | **None configured.** The repo was `git init`-ed locally on 2026-11-06 and has never been pushed. |
| Branch at handoff | `main` (local only), plus `saketh/autonomy` created at the same commit (§16) |
| Commits | `02f6209` baseline (stack as it was before the integration hardening) → `b40ea66` safety and timing hardening → **WIP checkpoint** (this handoff; hash in the final report and `git log -1`) |
| Working tree after checkpoint | Clean (everything committed). Ignored: `__pycache__/`, `*.pyc`, `tools/sil/variants/` (generated meshes), `build/ install/ log/` |
| Not in the repo | The official course mesh `dronehacks-course-v3.obj` (download it from the DroneHacks Challenge page; set `DH_COURSE_OBJ`). The research handoff `stanford_drone_hacks_research_handoff.md` (in the parent folder). |

## 2. Architecture and data flow

```
OAK-D S2 raw stereo (/oak/{left,right}/image_raw bgr8 1024x640@30 + camera_info)  [organizers' depthai driver]
   ├─► stereo_scan_node (OpenCV SGBM)  ─► /dh/stereo_scan (LaserScan, REP-117) ─► localizer_node
   │                                    └► /dh/forward_clearance (Range)       ─► mission_node (safety)
   └─► line_node (black-hat floor-line) ─► /dh/line_obs (Vector3Stamped)        ─► mission_node (lateral fix)
OAK + IMU ─► cuVSLAM (isaac_ros_visual_slam) ─► /visual_slam/tracking/odometry ─► mission_node
                                             └► /visual_slam/status (vo_state)  ─► mission_node (health)
course OBJ ─► dh_core: occupancy slice → clearance A* → LOS post-smoothing → clothoid fillets
             → forward/backward speed profile (combined accel limit) → Trajectory      [at mission start-up]
mission_node: MapAlignment(course←odom) + time-synced map/line fixes → TrajectoryFollower
              → health (odometry, VO, dark zone, jumps, big fixes) + forward-clearance governor
              → backend:  DryRunBackend (DEFAULT: /dh/dry_run/cmd_vel)
                          SkybetaBackend (/skybeta/setpoint/twist, only with dry_run:=false)
                          Px4Backend (/fmu/in/*, SITL only)
mission_node ─► /dh/pose_map (stamped with the odometry stamp) ─► localizer_node, line_node
localizer_node ─► /dh/map_fix (absolute pose at scan capture time) ─► mission_node
```

## 3. Nodes, topics, message types, QoS, frames

QoS: "sensor" = `rclpy.qos.qos_profile_sensor_data` (best-effort, depth 5). Best-effort subscribers match both reliable and best-effort publishers. A number = default reliable profile with that depth.

**`dh_mission`** (`dh_autonomy/mission_node.py`)

| Dir | Topic / service | Type | QoS | Notes |
|---|---|---|---|---|
| sub | `/visual_slam/tracking/odometry` (param `odom_topic`) | nav_msgs/Odometry | sensor | `header.stamp` used for synchronisation; frame_id expected `odom` |
| sub | `/visual_slam/status` | isaac_ros_visual_slam_interfaces/VisualSlamStatus | sensor | Optional import; `vo_state` 2 = lost (organizer guide; **unverified**) |
| sub | `/dh/start` | std_msgs/Empty | 10 | The single start command |
| sub | `/dh/map_fix` | geometry_msgs/PoseWithCovarianceStamped | 10 | Absolute course pose at the scan stamp |
| sub | `/dh/line_obs` | geometry_msgs/Vector3Stamped | 10 | x lateral, y heading, z confidence; stamp = image time |
| sub | `/dh/forward_clearance` | sensor_msgs/Range | sensor | REP 117: +inf clear, NaN unknown, finite obstacle |
| sub | `/skybeta/state` | std_msgs/String (JSON) | sensor | Via SkybetaBackend; DryRunBackend also reads it, read-only |
| pub | `/dh/pose_map` | geometry_msgs/PoseStamped | 10 | frame `course`; stamp = odometry stamp; published whenever aligned and fresh |
| pub | `/dh/plan` | nav_msgs/Path | 1 | frame `course` |
| pub | `/dh/mission_state` | std_msgs/String | 10 | WAIT_START, TAKEOFF, COURSE, LAND, LANDED, RETURN, RETURN_LAND, DONE, ABORTED |
| pub | `/dh/health` | std_msgs/String | 10 | `"<OK\|DEGRADED\|HOVER> fc=<state> <reason>"` |
| pub | `/dh/dry_run/cmd_vel`, `/dh/dry_run/events` | TwistStamped, String | 10 | **Dry run only** |
| pub | `/skybeta/setpoint/twist`, `/skybeta/setpoint/pose` | TwistStamped, PoseStamped | 10 | **Only with `dry_run:=false`**, backend skybeta |
| cli | `/skybeta/hold`, `/skybeta/request_offboard` | std_srvs/Trigger | — | **Only with `dry_run:=false`**. `request_offboard` is never called by the skybeta path |
| pub/sub | `/fmu/in/{offboard_control_mode,trajectory_setpoint,vehicle_command}`, `/fmu/out/{vehicle_status_v1,vehicle_land_detected}` | px4_msgs | PX4_QOS (best effort, transient local, depth 1) | `backend:=px4`, **SITL only** |

**`dh_localizer`** (`localizer_node.py`)
- Subscribes:
  - `/dh/pose_map` (PoseStamped, depth 50), into a 3 s `PoseHistory`;
  - `scan_topic` (default `/dh/stereo_scan`, LaserScan, sensor QoS).
- Publishes:
  - `/dh/map_fix` (PoseWithCovarianceStamped, frame `course`, stamp = scan stamp);
  - `/dh/loc_health` (Float32MultiArray `[ok, inlier_ratio, rms, min_eig, latency_s]`).
- Uses a tf2 listener when a scan arrives in a frame other than `base_frame`.

**`dh_stereo_scan`** (`stereo_scan_node.py`)
- Subscribes:
  - `/oak/{left,right}/image_raw` (Image, sensor QoS) through `message_filters.ApproximateTimeSynchronizer` (queue 4, slop `max_pair_dt` 5 ms);
  - `/oak/{left,right}/camera_info` (sensor QoS).
- Publishes:
  - `/dh/stereo_scan` (LaserScan, frame `base_link`, image stamp; per-beam **median**);
  - `/dh/forward_clearance` (Range, conservative 10th-percentile nearest return).
- Uses a tf2 listener, only for stereo extrinsics when camera_info lacks a baseline.

**`dh_line`** (`line_node.py`)
- Subscribes: `/oak/left/image_raw` and `/oak/left/camera_info` (sensor QoS); `/dh/pose_map` (height).
- Publishes: `/dh/line_obs` (Vector3Stamped).

**`dh_sitl_odom`** (`sitl_odom_node.py`, SITL only): `/fmu/out/vehicle_odometry` → `/dh/sim_odom`.

**Frames:**
- `course`: the map frame of the OBJ (m, +X east, +Y north, Z up, origin at the floor centre).
- `odom`: cuVSLAM, ENU-labelled but axes = the vehicle's axes at SLAM start.
- `base_link`: body FLU; scans and clearance are expressed here.
- Camera optical frames: names come from the camera_info headers.

`course←odom` lives only inside `MapAlignment`. **We do not broadcast it on TF** (possible improvement for RViz).

## 4. Perception (OpenCV)

**Stereo** (`stereo_scan_node.py`, geometry in `dh_core/camera_geom.py`):
- **Input checks:**
  - encodings mono8/16, bgr8/rgb8/bgra8/rgba8; anything else is rejected;
  - camera_info must have intrinsics;
  - left/right stamps must match within 5 ms;
  - image size must match camera_info;
  - the image must not be too dark (mean < 12).
- **Rectification** (`rectify_source: auto`), in order:
  1. camera_info, if `P_right[0,3] ≠ 0`;
  2. `cv2.stereoRectify` from the left→right TF;
  3. an assumed 0.075 m baseline (logged warning; 15–40 % depth error if the pair is rotated 1°).
- **Depth:** SGBM at `scale` 0.5 (512×320), 64 disparities, so the minimum measurable depth is about 0.33 m (closer reads NaN). Points are rotated back by R1ᵀ and then into the body frame (pitch from `camera_pitch_deg`).
- **Pseudo-scan:** `points_to_scan` (81 beams, ±0.15 m height band). `scan_validity` gives REP-117 output:
  - +inf = beam has data, nothing within `max_range`;
  - NaN = no data.
- **Forward clearance** (±15°): an obstacle is reported even with poor coverage; "+inf clear" needs ≥ 60 % of the forward beams to have data, otherwise NaN.

**Floor line** (`line_node.py`):
- morphological black-hat (kernel ≈ 2.2 × the expected line width) → largest component → floor projection → TLS line fit;
- confidence < 0.5 is ignored by the mission;
- the mission also has a 0.2 m innovation gate.

**Tests:** both are tested only on **rendered** images (`tools/sil/test_stereo_node.py`, `test_line_node.py`). **Not real-camera validation.**

## 5. Visual SLAM and localization

- **cuVSLAM** (organizers') provides odometry; we do not run our own SLAM.
- **Frame alignment:** `MapAlignment` builds the course←odom transform from the launch mark and `launch_yaw_map` (default π/2, facing north). It is set at `/dh/start`, or at the first odometry with `align_mode:=first_odom` (stationary tests).
- **Map matching** (`dh_core/localize.py`, `DistanceFieldMatcher`): DLL-style Gauss-Newton + Huber against a **signed** distance field of the course slice at 0.58–1.12 m, with observability-projected updates and yaw estimation off by default.
- **Localizer gating:**
  - jump gate 0.15 m, growing at 0.25 m/s since the last accepted fix;
  - jumps beyond 0.15 m need two consistent matches;
  - consistency override: 3 agreeing high-quality matches (inliers ≥ 0.85, rms ≤ 3 cm, jump ≤ 1 m) are accepted even outside the gate.
- **Time synchronisation (new this task):**
  - the prior for each scan is `PoseHistory.at(scan_stamp)`;
  - scans that are unstamped, stale (> 0.5 s behind the newest pose), in the future, or in an unknown frame are rejected and counted in `LocalizerNode.rejected`;
  - `/dh/map_fix` carries the **absolute** matched pose. The mission re-evaluates it against its own odometry history at that stamp (`_map_pose_at`), applies `correction_gain` 0.5, and rejects fixes older than the last applied one.
  - Line observations are treated the same way.
- **Health** (`dh_core/health.py`, `LocalizationHealth`):

| Condition | Response |
|---|---|
| Odometry stale > 0.4 s | HOVER |
| Odometry stale > 2 s | ABORT |
| VO lost outside a mapped dark zone | HOVER; ABORT after 3 s |
| Inside a dark zone for more than `max_dark_s` (4 s) | HOVER (**see bug B2**) |
| Odometry jump (> v·dt + 0.25 m) | 0.6 s pause |
| Map fix larger than 0.3 m | DEGRADED (40 % speed) for 1 s |
| Reported position std > 0.15 m | DEGRADED |

  HOVER means zero-velocity setpoints with the trajectory clock frozen.

> **Correction from the team (2026-11-06): the course tube is ILLUMINATED.** Do not assume total SLAM or stereo loss inside it. The current code and simulators still **assume a dark tube**:
> - `dark_zone_hints = ("tube", "tunnel")` in `route.py`;
> - the post-tube slowdown (`post_dark_scale`, `_blackout_governor`);
> - `max_dark_s` in health;
> - `expected_blind` relaxing NaN clearance inside the tube;
> - `run_sil.py`, which adds a velocity error and blinds the scans inside the tube;
> - `run_perception_loop.py`, which renders the tube at 3 % brightness.
>
> These are now **conservative assumptions to revisit**, not facts. The tube-axis anchoring in the planner is still sensible: it keeps the drone on the centreline of a narrow passage.

## 6. Planning and tracking

- **Planner** (default `planner: fillet`), with no hard-coded coordinates (everything is derived from the OBJ):
  - `dh_core/course.py`: OBJ slice → occupancy and EDT; detects apertures, the launch mark, the pad and the guideline.
  - `dh_core/planner_astar.py`: clearance-weighted 8-connected A* + greedy line-of-sight post-smoothing (not Theta*).
  - `dh_core/timing.py`, `fillet_path(..., transition="clothoid")`: clothoid-pair corners (continuous curvature), apex radius ≤ 0.6 m, shrunk for clearance. `max_tangent` protects a 0.6 m straight run on each side of the window.
  - `time_parameterize`: exact curvature, `a_total` 1.5 m/s² combined (friction circle), v_max 0.85 m/s.
  - `route.py`, `plan_mission`: route choice (window/gate/auto), tube-axis anchors, descent ramp, return leg.
  - Alternatives `planner: sfc` (`planner_sfc.py`) and `mpl` (`planner_mpl.py`) are slower and not recommended.
- **Tracking** (`dh_core/tracker.py`, `TrajectoryFollower`):
  - velocity feed-forward taken 0.3 s ahead (`v_lead`) + P (1.2), capped at `v_cap` 1.0;
  - a reference-clock governor slows at 0.15 m error and freezes at 0.35 m;
  - `speed_scale` = health × blackout × forward clearance.
- **Mission safety** (`_safety`):
  - forward-clearance scaling (horizontal);
  - software geofence;
  - ceiling below the walls except on the return.

  The forward-clearance policy (`ForwardClearanceMonitor`):
  - MISSING/STALE → speed 0;
  - INVALID → 35 %, then a 20 % creep after 1.5 s;
  - a finite reading consistent with the **map's** predicted range (`_map_range`) is a planned wall, ignored unless < 0.35 m;
  - unexpected obstacles scale the speed down from 0.9 m to 0.45 m.

## 7. Sky Beta / PX4 restrictions (organizer contract, from the research handoff §6.3; read from github.com/Artash-N/skybeta `fleet-ready`; the event image is unverified)

- Command only through `/skybeta/setpoint/{pose,twist}` (ENU, `odom` frame). Twists expire after 1 s.
- **Never publish to `/fmu/in/*`** (two OffboardControlMode publishers would fight).
- **Wait for `offboard_active` in `/skybeta/state` before commanding.** Implemented:
  - Sky Beta publishes **nothing** before then;
  - stale state (> 1 s) counts as inactive;
  - Offboard lost after being active → ABORTED, never resume (R5: human input ends the run).
- The interface never arms or changes mode; it clamps altitude to 0.3–3.0 m and twist to 1.5 m/s. PX4 `MPC_XY_VEL_MAX` = 1.0 on the bench drone.
- **The interface cannot land**, so CP5 (land + disarm) and the return bonus are impossible through it. The `px4` backend can do both but needs explicit organizer permission (organizer question 1).
- **Speed:** do not raise the limit without permission. The stricter simulator found no points gain from it anyway.

## 8. Changes this task (`b40ea66` + WIP checkpoint)

| Area | Change |
|---|---|
| Forward clearance | Float32 → `sensor_msgs/Range` with REP-117 semantics, image stamp, coverage-based "clear"; mission `ForwardClearanceMonitor` (missing/stale/invalid/obstacle), map-aware unexpected-obstacle policy, creep instead of a permanent stop on long invalid spells |
| Time sync | `PoseHistory`; localizer matches at the scan stamp; absolute `/dh/map_fix`; mission applies fixes and line observations at their stamps; out-of-order/unstamped/stale handling; pose_map stamped with the odometry stamp |
| Offboard gating | No Sky Beta output before `offboard_active`; trajectory clock starts only then; abort on loss; raw-PX4 landing exempt from Offboard gating (fixes a regression where LAND re-requested Offboard) |
| Localization health | `LocalizationHealth` (see §5); `/visual_slam/status` subscription (optional import); `/dh/health` topic |
| Dry run | `DryRunBackend` (default `dry_run: true`); `verify_isolation()` refuses to start if any publisher or client is under `/skybeta` or `/fmu`; forbidden output namespaces raise |
| Perception | Input validation (encodings, stamps, sizes, intrinsics, darkness); rectification rebuild on size change; no more aliasing of the input header; scan median for localization vs 10th percentile for clearance (fixed a ~3 cm short-range bias in fixes) |
| Line node | `Vector3Stamped` output with the image stamp; intrinsics check |
| ROS 2 integration | Sensor-data QoS on all sensor inputs; launch file (`dry_run`, `use_sim_time`, `align_mode`, `use_localizer`, `require_forward_clearance` tied to `use_stereo`); package.xml deps (tf2_ros, launch, launch_ros, builtin_interfaces, ament_index_python, numpy; scipy and px4_msgs no longer hard deps) |
| Simulators | fake_ros with real-shaped Time stamps, Range, PoseWithCovarianceStamped, publisher/client introspection, recorded service calls; `run_sil.py` stamped messages, 0.12 s scan latency, height-aware forward clearance, vectorized raycast |
| New tools | `tools/sil/test_integration.py` (12 tests), `tools/sil/run_perception_loop.py` (real OpenCV in the loop), `tools/hil/check_command_isolation.py`, `tools/hil/stationary_report.py`, `ros2_ws/src/dh_autonomy/test/test_ros_graph_dry_run.py` (launch_testing) |
| Docs | `docs/JETSON_VERIFICATION.md` (staged props-off HIL guide); this file |

Earlier this week (before this task):
- clothoid fillets;
- combined acceleration limit and window straight run;
- the stricter simulator (`run_sil.py` default model, `--legacy` for the old one);
- the localizer consistency override;
- the SFC and MPL planners;
- stereo TF rectification and the line black-hat fix.

## 9. Completed this task

Everything in §8. In addition:
- the full ray-cast SIL regression is clean;
- PX4 SITL with the return is clean;
- the Jetson verification guide is written.

## 10. Partially completed

- **Real-perception closed loop** (`run_perception_loop.py`): implemented and runs end to end, but the **full course does not complete**:
  - seeds 0 and 1: estimate error up to 1.4–1.5 m after the tube, 1–2 wall contacts, never reaches LAND;
  - the first 14 s are fine (mean error 6.6 cm, fix median 7 cm);
  - seed 0 hovered 80 ticks on "dead-reckoning in the dark zone";
  - map-fix p90 error is 0.56–0.87 m: some badly wrong fixes are accepted after the tube.

  This was being diagnosed when work stopped. Likely contributors:
  - (a) the renderer's dark tube (now known to be wrong: the tube is lit);
  - (b) bug B2;
  - (c) SGBM range bias (−1 to −3 %) and textureless or ambiguous scans letting the consistency override accept wrong fixes.
- **ROS 2 launch test** (`test_ros_graph_dry_run.py`): written, **never run** (no ROS 2 on the laptop).
- **HIL tools** (`tools/hil/*`): written and syntax-checked only; they need rclpy on the Jetson.
- **`analyze_bag.py`:** updated for `/dh/map_fix` and `/dh/health`; untested (no bag exists).

## 11. Known bugs, safety issues, open integration problems

| ID | Severity | Issue |
|---|---|---|
| B1 | High | Real-perception loop diverges after the tube (§10) |
| B2 | Medium | `LocalizationHealth.set_in_dark` counts time **in the dark-zone geometry**, not time with tracking lost. A slow passage through the (lit) tube > 4 s triggers HOVER even when VO is fine. **Fix:** start the timer only while VO is lost and/or stereo is INVALID |
| B3 | Medium | The tube is lit (team correction) but `expected_blind`, the blackout governor, `run_sil` tube error and the renderer assume darkness; re-derive from real sensor status |
| B4 | Medium | `run_perception_loop.py --route gate` produced numbers identical to `window` for seed 0 in one batch: check that the route parameter reaches the mission (`Node.PARAM_OVERRIDES` set before import) |
| B5 | Medium | Clock domain unverified: all sync assumes OAK image stamps, cuVSLAM stamps and the node clock share the ROS clock (guide C5/D8) |
| B6 | Medium | `vo_state` numbering and `/skybeta/state` fields (`offboard_active`, `frozen`) are taken from the organizers' guide, unverified; also whether a new twist un-freezes after `/skybeta/hold` |
| B7 | High (rules) | The Sky Beta interface cannot land or arm: CP5 and the return need the `px4` backend, i.e. organizer permission |
| B8 | High (rules) | **R9.1**: "your autonomy stack is built during the event". Most of this code was written before the event (2026-11-02 to 11-06); confirm with the organizers |
| B9 | Low | `course→odom` is not broadcast on TF; RViz cannot overlay the plan on the camera/odometry frames |
| B10 | Low | SGBM rate on the Orin CPU and real-wall texture/lighting are unmeasured; `robot_radius` and camera mount values are guesses (marked MEASURE in the config) |
| B11 | Low | `docs/planner_suite.md`, `docs/sil_results*.md` and the README results **predate this task's changes** (time sync, clearance policy); re-run before quoting them |
| B12 | Low | The stereo forward-clearance band is ±0.15 m around camera height; obstacles below or above it are invisible |

## 12. Test results (exact, on the WIP checkpoint code, 2026-11-06)

| Test | Command | Result | What it is |
|---|---|---|---|
| Unit | `python ros2_ws/src/dh_core/test/test_core.py` | **17/17 pass** | Pure Python |
| Integration (fake ROS bus) | `python tools/sil/test_integration.py` | **12/12 pass** | Real nodes, no DDS. Includes: dry-run isolation; no Sky Beta output before Offboard; abort on Offboard loss; stale state; odometry loss hover→abort; stale clearance freezes progress; **delayed scans: 2.9 cm error with capture stamps vs 25.5 cm when stamped at delivery**; localizer rejects unstamped/stale/future/wrong frame; out-of-order fix; VO loss hover→abort; silent start-up; stereo unknown ≠ clear on rendered pairs |
| Ray-cast SIL | `python tools/sil/run_sil.py --route {window,gate} --localizer --v_dark 0.85 --seed {0,1}` | **4/4 clean**: 23.07 / 23.15 s (window), 24.39 / 24.43 s (gate), 0 touches, min clearance 0.11 m | No OpenCV; skybeta gating on a fake bus |
| PX4 SITL path | `run_sil.py --backend px4 --with_return --localizer --v_dark 0.85 --seed 0` | DONE: disarm 26.31 s, pad error 4 cm, return 0.7 cm from the launch mark | Fake PX4 |
| Stereo node | `python tools/sil/test_stereo_node.py` | 1–3 % depth error with camera_info/TF rectification; 15–40 % with the bare-baseline fallback on a 1°-rotated pair | Rendered images |
| Line node | `python tools/sil/test_line_node.py` | 14/14 cases correct | Rendered images |
| Real-perception loop | `python tools/sil/run_perception_loop.py --seed {0,1}` | **Does not finish** (B1) | Rendered images through real OpenCV |
| numpy 1.26 / OpenCV 4.10 | (earlier this week) | Unit + SIL identical | Matches the organizers' pinned numpy < 2 |
| ROS 2 launch test | `colcon test` on the Jetson | **NOT RUN** | — |
| Any real camera / sensor / Jetson test | — | **NOT DONE** | Hardware stage not authorised yet |

Simulator caveats:
- the velocity loop is first-order plus an acceleration limit, with no full attitude dynamics;
- odometry is truth plus random-walk drift;
- the tube is modelled as dark, which is wrong (B3).

## 13. Dependencies and environment

- **Target, per the organizers' repo; verify on the Jetson:**
  - Jetson Orin Nano Super, L4T R36.5.0;
  - ROS 2 **Humble** in Docker on an Isaac ROS 3.2 base image;
  - PX4 v1.16.2, MicroXRCEAgent 2.4.3;
  - depthai 3.10 + depthai_ros_driver_v3;
  - isaac_ros_visual_slam 3.2.6;
  - numpy < 2.
- **Team code:** `~/team_ws/src` (bind-mounted at `/root/team_ws`); `px4_msgs` must not be in the team workspace.
- **Python deps:**
  - numpy and OpenCV (in the image);
  - scipy optional;
  - `message_filters` and `tf2_ros` required;
  - `isaac_ros_visual_slam_interfaces` optional;
  - px4_msgs only for SITL.
- **Laptop (Windows, no ROS):** Python 3.12, numpy, matplotlib, opencv-python-headless. Run the tools with `DH_COURSE_OBJ=<path to dronehacks-course-v3.obj>`.

## 14. Files to inspect first

1. `ros2_ws/src/dh_autonomy/dh_autonomy/mission_node.py`: state machine, gating, health and safety integration.
2. `ros2_ws/src/dh_core/dh_core/health.py`: `PoseHistory`, `ForwardClearanceMonitor`, `LocalizationHealth`.
3. `ros2_ws/src/dh_autonomy/dh_autonomy/backends.py`: `DryRunBackend`, `verify_isolation`, `SkybetaBackend`, `Px4Backend`.
4. `ros2_ws/src/dh_autonomy/dh_autonomy/localizer_node.py` and `dh_core/localize.py`.
5. `ros2_ws/src/dh_autonomy/dh_autonomy/stereo_scan_node.py` and `dh_core/camera_geom.py`.
6. `ros2_ws/src/dh_core/dh_core/route.py` and `timing.py`.
7. `tools/sil/test_integration.py`, `tools/sil/run_perception_loop.py`, `tools/sil/run_sil.py`.
8. `docs/JETSON_VERIFICATION.md`, `docs/rules_audit.md`, `STRATEGY.md`, the research handoff §3, §6 and §7.

## 15. Prioritised next steps for Codex

1. **B2 + B3:** make "blind zone" handling data-driven (VO status / stereo validity), not geometry; stop rendering and simulating the tube as dark; keep tube-axis anchoring.
2. **B1:** re-run `run_perception_loop.py` after step 1. If it still diverges, log every accepted `/dh/map_fix` against truth (already computed as `fix_err_*`). Tighten the consistency override (e.g. require scan coverage on two non-parallel walls, i.e. observability `min_eig`) and possibly correct for SGBM bias.
3. **B4:** confirm `--route gate` takes effect in the perception loop.
4. Re-run `tools/sil/compare_suite.py` and refresh `docs/planner_suite.md` and the README results (B11).
5. **Before any hardware session** (needs the user's and organizers' authorisation): follow `docs/JETSON_VERIFICATION.md` stages A–F, all props-off, sensor-only launch with no PX4 bridge, and dry run. Resolve B5 and B6 from the real topics; measure the camera mount and `robot_radius`.
6. Run `test_ros_graph_dry_run.py` in an isolated domain on the Jetson (`ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1`).
7. Optional: broadcast `course→odom` on TF (B9); a 2-D LiDAR as an auxiliary sensor (see STRATEGY).
8. Organizer questions: landing/arm authority (B7), R9.1 (B8), whether the camera pipeline may change. See `docs/organizer_questions.md`.

## 16. Recovery: how Codex gets this code

The repository currently lives in a temporary session folder. Two durable copies are made at handoff:
- `C:\Users\saket\Downloads\dronehacks_autonomy.bundle`: a git bundle with **all branches and history**;
- `C:\Users\saket\Downloads\CODEX_HANDOFF.md`: this file.

Restore and continue on `saketh/autonomy` (no force-push, never push to `main`):

```bash
git clone C:/Users/saket/Downloads/dronehacks_autonomy.bundle dronehacks_autonomy
```

```bash
cd dronehacks_autonomy && git checkout saketh/autonomy && git log --oneline
```

To publish to the team's GitHub repository, add the team remote. If `saketh/autonomy` already exists there with other history, **do not force**: fetch it, compare, and merge, or push under a new branch name.

```bash
git remote add origin <team-repo-url> && git fetch origin
```

```bash
git push -u origin saketh/autonomy
```
