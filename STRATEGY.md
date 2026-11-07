# DroneHacks 2026 — Points Strategy and Engineering Rationale

This document answers four questions: how to get the most points, whether to optimize for speed, whether hard-coding is worth it, and which waypoints, algorithms and papers to use. Every number is either from the official rules (see `../stanford_drone_hacks_research_handoff.md`) or produced by the code in this repository; each claim says which. The simulation results are **screening evidence, not proof**. Their noise levels are assumptions and must be re-tuned against real flight logs.

---

## 1. Where the points are (official scoring, verified by `dh_core/scoring.py` unit tests)

```
finished run (below walls) = 35 + 20 * T*/T  (+5 if through the window)    x1.25 if the return is completed
unfinished run             = sum(reached checkpoints 4/7/10/7/7) + 7 * T*_last/T_last   (always <= 35)
total = run (<=75 with return) + presentation (25) + generality (15)
```

Run points for a few finish times (computed with `scoring.py`):

| our finish T (s) | fastest T* (s) | Gate B | window | Gate B + return | window + return |
|---|---|---|---|---|---|
| 26 | 20 | 50.4 | 55.4 | 63.0 | **69.2** |
| 30 | 20 | 48.3 | 53.3 | 60.4 | 66.7 |
| 35 | 20 | 46.4 | 51.4 | 58.0 | 64.3 |
| 45 | 20 | 43.9 | 48.9 | 54.9 | 61.1 |
| 26 | 15 | 46.5 | 51.5 | 58.2 | 64.4 |

**What maximizes points, in order:**
1. **Finish.** Any finish below the walls scores more than any unfinished run.
2. **The return** (×1.25, up to +15 points). A window+return run at 45 s (61.1) beats a window run at 26 s without the return (55.4). Per the rules, the clock stops at the first disarm on the pad, so the return does not slow the scored time; it only has to fit inside 3 minutes.
3. **The window** (+5, or +6.25 with the return). It is also the *shorter* route here (15.6 m vs 16.1 m planned).
4. **Speed.** Going from 35 s to 26 s is worth about 4 points. Speed is the least valuable lever, and it is capped by PX4 anyway (§2).
5. **Generality (15) and presentation (25).** These are 40 points earned off the course. A planner that derives everything from the course file is worth more here than any hard-coded speed trick (§3).

## 2. Speed: what is achievable and what limits it

- **Hard ceiling:** the bench drone's PX4 parameter `MPC_XY_VEL_MAX` is 1.0 m/s (organizers' repo). PX4 applies it to offboard setpoints. The organizers' interface also caps twist at 1.5 m/s, but PX4's 1.0 m/s cap binds first.
- **Planned route length** (auto-planned from the official mesh, `tools/plan_course.py`; includes the straight tube-axis passage and the landing descent ramp): window **16.20 m / 20.2 s**, Gate B **16.75 m / 21.5 s**, at a planned 0.85 m/s with 1.5 m/s² limits. Including takeoff, landing and disarm, the SIL measurement with the px4 backend is about **25.0 s** from start to disarm (`tools/sil/run_sil.py`).
- **Why plan at 0.85, not 1.0:** if the reference already flies at the PX4 cap, the controller has no authority left to correct errors. Monte-Carlo runs (`docs/strategy_results_headroom.md`) show this is noisy at N = 50. The clean win came from **velocity lead** (§4.3), not from more speed.
- **Gain if the organizers allow raising `MPC_XY_VEL_MAX`:** planning at 1.25–1.5 m/s cut planned time by only ~2–3 s, and the simple Monte-Carlo model shows a sharp rise in touches (`docs/strategy_results.md`). **Not worth it** until localization is proven.
- **Cheap speed wins already in the code** (SIL finish went from 27.3 s to about 25.0 s):
  - disarm the moment the land detector is confident (saves ~1.5 s versus PX4's 2 s auto-disarm);
  - descend during the last 1.2 m before the pad, so PX4's slow landing crawl (MPC_LAND_CRWL 0.3 m/s below MPC_LAND_ALT3 = 1 m) starts from 0.3 m instead of 0.85 m. The ramp is checked against low obstacles such as the boulders. Landing-to-disarm went from ~3.6 s to ~1.7 s in SIL;
  - take off at 0.9 m/s (MPC_Z_VEL_MAX_UP is 1.0);
  - **don't slow down in the dark tube**: pose error grows with *time* in the dark, so flying through at full speed is safer (SIL: v_dark 0.85 vs 0.6 removed a tunnel failure; see §4.2);
  - corner fillets plus a time-optimal speed profile instead of stop-and-go waypoints.

## 3. Is hard-coding worth it? No, and here is the evidence

| | Hard-coded waypoints/timings | Map-derived plan (this repo) |
|---|---|---|
| Run time | same geometry → same time | 15.55 m / 19.8 s computed automatically in < 1 s |
| Robustness to "the layout may change slightly" | breaks silently | re-plan from the updated official file in < 1 s |
| Generality (15 pts) | AI code review flags fixed waypoints, routes and timings (R9.5) | nothing course-specific in code; course = data file |
| Presentation | "we typed coordinates" | A*, time-optimal profile, scan-to-map re-localization, MPL comparison, SIL tests |

What the code takes as *inputs* (legitimate, R6.4 allows a map from the published layout):
- the official course file;
- the launch heading (how the team places the drone);
- physical parameters: cruise height, drone radius, speed limits.

Group-name hints (`launch`, `pad`, `guide`, `tube`) only label features; the planner itself discovers the maze path, the tunnel as the only way through the barrier, Gate B and the window aperture from geometry (`dh_core/course.py`, `dh_core/route.py`). The window is found by detecting a rectangular hole in any wall mesh, not by name.

**Generality evidence** (`tools/generality_test.py`, `docs/generality_results.md`, `docs/figures/generality.png`): the unchanged planner was run on modified versions of the course and succeeded on all 15:
- 12 random perturbations (every wall, boulder and the tube moved up to ±15 cm and rotated up to ±5°; launch and pad moved up to ±10 cm): planned in 19.4–21.3 s, minimum clearance ≥ 0.30 m;
- an east–west mirrored layout;
- a version with the window bricked up: the planner found no aperture and switched itself to Gate B.

That is exactly the "how would your software work on a different course" question the judges score. Put the figure in the presentation.

The auto-generated waypoints are exported for reference in `docs/waypoints_v3.md`. Use them for tape-measure checks on site and for slides; the flight code never reads them.

**Recommendation:** never type waypoints. If a hand-tuned value is ever needed on site (e.g., a slower speed in one zone), express it as a *rule* ("slow where clearance < X"; "full speed in dark zones"), not as coordinates.

## 4. Biggest risks and what the code does about them

Evidence from three tools:
- `tools/evaluate_strategies.py` and `tools/evaluate_headroom.py`: simple Monte-Carlo models;
- `tools/evaluate_aiding.py`: the real matcher in the loop;
- `tools/sil/run_sil.py` and `batch_sil.py`: the *actual ROS node code* running against a simulated drone.

### 4.1 Pose drift in the dark tube (dominant risk)

- cuVSLAM's documented fallback without IMU fusion coasts for about 0.5 s; the tube transit lasts 2–3 s.
- **Result:** in the basic Monte Carlo with no correction after the tunnel, crash rates were **42–95 %**. With the estimate re-anchored to the map after the tunnel, they fell to **5–23 %** at 0.75–1.0 m/s (`docs/strategy_results*.md`).
- **Code:** `localizer` node — scan-to-map registration against the official course (§5). It resumes as soon as the camera sees walls again. A widening jump gate plus a two-match confirmation lets it accept the large but correct post-tunnel fix while rejecting single bad matches.
- **Auxiliary sensor with the highest expected value:** a small 360° 2D LiDAR that works in the dark, e.g. the YDLIDAR T-mini Plus ($79–95, 45 g, Class 1; it fits rule 4). In the closed-loop matcher experiment (`docs/strategy_results_aiding.md`, partial run), crash rates were:
  - no aiding: 47 %
  - forward stereo scan: 10 %
  - 360° LiDAR: **0 %** (touches 7 %)

  The same `localizer` node accepts its `/scan` (`scan_topic:=/scan`, `sensor_x:=` its mounting offset). It is also strong presentation material, since judges score "how well auxiliary sensors are used".

### 4.2 Time and geometry in the dark

- **Full speed through the tube.** The tube is about 1.5 m long, so flying through at full speed spends less time accumulating error. In the 10-seed SIL batch (`docs/sil_results.md`), tube speed 0.85 m/s beat 0.6 m/s in every route/localizer combination. Example: window route with the localizer, 9/10 clean vs 8/10.
- **Fly the tube's axis, aligned before entry.** Remaining SIL contacts were at the tube's south rim: the path had cut diagonally through the tube, wasting lateral margin where the estimate is worst. The planner now adds entry/exit anchors on the long axis of every dark zone (0.35 m outside each mouth). The path is straight down the centre (x = 2.326 m, axis 2.311 m; ≈0.49 m margin per side for drift), and the failing seed became clean.
- **Creep after a blackout until re-localized.** After leaving a dark zone, the mission slows to 25 % until the localizer delivers a fresh map fix, or 2.5 s pass (`post_dark_scale`, `post_dark_timeout`).
- **Final SIL batch with all fixes in** (`docs/sil_results_final.md`, 10 seeds each, tube at 0.85 m/s):

  | route | localizer | clean runs | course time | pad error |
  |---|---|---|---|---|
  | window | on | **10/10** | 22.2 s | 1.9 cm |
  | window | off | 6/10 | 24.1 s | 22.8 cm |
  | gate | on | **10/10** | 23.5 s | 3.5 cm |
  | gate | off | 1/10 | 25.3 s | 22.7 cm |

  The localizer is what makes the dark tube survivable. Without it the drift picked up in the tube is never removed, and the run touches a wall or lands about 20 cm off the pad centre.

### 4.3 Velocity-loop lag in corners

- PX4's velocity loop lags (modelled as τ ≈ 0.3 s). With feed-forward taken at the current time, the drone cut outward at the hairpins and touched walls.
- **Fix:** take the velocity feed-forward 0.3 s ahead on the trajectory (`FollowerParams.v_lead`). In SIL this went from **4 touches to 0** on the same seed.

### 4.4 Frame alignment

- cuVSLAM's `odom` frame starts wherever the SLAM node started.
- `MapAlignment` fixes `course ← odom` from the launch mark and the known launch heading, either at the start command or at the first odometry sample.
- SIL deliberately uses an odom frame rotated 40° and offset 1.3 m, and the alignment recovers it exactly.

### 4.5 Control authority (rules vs the organizers' interface)

- The public `skybeta` interface never arms, changes mode or lands, and its 0.3 m altitude floor prevents setpoint landings. The rules require autonomous takeoff, land+disarm (CP5) and re-arm for the return.
- The code supports both cases:
  - `backend:=skybeta`: course flying works. Landing falls back to hovering 0.35 m over the pad and logs a warning.
  - `backend:=px4`: full autonomy, including land, disarm, re-arm and the return.
- **Ask the organizers which is allowed** (handoff Q-P2). The px4 backend must not run alongside their flight_interface (two OffboardControlMode publishers).

### 4.6 Rule 6.4 and SLAM maps

The organizers' cuVSLAM runs in SLAM mode. **Restart the SLAM node before each scored run** (fresh map), or ask whether odometry-only mode is preferred. The localizer uses only the published course model, never data from earlier runs.

## 5. Algorithms in this repository (and the papers they come from)

| Function | Implementation | Source / paper | Status |
|---|---|---|---|
| Course model from mesh | `dh_core/course.py`: OBJ loader, band slicing, aperture (hole) detection, guideline centreline | — | tested on the official v3 OBJ |
| Occupancy distance field | exact EDT (SciPy if present, else Felzenszwalb–Huttenlocher) | Felzenszwalb & Huttenlocher, *Theory of Computing* 2012 | unit-tested vs brute force |
| Global path | clearance-weighted 8-connected A* + greedy line-of-sight post-smoothing | Hart, Nilsson & Raphael 1968; post-smoothing as in the A* PS baseline of Nash et al. 2007 (not Theta*) | plans both routes in < 1 s |
| Smoothing + timing | clothoid-pair corner fillets (continuous curvature; circular arcs still available with `corner: arc`), a protected straight run through the window, then a forward/backward speed profile on the exact curvature with a combined 1.5 m/s² acceleration limit (plus clearance and dark-zone caps) | arc fillets: Beard & McLain 2012; clothoids: Fraichard & Scheuer 2004; speed profile: Bobrow et al. 1985, Shin & McKay 1985 (cf. TOPP-RA) | unit-tested. In the stricter simulator: arcs 2 touches every run, clothoids 0 (6/6), +0.6 s |
| **Alternative planner** | `dh_core/planner_mpl.py`: search-based motion primitives, J = ∫‖u‖² + ρT, guided through A* waypoints; **LQMT heuristic h₂** (Prop. 3, free-end form) | **Liu et al., IROS 2017 (arXiv 1709.05401); sikang/motion_primitive_library** (professor's paper) | Works. h₂ gives the same optimum with 2.7–3.5× fewer expansions. Flies 24.6 s against 20.2 s (window), so it is kept as an option and presentation material |
| **Alternative planner** | `dh_core/planner_sfc.py`: **Safe Flight Corridor** (ellipse → polygon decomposition, robot-radius shrink) + min-jerk QP (own dense ADMM solver) + exact time rescaling; `planner: sfc` | **Liu et al., RA-L 2017; sikang/DecompUtil** (professor's paper) | Corridor safety is unit-tested; SIL 3/3 clean. Flies 27.2 s against 20.2 s (window) and bends inside the dark tube, so not the default (`docs/paper_notes.md`) |
| Tracking | feed-forward with 0.3 s lead + P, reference-clock governor (pauses the reference when the error grows), yaw along travel | standard; lead compensates the PX4 velocity-loop lag | SIL-verified |
| Re-localization | DLL-style direct registration of a scan to a **signed** distance field of the official course, Gauss–Newton + Huber, observability-projected update (corrects only directions the scan constrains) | Caballero & Merino, "DLL: Direct LIDAR Localization", IROS 2021 | offline + SIL-verified |
| Depth from the raw stereo pair | `stereo_scan_node`: OpenCV StereoSGBM on a 512×320 rectified pair → horizontal-band pseudo-scan + forward clearance. Rectification comes from camera_info if it has stereo calibration, else `cv2.stereoRectify` from the left→right TF, else an assumed 7.5 cm baseline | Hirschmüller, SGM, TPAMI 2008; Bouguet rectification (OpenCV) | **End-to-end tested on rendered stereo pairs** (`docs/stereo_node_test.md`): wall distance within 1–3 % out to 2.5 m. A 1° mis-rotated pair rectified with the bare-baseline assumption reads 15–40 % short, so the TF path matters. Untested on hardware |
| Floor-line cue | `line_node`: morphological black-hat (thin dark stripes only) → floor projection → TLS line fit → lateral correction only along the line normal, with a 0.2 m innovation gate | Brandão et al., ICINCO 2015 (vision line following for UAVs); Soille, *Morphological Image Analysis* (top-hat) | **End-to-end tested on rendered corridors** (`docs/line_node_test.md`): ≤ 2 mm / 0.2° error over 10 cases; no false lines in 4 line-free scenes. That test found and fixed a dark-wall false positive. Untested on hardware |
| Mission logic | `mission_node`: align → takeoff → course → land → disarm → (re-arm → return → land) with safety governor and software geofence | — | SIL-verified, both backends |

**Read-only recommendations from the professor's list** (details in `docs/research_notes.md`):
- **ORB-SLAM3** (stereo-inertial): GPL-3, CPU-heavy, and no proven Orin Nano + Humble stereo-inertial build. It would struggle in the dark tube just like cuVSLAM. Keep the organizers' cuVSLAM.
- **VGGT**: ~1.2 B parameters, far too heavy for real time on an Orin Nano 8 GB. Usable offline on a laptop for analysing logs; not on the drone.
- **MPL / mpl_ros**: ROS 1 only, so it was re-implemented in simplified form (above).
- **Swift (Kaufmann et al., Nature 2023)**: deep-RL racing. Not trainable before the event, and our interface takes velocity setpoints, not body rates. Two ideas carry over (`docs/paper_notes.md`):
  - after the first real flights, measure the velocity-tracking lag and bias from the bag and put those residuals into the SIL and `v_lead`;
  - keep the camera looking where perception needs it, as the tube-axis anchors already do.

## 6. Recommended configuration for the first scored block

These settings follow from the evidence above. Change them only after on-site tests.
- `route: window`. Use `gate` if the window fails twice in practice.
- `v_max: 0.85`, `v_dark: 0.85`, `cruise_z: 0.85`, `robot_radius`: measured half-span + 3 cm.
- Localizer **on**. In the final SIL it took clean runs from 6/10 to 10/10 (window) and from 1/10 to 10/10 (gate). Turn it off only if `/dh/stereo_scan` is clearly wrong against the real walls and no LiDAR is available. In that case also drop `v_dark` and expect touches after the tube.
- `enable_return: true` **only** if `backend:=px4` (or an organizer-provided equivalent) is allowed.
- Log every flight (`host/run.sh bag`) for the presentation's "evidence of testing and iteration". Never feed logs back as run inputs.

## 7. On-site checklist (first hours with the drone)

1. Measure the span (→ `robot_radius`), camera pitch and offset (→ `camera_pitch_deg`, `camera_x`) and the launch heading.
2. Confirm the topics: `/visual_slam/tracking/odometry`, `/oak/{left,right}/image_raw`, `/oak/*/camera_info`. Then check stereo calibration:
   - Does `P` of the right camera have a non-zero `P[3]` (a baseline)?
   - If not, does `ros2 run tf2_ros tf2_echo <right optical frame> <left optical frame>` (frame names from the camera_info headers) show about 7.5 cm?
   - The `stereo_scan` log line `rectification ready (camera_info|tf|baseline)` says which source it used. `baseline` means depth may be biased, so hold a board at a measured 1.0 m and check.
3. Run `stereo_scan` and look at `/dh/stereo_scan` in RViz against the walls. Run `line` over the guideline and echo `/dh/line_obs`.
4. Props off: start the mission with the drone on the launch mark and check `/dh/pose_map` while carrying it around the maze.
5. Second net: takeoff → hover → short leg → land. Then the full course at `v_max 0.5`, then 0.85.
6. Ask the organizers: arming/landing/return authority (Q-P2); whether `MPC_XY_VEL_MAX` may change; whether camera pipeline changes are allowed (depth); SLAM map reset between runs.

> **Rule note (R9.1):** "Your autonomy stack is built during the event." Before using any of this code at DroneHacks, confirm with the organizers that pre-written code is acceptable, or treat it as a reference and rebuild it during the event. The repository is MIT-licensed, and every external idea is credited above as R9.1 requires.
