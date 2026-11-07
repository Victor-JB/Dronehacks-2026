# Local validation record — 2026-11-06

All evidence in this document is from the Windows laptop. No SSH, Jetson, physical drone, Gazebo, actual PX4 process or ROS 2 DDS graph was accessed or run. Physical flight is prohibited.

## Preserved source and ownership

- Claude bundle history retained: `02f6209` -> `b40ea66` -> `46925ee`.
- Read the clean existing team checkout at `C:/Users/saket/OneDrive/Documents/GitHub/Dronehacks-2026`, main and origin/main both `e9532fa`. Read root AGENTS.md, AI_CONTEXT.md, CLAUDE.md, native onboard examples/docs, notebook inventory and all five shared context files. Its files and branches remain unchanged.
- Merge `4e2866b` joins both histories on `saketh/autonomy`. Team root README preserved with an autonomy link; the original autonomy README is in `docs/AUTONOMY_README.md`.
- Runtime/test implementation commit: `893dbfd`. Documentation/status follow in a separate commit. No push, credential edit, force-push, branch overwrite or Gazebo worktree access occurred. Only Saketh's shared context file was updated.
- Local organizer reference: [Sky Beta fleet-ready 7efe7e4](https://github.com/Artash-N/skybeta/tree/7efe7e45080cdf9fa05814824ac8bfe7eca3ef9d). Read competitor guide, executable example, flight interface, auto-Offboard state machine and camera/VO launch/config.
- Official estimator reference: [Isaac ROS release-3.2 e31f4cc](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/tree/e31f4cc1d41a329a01946e5fe63669f8b15da677). `VisualSlamStatus` has a Header; implementation copies the input header and reports 1=success, 2=failure; the message defines 0=unknown. This verifies the source contract, not the event image.

## Implemented behavior

Missing, stale, invalid, unstamped, future-stamped or malformed clearance cannot enable blind motion. No tube-geometry exemption or timed creep remains. The mission sends zero velocity/yaw and freezes its follower in every phase when clearance cannot be trusted. Disabling the stereo producer no longer silently disables clearance requirements; an explicit offline setting is required.

Camera fixes and line observations are evaluated against bracketed historical odometry, with no future extrapolation. The line detector also uses capture-time height. Scan TF and optical-to-body TF use capture stamps. The map's expected forward range is queried at the clearance capture pose; if there is no bracketing pose it holds, and a changed viewing direction also holds. Duplicate/out-of-order data cannot refresh sensor health. Bad future clearance stamps do not prevent recovery on subsequent valid stamps.

Default camera mount comes from measured TF. Auto stereo rectification waits for calibrated CameraInfo or inter-camera TF instead of assuming a nominal 75 mm baseline. Invalid intrinsics, buffers, strides, frame/dimension mismatches, dark images, mismatched pairs and OpenCV failures yield unknown clearance. Explicit nominal-baseline mode remains available only for offline negative-control experiments.

Fresh VO success is required by default. Unknown/lost/stale VO holds then aborts; odometry health uses capture and receipt time, validates frame/child frame, finite values, quaternion/covariance and detects XY/Z/yaw discontinuities. A discontinuity holds until three accepted map fixes after a 0.6 s pause, then resumes at 40% for 1 s; without recovery it aborts after 3 s. High covariance slows; excessive uncertainty holds. Absolute fixes have independent final 0.30 m / 0.15 rad innovation limits.

Sky Beta source has a startup inconsistency: guide text says wait before commanding, but executable auto-Offboard requires a recent team setpoint and the example publishes a readiness target. The implementation permits **only zero twists** before fresh authority (after start/readiness); trajectory motion starts after Offboard. Pose targets are not sent before authority, avoiding the organizer altitude-floor lift. A frozen interface is never unfrozen by a new command. Invalid/stale authority or takeover after activation aborts permanently. No Sky Beta arm/mode/land commands are used. Dry run creates no flight-control publishers/clients.

A* + LOS smoothing + clothoid generation, speed parameterization and the follower algorithm remain the default and were not replaced. Tube-axis route anchors remain; geometry no longer indicates sensor loss. The tube is illuminated in nominal Python simulations; `--blackout_tube` is explicit fault injection.

## Test classification

Environment: Python 3.12.5, numpy 2.1.3, OpenCV 5.0.0, no local ROS 2 installation. Pytest 8.3.5 was installed into task scratch only for preserved team/organizer tests. Target-library checks also run in a task-local Windows environment with numpy 1.26.4 and OpenCV 4.10.0; exact ARM/ROS/Humble compatibility remains pending. The additional dependency results are recorded below.

Course OBJ SHA-256: `e610bcf9666422e2a0a78cb46f542fc8c4d5a7ef9b15f6123f1abe40f5b8e9e1`. This course is external input, not committed or hard-coded into flight algorithms.

| Classification | Command / evidence | Outcome |
|---|---|---|
| Unit tested, dh_core | `python ros2_ws/src/dh_core/test/test_core.py` | 17 pass |
| Unit tested, preserved team code | `python -m pytest -p no:cacheprovider tests -q` | 30 pass; team mission code unmodified |
| Unit tested, organizer reference | pytest auto_offboard, envelope, frames, topics at pinned source | 88 pass, 2 skipped (real bridge / installed px4_msgs unavailable) |
| Node integration on fake bus | `python tools/sil/test_integration.py` | 12 pass; no DDS/QoS/executor validation |
| Additional safety/OpenCV regressions on fake bus | `python tools/sil/test_safety_regressions.py` | 12 pass; includes actual SGBM unknown -> mission hold, capture-height line processing, timestamped optical TF, height-aware renderer, replay/frame/authority/innovation guards |
| Stereo processing, synthetic images | `python tools/sil/test_stereo_node.py --out <report>` | 12 cases; calibrated/TF and aligned nominal cases assert <5% error. Deliberately miscalibrated negative controls show 15-39% bias and are not qualification |
| Floor-line processing, synthetic images | `python tools/sil/test_line_node.py --out <report>` | 14 cases correct, now assertions rather than report-only exit success |
| Syntax/static checks | compileall on packages/tools, XML parse, `git diff --check` | Recorded with final evidence |
| Actual ROS 2 integration | `test_ros_graph_dry_run.py`, new `test_ros_perception_dry_run.py` | **NOT RUN**. The latter runs the actual launch + synthetic Image/CameraInfo + TF + DDS + OpenCV + mission isolation. Both require localhost-only domain 77 |
| Jetson hardware tested | — | **NOT TESTED** |
| Real camera, cuVSLAM under motion, FC/Gazebo, flight/landing | — | **NOT TESTED** |

Baseline before changes: 17 unit / 12 fake-bus tests passed. Delayed-scan test retains capture-time vs arrival-time negative control: approximately 2.9 cm vs 25.5 cm in the baseline; see final integration log for current numbers.

## Ray-cast Python SIL, final capture-time gating

Each command: `python tools/sil/run_sil.py --route <route> --localizer --v_dark 0.85 --seed <seed>`. This bypasses OpenCV and models an ideal ray-cast sensor, noisy odometry and a simple acceleration/velocity response. `LAND` in the Sky Beta path is the pad-hover state, not touchdown/disarm.

| Route | Seed | Pad state time (s) | Contacts | Pad XY error (m) |
|---|---:|---:|---:|---:|
| window | 0 | 29.19 | 0 | 0.010 |
| window | 1 | 29.27 | 0 | 0.005 |
| gate | 0 | 30.51 | 0 | 0.024 |
| gate | 1 | 30.59 | 0 | 0.005 |

Minimum simulated clearance was 0.043-0.052 m after the simulator's assumed 0.24 m radius. These small margins need measurement and motion validation; they are not hardware safety clearance. Strict no-extrapolation clearance gating introduces temporary holds and slows these 10 Hz sensor runs relative to earlier benchmarks.

Fake-PX4 return: `run_sil.py --backend px4 --with_return --localizer --v_dark 0.85 --seed 0` reaches DONE, zero contacts, first disarm at 37.83 s (36.83 s after start), pad error 0.039 m, return error 0.001 m. The final `pad_xy_error_m` field is measured after returning to the launch mark and must not be mistaken for the first landing error. This is a Python backend simulation, **not actual PX4 SITL**.

## Real OpenCV closed loop

Original failed evidence was retained. Removing the darkness assumption alone produced a safe hold around 13.9 s: SGBM reported 0.41-0.43 m where the flight-height map predicted 0.75-0.83 m, with pose error below 6.6 cm. The renderer had projected all obstacle heights into a single 2-D footprint and extruded it to the ceiling, turning low obstacles into tall walls and filling apertures. The sensor safety stop was appropriate for those images.

The corrected offline renderer uses height slices and 3-D distance-field raycasting, tested to preserve free space above a low obstacle. It remains a voxel approximation of the mesh, not photographic realism or Gazebo. A 0.20 m body-center resting height is an explicit, unmeasured landing-gear assumption; the old zero-height camera was below the floor. No flight thresholds were weakened to make the images pass.

A diagnostic corrected-scene run reached pad hover in 22.91 s, zero contacts, 2.2 cm pad error, 11.1 cm peak pose error. The final four-run route/seed matrix is below. Routes report `route_used` and planned length (window 16.4506 m, gate 16.8006 m), verifying that `--route gate` is honored.

## Limitations and next boundary

- Real ROS message generation, DDS QoS matching, executors, launch execution, ARM dependencies and target numpy<2 have not been run. The prepared tests and stationary procedure are the next validation layer.
- Real camera calibration, timing, texture, illumination, beam coverage, Nano SGBM throughput and cuVSLAM drift are unmeasured. The pseudo-scan uses a limited height band; it cannot prove clearance above/below that band.
- Body mount/radius/resting height must be measured. No course-to-odom TF broadcaster was added; the alignment remains internal. The localizer intentionally rejects large/ambiguous corrections, so substantial drift/reset may abort instead of automatically recovering.
- A zero-command hover is conservative software behavior, not a guarantee of position hold if physical VIO fails. No physical dynamics or failsafe validation is implied.
- Sky Beta cannot land/disarm or re-arm for the return. Keep raw PX4 simulation-only. Current organizer code also requires a single flight interface launched separately; do not simply replace COMPETITOR_LAUNCH with this sensor-only dry-run launch and assume the hardware handover is configured.
- Legacy planner-comparison scores, dark-tube experiments and old README timings were marked historical. The planner comparison suite was not rerun because planner algorithms were unchanged; current default-planner regressions are above.
- Stop before the first SSH connection or hardware operation. Follow `docs/JETSON_VERIFICATION.md` only in a later explicitly authorized stationary session, props off and no flight battery. No flight or hand-carried test is included.

## Final OpenCV route/seed matrix

Commands: `python tools/sil/run_perception_loop.py --route <route> --seed <seed> --t_max 45 --out <jsonl>`. Default 5 Hz rendered 512x320 pairs, explicit synthetic mount/calibration, 0.20 m initial body height, illuminated tube. Actual ROS is replaced only at the boundary.

| Route | Seed | Pad hover time (s) | Contacts | Pad XY error (m) | Max pose error (m) |
|---|---:|---:|---:|---:|---:|
| window | 0 | 22.71 | 0 | 0.026 | 0.110 |
| gate | 0 | 24.07 | 0 | 0.031 | 0.110 |
| window | 1 | 22.59 | 0 | 0.022 | 0.105 |
| gate | 1 | 23.91 | 0 | 0.024 | 0.103 |

All four pass. The Sky Beta LAND state is pad hover, not a physical landing. Window/gate planned lengths differ, so the old identical-route suspicion is resolved. Rendered textures, voxel geometry and synthetic odometry do not qualify a real camera, SLAM, Nano throughput or flight.

## Additional negative evidence

The first revised integration run classified a zero scan stamp as stale rather than unstamped; reordered validation restored the intended rejection counters without accepting the scan. The new final mission innovation gate also exposed a test fixture that used identity orientation for a course pose facing north; the valid fixture now carries the correct yaw, while excessive-yaw and 100 m corrections remain rejected. Failed renderer traces and the intermediate integration log are retained in the output validation evidence.

## Organizer-library version check on the laptop

In an isolated Windows Python 3.12 environment: numpy **1.26.4**, opencv-python-headless **4.10.0.84** (cv2 4.10.0), pytest 8.3.5. Results: **47/47 unit tests**, **12/12 node integration**, **12/12 safety regressions**, **12 stereo scenarios** and **14 line scenarios** pass. Full-course matrices above used numpy 2.1.3/OpenCV 5.0; they were not repeated under the older pair. This validates library compatibility on x86 Windows only, not ARM/Jetson or ROS 2 Humble. No global packages changed.

Local ROS availability check: `wsl --list --verbose` reports no installed distributions; `ros2`, `colcon` and Docker CLI are unavailable on PATH. Actual DDS/launch tests remain unexecuted. Installing a Linux/ROS operating environment was not part of this checkpoint.
