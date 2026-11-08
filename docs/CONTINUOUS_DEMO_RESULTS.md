# Continuous forward-camera results — 2026-11-08

**Partial navigation demonstrated. Full-course acceptance failed; physical flight remains NO-GO and untested.**

Published policy source: `8e2a594d478f56ad071cbfdf4acb99ac49563c0a`. Final reproduction 20 matches all five committed policy/simulator/render source hashes. Its first 1,800 frames match attempt 19 exactly in position and command; the later metadata aging and stricter initial-clearance check did not alter that trajectory.

Strongest run: **15.714 m**, **19 replans**, **184.25 s**, **0 contacts**, **0.153 m minimum sampled hull clearance**. Full inflated-envelope tube crossing verified by the simulator grader. The live policy did not semantically identify the tube; it followed sensor-observed safe geometry. No window crossing, blue-pad discovery, landing or completed maze mission is established.

| Configuration | Distance m | Time s | Replans | Hull clearance m | Contacts | Outcome |
|---|---:|---:|---:|---:|---:|---|
| Original walls | 3.686 | 82.85 | 4 | 0.160 | 0 | NAVIGATION_DEADLOCK |
| North-facing start | 3.313 | 76.50 | 9 | 0.154 | 0 | NAVIGATION_DEADLOCK |
| Altered walls, final source | 15.714 | 184.25 | 19 | 0.153 | 0 | NAVIGATION_DEADLOCK |

Final current classified coverage is 14.73% of the fixed 8 m × 8 m planning band; cumulative sampled lower-bound coverage is 46.70%. This is not maze-completion percentage. There are 780 currently confident occupied columns; individual wall identities are not classified. Old-cell re-entry distance lower bound is 0.000 m; a comprehensive unnecessary-backtracking metric remains unimplemented. Clearance is sampled at 10 Hz; contact detection runs at 200 Hz.

Wall execution time 2,032.99 s and Python CPU time 1,863.44 s include physics, rendering, mapping and encoding (real-time factor 0.091). They are not a standalone planner or Jetson benchmark.

## What changed

- Existing team VoxelMap retained. Added optional bounded free-evidence retention, confidence aging to blocked unknown, per-voxel timestamps and conservative whole-voxel depth-image carving. Live depth300ms, odometry400ms, TF and map-update500ms gates remain. No stale observation becomes free by aging.
- Explicit expiring launch-clearance certificate is separate from sensor evidence, never overrides positive hits, and certifies no route. Simulated profile uses a verified +50mm east,+150mm north placement. Original marker-band certificate is rejected. Physical defaults remain unverified/passive/dry.
- Guarded stationary yaw scan and route-heading observation, observed-side targets, held-reference recovery, stopping-sweep speed scaling and redundant A* starting-cell stub removal. Existing A*/clothoid, follower and independent evaluator retained. Circular swept clearance preserves the complete envelope.
- Existing registered-color HSV/depth pad detector and bounded tracker connected optionally; registration defaults false. A confirmed pad can bias observed-free local targets. No landing command was created. Actual pad discovery was not achieved in these runs.

## Remaining blockers

1. Persistent current-envelope observability after pauses and bends: original run nearest unknown column.418m versus required inflated radius.433m; north run has unknown at its current cell. No blind rotation/translation is allowed to escape this state.
2. Window approach is not robust. CAD-only offline grading measures a.9144m square opening. Sensor maps briefly certified its centre (.443m clearance versus.433m required), then dropped to.427m and disconnected; another.437m centre sample was not connected to the drone. These grader coordinates never enter autonomy. Robust observation/target selection through the actual opening remains unresolved; do not waive unknown cells or shrink the vehicle.
3. Full-course robustness failed: two starts deadlock early; altered geometry reaches the tube and bottom/inner corridors but also deadlocks. Semantic passage recognition, the required window, blue goal and landing are unfinished at policy level.
4. Actual ROS2 Humble/DDS and Sky Beta/PX4 closed-loop execution were not available. MuJoCo force/torque servo plus fake ROS and bounded noisy physics-derived odometry is not reconstructed VO, stereo disparity, PX4 dynamics or hardware qualification. Actual stereo-SGBM tests are separate fixtures.
5. Physical dimensions/clearance, legal altitude and floor reference, camera/depth/RGB registration/TF, stopping/latency bounds, safe launch condition and real execution budget need verification. Initial main 11280a8 documents a depth/VIO latency issue; reconcile it with the actual tested configuration. No autonomous landing authority is verified.

## Tests and preserved evidence

- 94 core pytest cases; 10 new focused evidence/startup/geometry checks; 11 existing fake-ROS checks; corrected obstacle scenario; 16 safety regressions; 12 rendered stereo cases all pass. Stereo cases include intentional negative calibration controls. Obstacle regression retains its original assertions and reports 0.764 m / 3 replans / 0 contacts on an ideal-panoramic fixture, not the course camera.
- Actual ROS/DDS, Gazebo, PX4 and Jetson: NOT TESTED. Frozen Phase2 f85c492 ZIP/TAR hashes reverified unchanged, controls and existing launch/profiles unchanged. No SSH, deployment, physical publisher, main/drone push, promotion or teammate worktree changes.
- 20 bounded attempts preserved. Attempt 02 is a collision after an unstable simulator attitude controller mixed body-rate/world-torque frames; it is not navigation success. Bootstrap 01–03 have logs/traces but no movie. 04 onward preserve original recordings. Early 04–08 displayed launch certificate in green; 09 onward distinguish it in purple. Source fingerprints exist from 07 onward; only final 20 matches final committed sources byte-for-byte.
- Local media: main 120 s at 1920×1080, highlight 25 s including the complete tube crossing, full uncut 184.3 s, separate tracking/first-person crops, 15 verified 1920×1080 video-frame PNGs and 1800×1600 sensor-map/actual-physics-track figure. Full sequential decoding passed for all five MP4s. No still-image replay or scripted path is presented as policy execution. Native simulation-time rate is preserved.

## Judge paragraph

Using only simulated forward-depth observations and odometry, our existing A* and clothoid pipeline drove a rigid-body drone model 15.7 m through corridors and a tube without contact. An independent filter checks sensor freshness, localization, the full collision envelope and stopping space. The run ends in a conservative hold before course completion; this is a local MuJoCo integration demonstration, not ROS/PX4 or flight qualification.

## All attempted runs

| Attempt | Outcome | Time s | Distance m | Replans | Hull clearance m | Contacts | Known band % |
|---|---|---:|---:|---:|---:|---:|---:|
| 01_startup | NAVIGATION_DEADLOCK | 34.95 | 0.002 | 0 | 0.229 | 0 | N/A |
| 02_sensor_visibility | COLLISION | 16.05 | 1.852 | 0 | 0.018 | 1 | N/A |
| 03_attitude_frames | NAVIGATION_DEADLOCK | 59.95 | 0.013 | 0 | 0.228 | 0 | N/A |
| 04_dense_evidence | NAVIGATION_DEADLOCK | 69.95 | 0.013 | 0 | 0.228 | 0 | 0.75 |
| 05_bounded_vertical_error | NAVIGATION_DEADLOCK | 119.95 | 1.598 | 4 | 0.190 | 0 | 5.24 |
| 06_heading_alignment | NAVIGATION_DEADLOCK | 149.95 | 1.503 | 9 | 0.175 | 0 | 2.58 |
| 07_stop_recovery | NAVIGATION_DEADLOCK | 159.95 | 1.503 | 7 | 0.176 | 0 | 2.56 |
| 08_start_north | NAVIGATION_DEADLOCK | 129.95 | 0.015 | 1 | 0.255 | 0 | 0.97 |
| 09_altered_walls | NAVIGATION_DEADLOCK | 149.95 | 0.038 | 3 | 0.255 | 0 | 3.22 |
| 10_capsule_clearance | NAVIGATION_DEADLOCK | 53.00 | 1.479 | 9 | 0.182 | 0 | 11.24 |
| 11_capsule_north | NAVIGATION_DEADLOCK | 45.05 | 0.015 | 1 | 0.255 | 0 | 7.70 |
| 12_observed_turn_targets | NAVIGATION_DEADLOCK | 45.05 | 0.016 | 2 | 0.255 | 0 | 8.35 |
| 13_final_baseline | NAVIGATION_DEADLOCK | 45.05 | 0.016 | 2 | 0.255 | 0 | 8.24 |
| 14_final_north | NAVIGATION_DEADLOCK | 45.05 | 0.017 | 2 | 0.255 | 0 | 7.44 |
| 15_geometric_heading | NAVIGATION_DEADLOCK | 45.05 | 0.016 | 2 | 0.255 | 0 | 8.24 |
| 16_original_start_band | INITIAL_PLACEMENT_FAILURE | 0.00 | 0.000 | 0 | N/A | 0 | N/A |
| 17_start_cell_correction | NAVIGATION_DEADLOCK | 82.85 | 3.686 | 4 | 0.160 | 0 | 8.26 |
| 18_corrected_north | NAVIGATION_DEADLOCK | 76.50 | 3.313 | 9 | 0.154 | 0 | 17.68 |
| 19_corrected_altered | NAVIGATION_DEADLOCK | 179.95 | 15.714 | 19 | 0.153 | 0 | 20.26 |
| 20_final_reproduction | NAVIGATION_DEADLOCK | 184.25 | 15.714 | 19 | 0.153 | 0 | 14.73 |


Evidence audit: all 20 result files, 1,573 sensor snapshots and the first/last frames of all 16 recorded development movies are readable. All five selected delivery videos were decoded completely, and all 15 delivery PNGs were verified. Movie/PNG SHA256 receipts and exact source-byte checks are saved with the local artifacts.

Final team-context fetch: `origin/main` advanced to `06dcacf`, adding a separate recorded-flight/walkthrough waypoint mission (`box_route_v2`) and an over-wall leg. Mapper/depth interface files are unchanged. It was inspected read-only and neither merged nor executed. This branch retains the requested CAD-free and no-over-wall policy; its NO-GO assessment does not assess the separate team mission.
