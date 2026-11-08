# Milestone 2: passive telemetry + shadow-mode safety (offline; not flight-qualified)

Code: `ros2_ws/src/dh_core/dh_core/shadow/` (pure Python) and `ros2_ws/src/dh_autonomy/dh_autonomy/shadow_node.py`
(thin read-only ROS wrapper, deliberately **not** a console script). Tests: `dh_core/test/test_shadow.py`,
`tools/sil/test_shadow_node.py`. Real-data fixture: `dh_core/test/data/odom_trace_20261107_vio_divergence.csv`,
cut from main's `drone-logs/20261107-155647/20261107-155647_0.mcap` by `tools/replay/extract_odom_trace.py`.

## Read-only guarantee
- `shadow_node` creates subscriptions and a timer only. At start-up it lists its own publishers, service
  clients and services and **refuses to run** if any is not rclpy's implicit infrastructure
  (`/rosout`, `/parameter_events`, parameter services). `guard.check_endpoints` holds the allowlist.
- `guard.scan_sources` (AST) fails the tests if any shadow file creates a publisher, a client or an action
  client, imports `px4_msgs`, or contains a `/skybeta/setpoint`, `/fmu/in` or vehicle-command string.
  A positive control proves it catches the existing `backends.py`.
- `SafetyGate` runs with an inert follower: its clamp is applied to the *measured* velocity, only to report.
- The node is independent of `mission_node`, `flight_interface` and main's `skybeta_node`.

## Topics: confirmed vs. assumed
| Topic | Type | Evidence | Status |
|---|---|---|---|
| `/visual_slam/tracking/odometry` | nav_msgs/Odometry | organizer contract + 2026-11-07 bag: 29.3 Hz, frame `odom` → child `base_link` | **confirmed** |
| `/skybeta/state` | std_msgs/String JSON | contract + bag: 5 Hz; keys listed in `observation.py` | **confirmed** |
| `/visual_slam/status` | isaac_ros_visual_slam_interfaces/VisualSlamStatus | bag: 29.3 Hz, vo_state ∈ {1, 2} | **confirmed**; package exists only on the drone |
| `/oak/stereo/camera_info` | sensor_msgs/CameraInfo | bag: 30 Hz, 320×200, frame `oak_rgb_camera_optical_frame` | **confirmed** |
| `/oak/stereo/image_raw` | sensor_msgs/Image 16UC1 | main's `config/drone.yaml` and `fly-sim.sh`'s bag list | **assumed**: absent from the bag, so its rate, latency, encoding and frame are unverified |
| TF `base_link` → camera | tf2 | bag `/tf_static`: base_link→oak_parent_frame (0.1053, 0, 0.0304) → … → optical | **confirmed** for that bag |
| `/skybeta/setpoint/*`, `/fmu/in/*` | — | — | never touched |

Every topic name is a ROS parameter. The organizer's `autonomy_pipeline.md` lives only on the drone
(`~/skybeta/docs/`) and was not read: the interface facts above come from the repository and the bag.

## Measured on real data (2026-11-07 bag, laptop analysis)
- Clock domains: odometry `header.stamp` and receipt are both Unix epoch → latency is meaningful.
  Capture→receipt: p50 0.08 s, p95 0.32 s, p99 0.36 s, max 0.41 s (whole 188 s healthy segment).
  `ClockDomainMonitor` decides comparability from the data; on a mismatch, a zero-based or sim-time stamp,
  a stamp ahead of receipt, or a clock step, no latency is reported and the stream counts as unusable.
- Twist is in the child (body) frame: rotating it to ENU halves the error against finite differences
  (0.20 vs 0.40 m/s RMS while moving). `twist_in_child_frame` stays a parameter.
- Odometry gaps: none over 0.2 s by stamp in the healthy segment; about 12 dropped frames in 35 s.
- A real cuVSLAM divergence at t≈188 s (positions jump to tens or thousands of metres, covariance up to 1.2e4).
  On replay, the monitor goes OK → HOVER (pose discontinuity, STOP) → ABORT and never permits motion.
- Defaults raised from Milestone 1 on this evidence: `odom_timeout_s` 0.40→0.60 (the stamp age at a 10 Hz
  poll reached 0.43 s while tracking was healthy); `latency_s` 0.40→0.80.
- Depth: **nothing measured**. The bag holds `camera_info` at 30 Hz and no image, which is exactly why
  `camera_info` is never used as the depth rate.

## Per-observation output (`ShadowMonitor.evaluate`)
`verdict` (SAFE/LIMIT/STOP/UNKNOWN), `max_speed`, `reason`, `free_distance` (corridor along the travel
direction, minus the distance travelled since capture), `free_distance_at_capture`, `valid_fraction` (the worst
quadrant of the corridor cross-sections), `approach_speed`, `stop_distance`, `current_motion_exceeds_limit`,
`loc_mode`/`loc_reason` (reused `health.LocalizationHealth`), `depth_status`, `rejects` (counted per reason),
and per stream: rate by receipt and by stamp, clock state, latency p50/p95/max, gaps, estimated drops,
out-of-order, duplicates, unstamped and ages.

## LIMITATION: not full collision protection
The forward-clearance model judges **one swept corridor** (±0.45 m across, ±0.30 m vertically, along the
direction of travel) inside the camera's field of view. It does **not** protect against obstacles beside
the corridor, above or below the band, behind the drone or outside the FOV, or thin objects that give fewer
than 3 returns. Reversing or sliding sideways out of view therefore reads UNKNOWN, which is correct but
means no clearance at all. Stage 5's 3-D map and trajectory checks are needed for lateral and vertical
protection.

## Conservative choices
No return is unknown, never free. `+inf` is capped at `reliable_range` (3 m) before motion compensation.
A rejected newest frame clears the cached clearance; an older frame never replaces a newer one. A depth
frame needs an odometry sample within 0.10 s of its capture stamp. A blind quadrant gives UNKNOWN even when
the floor fills the rest (a textureless wall), and a test pins this.
Open space with nothing inside 8 m gives no returns and reads UNKNOWN. Indoors this needs real-depth checking.

## Performance (laptop, `tools/replay/bench_shadow.py`; Orin NOT measured)
320×200 depth frame at stride 4: p50 5.0 ms, p95 7.1 ms, max 13.3 ms. Odometry message: 0.37 ms.
`evaluate()`: p50 1.8 ms.

## Running it (later, with approval; not part of this milestone)
`python3 -m dh_autonomy.shadow_node --ros-args -p log_path:=/tmp/shadow.jsonl` from a sourced workspace that
contains `dh_core` and `refnav`. Observation only, but it is still new code on the drone: it needs team
approval first, and must not go into `~/team_ws/src` until it is reviewed.
