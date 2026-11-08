# nav_policy: CAD-free navigation policy (integration review; NOT flight-approved)

**Run:** `ros2 launch dh_autonomy nav_policy.launch.py` (dry run by default). Executable `nav_policy`
(`dh_autonomy/nav_policy_node.py`), config `dh_autonomy/config/nav_policy.yaml`. The launch file starts one
`flight_interface` (`auto_offboard_on_arm: true`) and this node. Use it *instead of* the team's current
launch file, never alongside it.

**Pipeline:** sensors → `ShadowMonitor` (validation, freshness, clocks, TF) → `Explorer` (online map over main's
`VoxelMap`, frontiers + dead-end memory + goal persistence, `ClearancePlanner` A*, clothoid `fillet_path`,
`time_parameterize`, `TrajectoryFollower`, opening verification) → `CommandSafety` (Stage 4 `SafetyGate` on the
final velocity) → `SkybetaBackend` (the only `/skybeta/setpoint/*` publisher) or `DryRunBackend`.

**Topics:** reads `/visual_slam/tracking/odometry`, `/skybeta/state`, `/visual_slam/status`,
`/oak/stereo/image_raw`, `/oak/stereo/camera_info`, and TF `base_link`←camera. Writes `/skybeta/setpoint/twist`
(world-frame ENU, `angular.z` = yaw rate, 10 Hz) and `/skybeta/setpoint/pose` (takeoff and hold only). Never
writes `/fmu/in/*`.

**Organizer rules followed:** before Offboard, a zero twist every tick; one go per arming (a takeover latches
YIELDED until disarm); pose only for takeoff and hold; speed ≤ `v_max` (0.35 m/s) < 1.5 m/s; height fixed at
`cruise_z` inside the 0.3–3.0 m clamp.

**Safety (fails closed):** no geofence set means no horizontal motion. Stale depth or odometry, or unknown
localization, means zero velocity. Localization loss means a zero twist for 1.5 s, then no commands
(flight_interface holds, the pilot takes over); position hold is not trusted. Unknown space is blocked; the free
distance comes from a body-inflated march through the map in the commanded direction (lateral and reverse
included). A vertical guard keeps the flight band. No landing or descent.

**Clearance (layered):** the planner inflates obstacles by 0.33 m (physical half-span 0.24 m + 0.04 m margin
+ 2 × 0.025 m pose σ), so it needs gaps of at least 0.66 m. The final-command gate stops anything that would
bring the body centre within 0.28 m (body + 4 cm) and keeps a further 0.10 m along the direction of travel;
it slows to 0.15 m/s where clearance is tight. Unknown space counts as an obstacle in both.

**Exploration:** frontiers are FREE cells 8-adjacent to UNKNOWN. The old distance-based test turned every
free cell along every wall into a frontier, because wall back sides are never seen. Frontier cells are
grouped into 0.8 m tiles. Each goal is kept until it is reached, it disappears, or another goal scores at
least 1 m better. A reached tile stays retired unless its frontier doubles; an unreachable tile is retried
after 20 s. A previously verified opening is reused unless the map shows something inside its envelope.
The start-up 360° scan now counts measured rotation; it never actually turned before.

**Blue square / RGB:**
- **Policy side:** the RGB camera is handled as its own camera (`rgb_topic`, its own `camera_info` and TF);
  depth is not aligned to it, so the depth path is untouched.
- **Candidate OAK config:** `skybeta_overrides/oakd_vslam.rgb_candidate.yaml` (not read by the Dockerfile,
  so disabled): `i_pipeline_type: rgbd` plus a 320×200 @ 15 Hz `rgb:` block (about 2.9 MB/s).
- **Blockers, unverified:** that driver v3's `rgbd` still publishes the left/right raw images cuVSLAM
  needs; the v3 key names; and the USB budget, measured by the team's procedure (about 114 of about
  143 MB/s are already used).

**Descent:** disabled. Over the target the policy hovers and reports.

**Before any live test:** measure braking and latency (`brake_decel` and `latency_s` are placeholders); set the
geofence; confirm `/oak/stereo/image_raw` is actually published (it was absent from the 2026-11-07 bag);
validate the RGB candidate on the bench; get team approval; props off for the first run.
