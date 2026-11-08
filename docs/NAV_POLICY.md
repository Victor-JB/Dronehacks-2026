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

**Status:**
- Tested offline: the node's handoff, endpoint and failure behaviour (fake ROS); map, opening, safety,
  detector and replanning units.
- Closed-loop (simplified point-mass sim, rendered depth): explores, passes through a verified window with no
  contact.
- Not demonstrated: tunnel traversal and reaching the pad end-to-end.
- **Blue square:** detector and tracker work on rendered images, but the drone's OAK config publishes **no
  colour stream**, so it is off (`rgb_topic: ""`).

**Before any live test:** measure braking and latency (`brake_decel`, `latency_s` are placeholders); set the
geofence; confirm `/oak/stereo/image_raw` is actually published (it was absent from the 2026-11-07 bag); get team
approval; props off for the first run.
