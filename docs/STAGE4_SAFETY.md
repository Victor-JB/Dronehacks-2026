# Stage 4 safety filter (offline, not flight-qualified)

Code: `ros2_ws/src/dh_core/dh_core/safety_filter.py`. Tests: `ros2_ws/src/dh_core/test/test_safety_filter.py`.
Pure Python + numpy. It publishes nothing and is not wired into any node or launch file.

## Model
`d_stop(v) = v*latency + v^2/(2*brake_decel)`; allowed speed is the largest `v` with
`d_stop(v) <= free_distance - vehicle_radius - uncertainty_margin - k*pos_std`.
Verdicts: SAFE / LIMIT / STOP / UNKNOWN, each with `max_speed` and a reason code.
UNKNOWN (missing, stale, invalid depth/odometry/transform, low depth coverage, bad config) and STOP
(localization HOVER/ABORT, or no stopping room) both give 0 m/s. `+inf` depth is capped at `reliable_range`.

## Finding: `speed_scale` alone does not enforce a limit
`TrajectoryFollower.step` applies `speed_scale` only to the reference clock and feed-forward term. The
`kp*e` position-error term is added afterwards, limited only by `v_cap`. With `speed_scale=0` the
follower still commands motion toward a reference ahead of the drone; with 0.5 it can command full speed
(pinned by `test_raw_speed_scale_*`). `SafetyGate.clamp` hard-limits the command actually sent and zeroes it
on STOP/UNKNOWN. `tracker.py` is unchanged.

## Placeholders that MUST be measured on the real drone before any flight use
- `latency_s = 0.40` (pose latency + PX4 velocity-loop lag + loop period) - pessimistic guess.
- `brake_decel = 1.0 m/s^2` - pessimistic guess; the sim's 3 m/s^2 cap is not evidence about the airframe.
- `vehicle_radius`, `uncertainty_margin`, `reliable_range`, timeouts.

## Not covered yet
Computing `free_distance` and `depth_valid_fraction` from a depth image (swept-corridor extraction), lateral
and vertical clearance (the filter judges one distance along the travel direction), integration into
`mission_node`, and hardware timing. Closed-loop tests use a point-mass plant, not PX4.
