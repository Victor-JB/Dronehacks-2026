# Integrating nav_policy (review before any use; NOT flight-approved)

`dh_nav_policy/` is a GENERATED, self-contained ROS 2 package
(`python tools/integration/build_nav_policy_pkg.py integration/dh_nav_policy`). It has the same shape as
the package that flew on 2026-11-07 (`test_files`, branch `backup/testFiles`):
- **Location:** its own package next to `dronehacks` in `~/team_ws/src`.
- **Launch:** its own launch file, starting exactly one `flight_interface` (`auto_offboard_on_arm: True`,
  `anchor_setpoints: True`, the same as `test_files`) and the `nav_policy` node.
- **Selection:** chosen with `COMPETITOR_PKG=dh_nav_policy` and `COMPETITOR_LAUNCH=nav_policy.launch.py`.
- **Dependencies:** it changes nothing in `main`, and imports `main`'s `refnav.depth` and
  `dronehacks_nav.mapping` (`exec_depend dronehacks_nav`).

## Flight behaviour copied from test_files
| | test_files (flown) | nav_policy |
|---|---|---|
| Before Offboard | pose at current position, no yaw, 10 Hz | same (`send_handover_pose`), **only while preflight passes**; otherwise nothing, so Offboard cannot start |
| Offboard starts | pose = start + 1 m up, no yaw | pose = start xy, `cruise_z` (0.75 m), no yaw; 15 s timeout → hold |
| Moving | pose waypoints | world-frame twist, **with yaw rate (not yet proven on hardware)** |
| Offboard ends | stop, keep publishing current-position pose | same; never navigates again until disarm |
| Hold | fixed pose | fixed pose captured once |
| Descent | twist descent | **disabled** (hovers; pilot lands) |

**Additions that test_files did not have:**
- The Stage 4 gate acts on every navigation twist: stopping distance, a 0.28 m body floor, unknown space
  counts as blocked, stale data or lost localization means 0 m/s, plus the flight band and the geofence.
- **Pre-Offboard setpoint only when healthy.** The pose is sent only while preflight passes (geofence
  set, fresh odometry, VO tracking, fresh depth). Otherwise nothing is published, so `flight_interface`
  stays out of Offboard and the drone remains in Position under the pilot (`autonomy_pipeline.md`).
- **Motion check.** `main`'s `dronehacks_nav.motion_check` runs unchanged on every outgoing command, with
  the same parameters as `drone.yaml`. A failure freezes the drone at a fixed point and hands it to the pilot.
- **Timing.** Odometry staleness is 1.0 s, as in `main`'s `drone.yaml`. Depth frames wait for their
  odometry before mapping, as in `main`'s `map_node`; the forward check uses the latest attitude, as in
  `main`'s `skybeta_node`.
- On localization loss it sends a zero twist for 1.5 s, then stops commanding; `flight_interface` holds and
  the pilot takes over.

## Required configuration (config/nav_policy.yaml)
- `geofence: [xmin, xmax, ymin, ymax]` in the VIO frame (origin = where the stack started, +x = nose at
  start). **Unset means no horizontal motion.** Measure the test area; keep the body centre 0.4 m inside it
  (netting may not show in depth).
- `dry_run` defaults to true in the launch file. Live needs `dry_run:=false`.
- `depth_topic /oak/stereo/image_raw`, `depth_info_topic /oak/stereo/camera_info`: confirm they are published.
  The depth images were not in the 2026-11-07 bag (`fly.sh start --depth` records them).
- `brake_decel` and `latency_s` are placeholders (1.0 m/s², 0.8 s) until measured.
- `rgb_topic: ""` means the blue-square detection is off. `../skybeta_overrides/oakd_vslam.rgb_candidate.yaml`
  is an unverified candidate, disabled.

## Envelope and passage widths (what the policy will and will not fly)
- **Body:** physical half-span 0.24 m (simulation contact radius).
- **Planner inflation:** 0.33 m = 0.24 m + 0.04 m margin + 2 × 0.025 m pose σ.
- **Openings:**
  - Minimum corridor or opening width: **0.66 m** (anything narrower is refused).
  - Vertical envelope: cruise 0.75 m ± 0.22 m, so an opening must be free from 0.53 m to 0.97 m.
  - Round tunnels are checked slice by slice, with a 0.33 × 0.22 m half-envelope inside the circle.
- **Final gate:** stops anything that brings the body centre within 0.28 m of an obstacle or unknown space,
  plus 0.10 m along the travel direction.
- **Course used in testing:** the simulated end-to-end success used a **modified** course. A middle-room pillar
  that left 0.6 m gaps was replaced by a post with 0.8 m gaps, because 0.6 m is below the 0.66 m minimum.

## Checks run here (no ROS on this machine)
- `tools/integration/test_integration_pkg.py` runs against `origin/main`'s `refnav` and `dronehacks_nav`.
  It checks that:
  - the package is in sync with its sources;
  - its pure tests pass;
  - its node builds on fake ROS, with only `/skybeta/setpoint/{pose,twist}` publishers live;
  - there is no `px4_msgs` and no PX4 backend;
  - its launch equals `test_files`' `flight_interface` parameters, with exactly one `flight_interface`.
- **Not done:** `colcon build`, a `ros2 launch` dry parse, and any hardware.

First physical test procedure: `integration/FIRST_TEST.md`.

## Installing it later (team decision; not done here)
On the drone (team approval, props off):
1. Copy `integration/dh_nav_policy` to `~/team_ws/src/dh_nav_policy`.
2. Run `team_build.sh --packages-select dh_nav_policy`.
3. Edit `drone.env` (`COMPETITOR_PKG=dh_nav_policy`, `COMPETITOR_LAUNCH=nav_policy.launch.py`).
4. Restart the stack on the ground.

Every package in `~/team_ws/src` must build, or nothing launches. Switching back means restoring the
`drone.env` lines (cheatsheet §6).

`fly-sim.sh` launches only `dronehacks_nav`'s `mission.launch.py`, so it cannot fly this package as is.
Using the `policy/<name>` workflow would need either the sim to accept another launch or the policy moved into
`dronehacks_nav`. Neither is done here.
