# First controlled test of dh_nav_policy (team approval required at every step)

This uses the same stack, package switch and operator flow as the flown `test_files` mission (see
`DRONE_CHEATSHEET.txt` on `backup/testFiles`). There is a safety pilot on the RC at all times, the kill switch
is checked, and the pilot can take over at any moment by raising the throttle to centre or moving a stick.
Full-maze exploration stays OFF until stage 5 has passed. Props-off steps prove wiring and logic only, not
yaw response or braking.

## 0. Build (container, drone on the ground, disarmed). Not run anywhere yet
```bash
cp -r <repo>/integration/dh_nav_policy ~/team_ws/src/dh_nav_policy     # sibling of dronehacks, like testFiles
team_build.sh --packages-select dh_nav_policy                            # must succeed: a failed package blocks the stack
ros2 launch dh_nav_policy nav_policy.launch.py --show-args               # parses; dry_run defaults to true
python3 -m pytest -q ~/team_ws/src/dh_nav_policy/test                     # DRONEHACKS_MAIN=~/team_ws/src/dronehacks
```
Edit `~/team_ws/src/dh_nav_policy/config/nav_policy.yaml`: `geofence` = the measured test area in the VIO frame
(origin = take-off spot, +x = nose at start), at least 0.4 m inside any net. Keep `v_max: 0.35`.

## 1. Props OFF, dry run (nothing reaches PX4)
- Start the node by hand (cheatsheet "working by hand"):
  `ros2 run dh_nav_policy nav_policy --ros-args --params-file .../nav_policy.yaml -p dry_run:=true`.
- **Check:**
  - `ros2 topic hz /visual_slam/tracking/odometry` reads about 30 Hz.
  - `ros2 topic hz /oak/stereo/image_raw` reads about 30 Hz. It must be published; it was never in a bag.
  - `ros2 topic echo /skybeta/state --once` shows `px4_link` true and `local_enu` not null.
  - TF `base_link` ← `oak_rgb_camera_optical_frame` exists.
- **The log must say** `DRY RUN` and the geofence, and must not show "NOT READY". If it does, the reasons
  are printed.
- `ros2 topic pub --once /dh/start std_msgs/msg/Empty` grants simulated authority. Carry the drone by hand
  toward a wall and watch `/dh/dry_run/cmd_vel`: speeds are ≤ 0.35 m/s and drop to 0 near the wall and when
  the depth cable is unplugged.

## 2. Props OFF, live package switch (organizer interface; motors cannot spin without props doing work)
- Set `drone.env`: `COMPETITOR_PKG=dh_nav_policy`, `COMPETITOR_LAUNCH=nav_policy.launch.py`. Add
  `dry_run:=false` only for this test, by editing the launch default or adding a second launch file.
  Restart the stack on the ground.
- **Before arming:** `ros2 topic hz /skybeta/setpoint/pose` reads about 10 Hz only when preflight passes.
  With the geofence unset, or depth stopped, there is NO setpoint, so Offboard cannot start. Verify this.
- Arm on Altitude, switch to Position. `ros2 topic echo /skybeta/state` shows `offboard_active` true, and the
  takeoff pose is the start x/y at 0.75 m.
- Pilot takeover by moving a stick: the log says YIELDED, no twist follows, and it does not resume until a
  disarm.

## 3. Props ON, hover only (geofence set, open area, no obstacles within 2 m)
- Arm, then Position: the drone takes off to 0.75 m and starts the 360° scan.
- **Pilot takes over at any sign of drift.** Check that the motion check stays quiet, VIO and PX4 heights
  agree in `/skybeta/state`, and there is no clamp warning.

## 4. Props ON, supervised yaw-rate test (not yet proven in flight)
- During the scan, compare the commanded `angular.z` with the measured yaw rate from odometry orientation.
  Sign: positive = counter-clockwise from above (ENU). Rate: ≤ 0.7 rad/s.
- **Abort** if the drone turns the wrong way or translates while turning.

## 5. Props ON, single wall, low speed
- Put one tall wall about 2.5 m ahead inside the geofence, with the rest of the area open, and log a bag
  (`fly.sh start --depth`).
- **Pass:**
  - it slows near the wall and keeps at least 0.3 m from it;
  - it never climbs;
  - unplugging depth or covering the camera makes it stop;
  - a pilot takeover works.
- Measure the real stopping distance from the bag, then set `brake_decel` and `latency_s` from data.

Only after 1–5 pass, with the measured braking values in the config, consider multi-wall or maze exploration.
The blue-square target stays off (no colour stream) until `skybeta_overrides/oakd_vslam.rgb_candidate.yaml` is
validated on the bench.
