# Stationary Jetson verification — manual procedure, not executed

This task stops **before the first SSH connection or hardware operation**. All commands below are a guide for a later, explicitly authorized session. Physical flight remains prohibited. Keep propellers removed and the flight battery disconnected. Power the Jetson Orin Nano and sensors independently using the organizers' approved supply. The AGX Orin is for Claude's simulation work; do not connect to it for this procedure.

## 1. Preconditions and first connection

Have the organizers confirm the sensor-only power arrangement and how to prevent the automatically started stack from running a PX4 bridge, MicroXRCEAgent, flight interface or another competitor node. A default boot may start all four. Do not run `host/run.sh stack`, change autostart configuration, open UART devices, alter PX4 parameters, arm, or publish flight-control messages. Ask the organizers to establish the sensor-only environment; do not assume that removing the battery isolates every USB/UART path.

After the user explicitly authorizes hardware access, confirm the **Nano's** host name and login from the team. The first connection would be:

```bash
ssh <confirmed-Nano-user>@<confirmed-Nano-address>
```

Do not guess an address or use the AGX's address. No connection has been attempted by Codex.

## 2. Inspect the sensor-only environment

On the host, record versions and the active process/container list:

```bash
cat /etc/nv_tegra_release
cat /proc/device-tree/model
lsb_release -a
docker ps
ps -eo pid,args
```

Record the installed organizer image tag and source revision. Reference inspected locally: `Artash-N/skybeta`, fleet-ready commit `7efe7e45080cdf9fa05814824ac8bfe7eca3ef9d`. The physical image may differ.

Inside an existing, organizer-approved sensor-only container:

```bash
echo "$ROS_DISTRO"
python3 -c 'import numpy, cv2; print(numpy.__version__, cv2.__version__)'
ros2 pkg prefix isaac_ros_visual_slam_interfaces
ros2 pkg prefix message_filters
ros2 pkg prefix tf2_ros
ros2 node list
ros2 topic list
```

Expected: ROS 2 Humble and the pinned ARM/Isaac stack, numpy <2, sensor nodes and TF only. If a required dependency is absent, stop and report. Do not install packages into the running event container or copy x86 binaries onto ARM. Do not add `px4_msgs` to the team workspace.

## 3. Transfer and build, without replacing team code

From the delivered Git bundle, restore `saketh/autonomy` on the laptop. Confirm the commit and clean working tree. Check for existing `dh_core` and `dh_autonomy` packages in `~/team_ws/src` before copying; preserve any existing versions and compare differences rather than overwriting them. Transfer only:

- `ros2_ws/src/dh_core` -> `~/team_ws/src/dh_core`
- `ros2_ws/src/dh_autonomy` -> `~/team_ws/src/dh_autonomy`
- `tools/hil` -> `~/team_ws/tools/hil`
- the official OBJ -> `~/team_ws/course/dronehacks-course-v3.obj`

Keep a SHA-256 manifest of transferred files and the Git commit in the session report. The course used locally had the hash recorded in `docs/LOCAL_VALIDATION.md`; compare the event's file rather than assuming identical geometry.

Inside the container:

```bash
cd /root/team_ws
colcon build --symlink-install --packages-select dh_core dh_autonomy
source install/setup.bash
ros2 pkg executables dh_autonomy
```

Expected executables: `mission`, `localizer`, `stereo_scan`, `line`, `sitl_odom`. Building does not prove runtime compatibility.

## 4. Actual ROS integration tests, isolated from devices

Use a separate shell/container with no camera/FC transport and an unused local DDS domain. These tests publish **synthetic** images/odometry to test DDS, QoS, TF, the shipped launch, OpenCV and dry-run isolation. They do not qualify the real camera or cuVSLAM.

```bash
export ROS_DOMAIN_ID=77
export ROS_LOCALHOST_ONLY=1
export DH_COURSE_OBJ=/root/team_ws/course/dronehacks-course-v3.obj
cd /root/team_ws
source install/setup.bash
colcon test --packages-select dh_core dh_autonomy
colcon test-result --verbose
```

Expected: core tests and both ROS launch tests pass. The image test runs the actual autonomy launch with synthetic camera data and explicitly disables the VO-status requirement only for that fixture. Keep `require_vo_status:=true` for sensor verification. Save all failing logs; do not loosen timing, frames or clearance checks to obtain a pass. These tests have **not** been executed locally.

## 5. Real sensors, stationary, no mission start

Use the organizers' camera + cuVSLAM launch only, with the measured six-axis camera mount from their configuration. Do not start the bridge, agent or flight interface. Keep the drone stationary during estimator startup. In the sensor container's DDS domain:

```bash
ros2 topic info -v /oak/left/image_raw
ros2 topic info -v /oak/right/image_raw
ros2 topic info -v /visual_slam/tracking/odometry
ros2 topic echo --once /oak/left/camera_info
ros2 topic echo --once /oak/right/camera_info
ros2 topic echo --once /visual_slam/tracking/odometry
ros2 interface show isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus
ros2 topic echo /visual_slam/status --field vo_state
```

Stop each continuous observer with Ctrl-C. Verify:

- Images match CameraInfo dimensions, frame names and row stride. Supported encodings: mono8/16, bgr8/rgb8/bgra8/rgba8. Organizer reference: 1024x640 at 30 Hz; record actual values.
- Left/right capture stamps agree within 5 ms. Camera, odometry and node clock share the ROS time domain. Negative latency beyond 20 ms or image age >0.5 s fails verification.
- Odometry is finite, stamped, `header.frame_id=odom`, `child_frame_id=base_link`, unit quaternion, finite nonnegative covariance diagonals. Duplicate stamps must not keep health alive.
- VO state: 1=tracking, 0=unknown, 2=lost in the inspected organizer source. Verify the installed message/status behavior. Fresh tracking status is required by default.
- TF connects `base_link` to the left optical frame, and right optical to left optical at image time. Use the frame names actually seen:

```bash
ros2 run tf2_ros tf2_echo base_link <left-optical-frame>
ros2 run tf2_ros tf2_echo <right-optical-frame> <left-optical-frame>
```

Stereo rectification must come from calibrated CameraInfo R/P or TF. The nominal-baseline fallback is disabled in auto mode. Do not enable it for hardware verification. Record the measured camera mount, body resting height (`ground_z`), vehicle radius and launch heading. The course tube is illuminated; no blackout is expected from geometry.

## 6. Stationary dry-run perception and localization

On the actual launch mark with its known heading, use `align_mode:=first_odom`. On an unrelated bench, map alignment and map-fix accuracy are not meaningful; validate camera depth/timing there, and disable the localizer explicitly if it would be matching an unrelated room.

```bash
ros2 launch dh_autonomy autonomy.launch.py \
  course_obj:=/root/team_ws/course/dronehacks-course-v3.obj \
  dry_run:=true align_mode:=first_odom \
  require_forward_clearance:=true require_vo_status:=true
```

Set `ground_z` in the reviewed mission YAML to the measured body-center resting height before the launch; it is not the camera's height. Keep `mount_source: tf` and the measured TF. The log must say DRY RUN. With no `/dh/start`, state remains `WAIT_START` and health is still published.

In separate shells:

```bash
python3 /root/team_ws/tools/hil/check_command_isolation.py --wait 5
python3 /root/team_ws/tools/hil/stationary_report.py --period 2 --duration 120
ros2 node info /dh_mission
ros2 topic echo /dh/health
```

Pass conditions:

- No `dh_*` node owns a publisher or client under `/skybeta` or `/fmu`. The isolation script passes before and after any dry-run start. Inspect all nodes, not just topic names.
- Record camera/odometry/scan rates, latency distributions, stationary pose spread and CPU load. An 8 Hz stereo output is a target to measure, not a proven Nano capability. No fresh clearance for 0.5 s causes a hold.
- A textured board at measured body-relative ranges of 1, 1.5 and 2 m gives finite forward range within 5%. Account for camera-to-body translation. A textured distant scene may give +infinity only with sufficient beam coverage.
- Blank/dark/mismatched images, covering either lens or missing calibration produce NaN or stale data, never a clear path. Missing/invalid clearance yields zero commands and no trajectory progression in every phase.
- Stationary localization target: xy standard deviation <2 cm and span <5 cm over 120 s on the actual map. Record deviations rather than treating this target as already achieved.

## 7. Optional stationary dry-run command and failure checks

Only after command isolation passes, publish to the autonomy-only start topic:

```bash
ros2 topic pub --once /dh/start std_msgs/msg/Empty '{}'
ros2 topic echo /dh/dry_run/cmd_vel
```

No command is sent to a flight interface. If sensors are healthy and forward clearance is valid, dry-run takeoff commands may point upward. The drone stays stationary, so the reference soon freezes on tracking error. With unknown clearance, commands must be zero. The full command vector is subject to separate XY/Z limits; do not interpret `v_cap` as a 3-D norm bound.

Cover a lens briefly while stationary: expect unknown clearance/VO HOVER and zero dry-run velocity. Tracking loss >3 s or odometry loss >2 s leads to **latched ABORTED**; uncovering does not restart the mission. Restart only the autonomy process for a new stationary test, with a fresh start command. A pose discontinuity requires three accepted post-pause map fixes and reduced-speed recovery; without recovery it aborts after 3 s. Do not inject faults into real `/visual_slam/*`, `/skybeta/*` or `/fmu/*` topics; use isolated ROS tests or recorded playback for timing/pose faults.

## 8. Record evidence and stop

Use `ros2 bag record` for `/oak/*/image_raw`, both CameraInfo topics, `/tf`, `/tf_static`, odometry, VO status and `/dh/*` outputs. Record only these sensor/autonomy topics, without starting any FC bridge. Save versions, commit, calibration, timing, isolation output and all failures. Playback uses `use_sim_time:=true` with `ros2 bag play --clock`; restart autonomy between playback runs/time resets.

Stop the autonomy/perception observers with Ctrl-C. Do not change `dry_run`, arm, fly, test motors, run a raw PX4 backend or proceed to hand-carried motion under this stationary procedure.

A stationary pass qualifies sensor/ROS compatibility and isolation only. Closed-loop flight, VIO drift under motion, flight dynamics, landing and the real Sky Beta handover remain untested. The handover's zero-twist readiness stream, unlike motion, precedes Offboard in the inspected organizer implementation; test that separately in Claude's isolated simulator before considering any future hardware command path.
