# Jetson Compatibility Report — skybeta-02 — 2026-11-07

## Scope and evidence

Read-only inspection completed through the user's existing SSH session and a separate password-authenticated audit connection. No credentials were saved. No sudo, installation, update, deployment, service changes, PX4 configuration changes, autonomy startup or physical command publication was performed. The separate audit connection was closed. Existing organizer/team processes were left running.

Baseline autonomy: `0ec8c81` on `saketh/autonomy`. Latest team-main context read from `9186743`; the other teammates' context files were preserved. Raw read-only observations are delivered separately as `jetson-readonly-evidence.json`. Brief ROS subscriber/TF observers disabled external file logging and rosout, and were destroyed after sampling. They did not create command publishers or call control services. Initial CLI logging-to-/dev/null queries failed safely; direct rclpy observation with external logging disabled succeeded.

## Actual hardware, OS and resources

| Item | Observed fact |
|---|---|
| Model | NVIDIA Jetson Orin Nano Engineering Reference Developer Kit Super |
| Architecture / kernel | aarch64, Linux 5.15.185-tegra |
| OS | SSH banner: Ubuntu 22.04.5 LTS |
| L4T | 36.5.0-20260115194252; R36 revision 5.0 |
| JetPack | `nvidia-jetpack` meta-package version was not returned; do not infer an exact JetPack release from CUDA alone |
| RAM / swap | 7.4 GiB RAM; about 3.0 GiB used, 4.2 GiB available; 3.7 GiB swap, about 137 MiB used |
| Storage | /dev/nvme0n1p1, 233 GiB filesystem; latest sample 134 GiB used, 87 GiB available |
| CUDA metadata | SDK 12.6.11; cudart 12.6.68; physical CUDA workload not benchmarked |
| OpenCV CUDA | `cv2.cuda.getCudaEnabledDeviceCount()` = 0; the existing SGBM code is CPU based |
| Connected camera evidence | USB 03e7:f63b Movidius VPU; organizer camera configuration identifies OAK-D S2. SDK device-open/serial enumeration was deliberately not performed |
| IMU | Existing `/oak/imu/data` is live; installed config identifies BNO086, requested 200 Hz |
| Downward range | PX4 local position has dist_bottom ~0.100 m but **dist_bottom_valid=false**; that numeric value is not usable clearance |

## Running environment and ownership

- Existing containers: `skybeta-stack` using `skybeta-team:0.3.3`; status sidecar `skybeta:0.3.3`.
- Active services: docker, skybeta-stack, skybeta-ros-status and skybeta-status. The stack unit is `/etc/systemd/system/skybeta-stack.service` and starts at boot. No unit was edited or restarted.
- Organizer source at `/home/dronehacks/skybeta`, observed Git HEAD `3c42933f5adc7b22997614946f6719b2c2f89f20`.
- Workspaces: `/root/skybeta_ws` with organizer packages; `/root/team_ws/src/dronehacks`, host `/home/dronehacks/team_ws/src/dronehacks`. Existing team software includes camera_web, dronehacks_map, rosbag recorder and Foxglove. No existing team controller was interrupted.
- ROS **Humble**, domain **2**, RMW `rmw_fastrtps_cpp`; ROS is in the container, not established as a host installation.
- Python **3.10.12**, numpy **1.26.4**, OpenCV **4.5.4**, depthai SDK **3.10.0**. Package versions: isaac_ros_visual_slam **3.2.6**, depthai_ros_driver_v3 **3.3.0**, skybeta_flight_interface package.xml **0.1.0** (image tag remains 0.3.3).
- Imports/packages confirmed: rclpy, message_filters, tf2_ros, sensor_msgs, nav_msgs, geometry_msgs, std_srvs, launch, launch_ros, isaac_ros_visual_slam_interfaces, px4_msgs and launch_testing. No missing core dependency was established. The feature packages were not built or deployed in this audit.

## Observed interfaces, QoS and data

Rates are **subscriber receipt measurements**, include discovery/observer load, and are not guaranteed source rates. Early discovery showed subscription-only topics; a longer observation found the publishers. Topic existence alone was not treated as sensor availability. DDS reported depth=0 for remote endpoints; that value is not evidence that the configured queue has zero capacity.

| Topic | Type | Publisher; observed subscribers | QoS / rate / frame | Autonomy use |
|---|---|---|---|---|
| /oak/left/image_raw, /oak/right/image_raw | sensor_msgs/Image | /oak; left/right_format_converter | reliable, volatile; 960x600 bgr8, step 2880; configured 30 Hz, observed 11.5-17.3 Hz under multi-stream audit load; left/right optical frames | Default SGBM and line inputs |
| /oak/left/image_mono, /oak/right/image_mono | sensor_msgs/Image | GPU format converter nodes; cuVSLAM pipeline | mono8, 960x600, step 960; both observed 24.75 Hz in a lighter 4 s observation; matching optical frames | Unit-02 profile reuses these supported grayscale inputs |
| /oak/left/camera_info, /oak/right/camera_info | sensor_msgs/CameraInfo | /oak; /visual_slam_node, recorder/sidecar for left | reliable, volatile; ~30 Hz in one sample, matching 960x600 dimensions | Calibration for stereo/line |
| /oak/stereo/image_raw | sensor_msgs/Image | /oak; /camera_web, /dronehacks_map, recorder | reliable, volatile; 320x200 **16UC1**, step 640; observed 29 Hz; header oak_rgb_camera_optical_frame | Not connected to our raw-stereo pipeline; do not use as a mono eye |
| /oak/stereo/camera_info | sensor_msgs/CameraInfo | /oak; /dronehacks_map, recorder | reliable, volatile; 320x200; RGB optical frame label | Depth-adapter calibration, if separately validated later |
| /oak/imu/data | sensor_msgs/Imu | /oak; recorder and status sidecar | live ~196.3 Hz observed; oak_imu_frame | Organizer estimator input; no new IMU estimator added |
| /visual_slam/tracking/odometry | nav_msgs/Odometry | /visual_slam_node; /px4_odom_bridge, /flight_interface, map, recorder, Foxglove, sidecar | reliable, volatile; observed 22.9-29.5 Hz; **odom -> base_link** | Mission odometry/history/health |
| /visual_slam/status | isaac_ros_visual_slam_interfaces/VisualSlamStatus | /visual_slam_node; bridge, recorder, sidecar | reliable, volatile; ~27 Hz; header map; vo_state=1 observed | Required tracking health; source uses header capture stamp |
| /skybeta/state | std_msgs/String JSON | /flight_interface; recorder/sidecar | reliable, volatile, 5 Hz | Backend authority/state, read-only in stationary mode |
| /skybeta/setpoint/twist | geometry_msgs/TwistStamped | **no publisher in observed snapshot**; /flight_interface subscriber | reliable, volatile subscription | Physical command output only in non-stationary mode; never published during audit |
| /skybeta/setpoint/pose | geometry_msgs/PoseStamped | Advertised in graph | Contents/publishers not exercised | Existing physical interface, not used by the audit |
| /fmu/out/vehicle_status_v1 | px4_msgs/VehicleStatus | bare DDS/PX4 endpoint; recorder, sidecar | best effort, transient local | Physical state discovery; raw backend remains simulation-only |
| /fmu/out/vehicle_local_position | px4_msgs/VehicleLocalPosition | bare DDS/PX4 endpoint; recorder, sidecar, Foxglove | best effort, transient local; ~86.8 Hz received in one window | Read altitude validity; not an autonomy input replacement |
| /tf_static | tf2_msgs/TFMessage | base_link_to_oak, oak_state_publisher, oak, visual_slam_node | reliable, transient local; non-periodic; TF Buffer successfully resolved required links | Optical-to-body and stereo extrinsics |
| /tf | tf2_msgs/TFMessage | Dynamic graph observed | Not rate-qualified in this audit | Dynamic pose frames; no course->odom broadcaster added |

Other observed PX4 outputs include attitude, odometry, land detection, control mode, estimator flags and command acknowledgments. No separate downward sensor output topic was found in the sampled graph; `/fmu/in/distance_sensor` was a subscriber endpoint, not a source. No `/dh/forward_clearance` producer was observed; our feature's stereo node would produce it after a future authorized launch.

Actual installed message definitions were inspected through generated `get_fields_and_field_types()`: VisualSlamStatus has `header`, `vo_state` and execution timing fields; Image has encoding/endianness/stride/data; CameraInfo has K/D/R/P; Odometry has header/child_frame_id/pose/twist; TwistStamped has header/twist. Those fields match our consumers.

## Calibration and timing findings

Camera pipeline is a **team override** of the stock organizer configuration: depth enabled, mono sockets requested 960x600 at 30 Hz, IMU 200 Hz, on-device stereo decimation enabled. Do not replace or reconfigure it during this stage. Topic labels alone do not prove whether lens/rectification corrections are already applied. Zero D and identity R were reported; physical calibration/depth accuracy remains unverified.

Observed CameraInfo:

- Left K: fx=595.1951, fy=595.3043, cx=487.3531, cy=305.8477; left P03=-44.82495.
- Right K: fx=597.6473, fy=597.5057, cx=481.5436, cy=298.5870; right P03=0.
- rational_polynomial, 14 zero distortion coefficients in the sampled left message.
- Because the **right** P03 is zero, our auto mode correctly falls back to factory TF rather than assuming the usual CameraInfo baseline layout. A local regression using these observed matrices and TF confirms valid rectification maps and baseline magnitude; it does not establish image accuracy.

Factory TF, left optical expressed in right optical:

```text
translation [-0.0753113556, 0.0006002480, 0.0002390595] m
quaternion  [-0.0011172621, 0.0001156801, -0.0015947103, 0.9999980976]
```

Base-link <- left optical:

```text
translation [0.1059459194, 0.0377736750, 0.0302884640] m
quaternion  [-0.4991339548, 0.5042654698, -0.4990317645, 0.4975372806]
```

Base-link <- right optical translation is [0.1052261447, -0.0375369596, 0.0303777213] m. Nominal mount settings in drone.env: X=0.1053, Y=0, Z=0.0304, roll/pitch/yaw=0. Our default measured-TF path already handles the full lateral offset and rotation. The old 20 degree parameter assumption must not be used for this unit's line-kernel estimate or parameter-only geometry.

Timing, 8 s observation including subscriber discovery/load:

- Image age medians ~51-52 ms; right maximum ~303 ms, left maximum ~74 ms.
- Odometry age median ~63 ms, p95 ~279 ms, maximum **422 ms**. This crosses the existing 400 ms hold threshold. Do not increase that threshold merely to hide load/jitter.
- Nearest received L/R capture stamps: median ~24 microseconds, p95 ~66.6 ms, 47/92 left receipts within 5 ms of a received right sample. This reflects the received set, including drops/discovery; it does not prove camera hardware desynchronization. Keep the 5 ms pair guard and measure usable paired throughput during controlled testing.
- A lighter mono-only sample received each eye at 24.75 Hz and odometry at 29.5 Hz. This is not a controlled performance comparison, but supports testing the already-converted mono path to reduce additional transport/conversion work.
- Odometry covariance was finite; position diagonal roughly [0.00204,0.00409,0.00203] m²; quaternion norm ~0.999999975. Frames and current timestamp ages are compatible. Clock startup/NTP/reset behavior is not qualified; status container uptime oddly reported 56 years, so no whole-boot synchronization guarantee is claimed.

## Sky Beta/PX4 state and control boundary

Observed state: px4_link=true, armed=false, landed=true, offboard_active=false, nav_state_name=AUTO_LOITER, envelope_ok=true, frozen=false, setpoint_mode=position, clamped=[min_altitude], envelope 0.3-3.0 m and 1.5 m/s. These are observations, not commands or approval to change flight state. Startup COMPETITOR_PKG=manual and COMPETITOR_LAUNCH=mission.launch.py were read, not changed.

The physical TwistStamped subscriber exists and state fields match our backend expectations. Command acceptance, handover, flight response, hold-service behavior and landing authority were **not** tested. No zero command was sent. Existing bridge/interface heartbeat traffic belongs to the pre-existing stack; it was neither started nor stopped by this audit.

## Compatibility matrix

| Actual interface | Autonomy expectation | Classification | Required fix / next evidence |
|---|---|---|---|
| Humble, Python3.10 | Humble Python ROS packages | VERIFIED COMPATIBLE at package/type/grammar level | Actual package build/import remains a controlled deployment check |
| numpy1.26.4 | numpy pure core | VERIFIED COMPATIBLE version; local tests | No upgrade |
| OpenCV4.5.4, SGBM/stereoRectify symbols present | CPU SGBM and calibration APIs | HARDWARE VERIFICATION PENDING for real processing | Exact-version local Windows wheel unavailable; no target install attempted |
| Core ROS imports and message fields present | Manifest runtime dependencies | VERIFIED COMPATIBLE observed imports | No missing core system dependency established |
| 960x600 bgr8 and mono8, matching CameraInfo | Dynamic raster size and supported encodings | VERIFIED COMPATIBLE message format | Board/blank-wall behavior still untested |
| CameraInfo right P03=0, valid factory TF | Auto CameraInfo then TF fallback | VERIFIED COMPATIBLE selection logic | Regression added with measured matrices; physical depth accuracy pending |
| Level mount, nonzero optical lateral offsets | TF authoritative; old parameter defaults20deg | CODE CHANGE REQUIRED for unit-specific parameters | Explicit jetson_02 profile supplied; TF still authoritative |
| Jittered/delayed VO | Historical capture-time association | CODE CHANGE REQUIRED in bounded retention | Retain up to16 images/scans instead of prematurely discarding; age/future/gap gates unchanged |
| Reliable volatile sensor publishers | Best-effort sensor subscribers | VERIFIED COMPATIBLE DDS QoS direction | Actual feature-node DDS startup still untested |
| TF reliable/transient-local | tf2 listener | VERIFIED COMPATIBLE observation | Successful pair/body lookups; no TF rewriting |
| Odom frame odom / base_link; finite covariance | Exact frame/pose health contract | VERIFIED COMPATIBLE sampled data | Peak age crosses timeout; full performance qualification pending |
| VO status state1, stamped map header | vo_state + capture stamp | VERIFIED COMPATIBLE sampled tracking | Tracking-loss injection not performed |
| On-device depth 16UC1, RGB-labelled frame | Our raw-stereo path, not depth adapter | HARDWARE VERIFICATION PENDING | Do not silently substitute depth input; units/frame/board accuracy need separate review |
| Downward dist_bottom_valid=false | Not required by present follower | HARDWARE VERIFICATION PENDING | Do not trust invalid altitude/range or infer landing compatibility |
| Sky Beta JSON + TwistStamped subscriber | ENU odom backend contract | VERIFIED COMPATIBLE type/fields only | Physical commands/handover require organizer permission and remain untested |
| No feature launch executed | Packages, manifests, launch files | HARDWARE VERIFICATION PENDING | Build/install only after explicit next-stage authorization |
| Need guaranteed stationary command separation | Configurable normal mission | CODE CHANGE REQUIRED | Dedicated stationary executable now forces DryRunBackend, tested against false/px4 overrides |
| Initial graph discovery incomplete | Isolation checker must prove dry-run active | CODE CHANGE REQUIRED | Checker now requires dh_mission plus its dry-run command publisher; incomplete graph cannot pass |
| Existing full stack/controller processes | No competing hardware operations | ORGANIZER PERMISSION REQUIRED | Coordinate before any later deployment; do not change existing services now |

## Local changes and tests

The following local changes are committed/published on saketh/autonomy after validation; no deployment is included:

1. `StationaryMissionNode` and `stationary_mission` entrypoint force the dry backend in code, independent of dry_run/backend parameters. Existing endpoint isolation remains enforced before command generation.
2. `stationary.launch.py` uses that executable; the normal launch accepts an explicit params_file. Default normal mission behavior is retained.
3. `jetson_02.yaml` is an explicit profile using observed level-mount parameters and existing left/right mono topics. Pair queue retention16 is bounded; capture slop5ms and sensor age0.5s remain unchanged. No measured robot radius or ground_z is invented.
4. Image/scan queues retain up to16 capture-time observations while waiting for odometry; line handling no longer drops a still-fresh image merely because VO lags by more than100ms. It still requires a bracketing pose and rejects stale data.
5. Isolation checker requires positive mission/dry-publisher evidence, not just the absence of forbidden endpoints.
6. Regression tests cover forced isolation despite false/px4 settings, incomplete graph rejection, delayed line height lookup, and the observed unit-02 calibration.

Local results: 17 core tests pass under the current laptop runtime; 16 safety/compatibility regressions pass, including the observed calibration and delayed-image case. The isolated numpy1.26.4/OpenCV4.10 environment passes 47 unit tests (17 core +30 preserved team), 12 existing fake-bus integration tests, all 16 safety regressions, 12 stereo scenarios and 14 line scenarios. Python3.10 grammar parsing, compileall and git diff checks pass. The exact OpenCV4.5.4 binary was not available for this laptop; API symbols were inspected on the Jetson, but no real image processing was run there. Actual ROS package build/launch/DDS integration of these feature nodes, full Nano throughput and physical command isolation remain pending controlled deployment. No old full-course simulation benchmark is represented as a new hardware test. No planner, trajectory timing, follower or flight safety thresholds were replaced or relaxed. No installed system package was changed.

## Safe next steps — separate controlled-deployment approval required

1. Review this report and the commits. Physically re-confirm props removed, flight battery absent, approved independent sensor power, and stationary support. Observed armed=false is not proof of those physical conditions.
2. Confirm the intended Nano/container/workspace and existing controller ownership with the team/organizers. Preserve existing source and do not change COMPETITOR_PKG, firmware, PX4 parameters or services as part of this review.
3. After new approval, transfer only reviewed packages/tools to a non-conflicting workspace location, compare existing packages, record checksums, then build dh_core/dh_autonomy in the existing Humble image. No `px4_msgs` vendoring or package upgrades.
4. Run prepared **synthetic ROS launch tests** in a separate localhost-only domain77 without live camera/FC transport. Check colcon results. Those tests were not run on the Jetson during this audit.
5. For the later live stationary check use domain2 and the installed overlay with the dedicated entrypoint, e.g.:

```bash
ros2 launch dh_autonomy stationary.launch.py \
  params_file:=/root/team_ws/install/dh_autonomy/share/dh_autonomy/config/jetson_02.yaml \
  course_obj:=<reviewed-existing-course-path> align_mode:=first_odom \
  use_localizer:=false use_line:=false
```

This is a **future command**, not executed. A bench unrelated to the course must not be matched to the course. After confirming placement/heading/resting height on the actual launch mark, review enabling line/localizer. Keep required clearance and VO health enabled. Do not switch to the normal mission executable for stationary verification.

6. Require a positive isolation check (allow >8s discovery), inspect node endpoints and /dh/dry_run output, then measure real SGBM depth against boards, blank-wall unknown handling, capture pairing, 95th/maximum ages, CPU/memory and usable output rate under the complete pipeline. A hold caused by 0.4s odometry age is expected safety behavior, not grounds to loosen it.
7. Only after isolation passes may a separately approved `/dh/start` exercise calculate stationary desired commands. Those outputs must remain exclusively `/dh/dry_run/*`; do not publish physical zero twists to test the interface. Stationary trajectories do not validate navigation performance.
8. Record failures and stop. No arming, mode changes, motor tests, physical commands, deployment, autonomy startup or full integration launch was performed in this audit.

## Remaining limitations

Exact JetPack meta-version, physical camera identity/serial, lens/rectification behavior, depth units/frame labeling/metric accuracy, full pipeline throughput, whole-boot clock stability, motion tracking, and physical control response remain unqualified. Current required message interfaces and TF were observed successfully, but actual autonomy package build/launch and real OpenCV processing on the Nano remain pending. No missing dependency should be inferred from a deliberately unperformed installation/build or a transient discovery snapshot.

JETSON COMPATIBILITY AUDIT COMPLETE — READY FOR CONTROLLED DEPLOYMENT REVIEW.
