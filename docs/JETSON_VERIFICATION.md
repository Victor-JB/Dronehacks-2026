# Jetson integration and stationary hardware verification guide

**Scope.** Powered, **stationary**, props-off hardware-in-the-loop testing on the issued Sky Beta drone:
- the Jetson Orin Nano, the OAK-D S2 and the onboard sensors are powered;
- **no propellers and no flight battery**;
- the organizers prohibit flying.

None of these steps arms, starts motors, or commands the flight controller. Our stack runs in **dry run** (the default), which is enforced in code:
- the mission node creates no publisher or client on `/skybeta/*` or `/fmu/*`, and **refuses to start** if one exists;
- all velocity commands go to `/dh/dry_run/cmd_vel` and the log only.

**Authorization gates.** Do not start a stage until its gate is satisfied.

| Gate | Needed before | Who |
|---|---|---|
| G0 | The first SSH session / any command on the Jetson | You, after reviewing this guide |
| G1 | Powering the Jetson and sensors (Stage C onward) | Organizers confirm powered, props-off testing is allowed |
| G2 | Hand-carried motion test (Stage G) | Organizers |
| G3 | Running the organizers' PX4 bridge / flight_interface alongside us | Organizers. **Not needed for anything in this guide** |
| G4 | `dry_run:=false` (any command to the real interface) | Organizers, at flight time only. **Out of scope here** |

**What is and is not validated by each stage**

| Stage | Validates | Does NOT validate |
|---|---|---|
| A. Read-only inspection | Versions, devices, topics, message types | Anything about our code |
| B. Build | Our packages build and import in the organizers' environment | Runtime behaviour |
| C. Sensor bring-up | Real cameras, camera_info, TF, IMU, cuVSLAM odometry, timestamps, rates | Our perception |
| D. Perception (stationary) | Real images → OpenCV → stereo depth, forward clearance, floor line | Behaviour while moving |
| E. Stationary localization | cuVSLAM + map alignment + scan-to-map matching while still: noise, bias, latency | Drift over distance, tube blackout |
| F. Command isolation | That nothing of ours reaches `/skybeta` or `/fmu`; the commands we *would* send | Real vehicle response |
| G. Hand-carried motion (optional, G2) | Localization while the sensors actually move: corrections, line fixes, tube darkness | Vehicle dynamics, tracking, speed |
| H. Recording + playback | Repeatable offline debugging on real data (`use_sim_time`) | Anything new about the hardware |
| — Flight | Tracking, PX4 lag, `v_lead`, touches, landing, timing | **Not possible under current rules** |

---

## 0. On the laptop, before any SSH (no hardware involved)

Files to transfer: only our two packages, the course model and the two HIL tools.

```text
dronehacks_autonomy/ros2_ws/src/dh_core/          -> ~/team_ws/src/dh_core/
dronehacks_autonomy/ros2_ws/src/dh_autonomy/      -> ~/team_ws/src/dh_autonomy/
dronehacks_autonomy/tools/hil/                    -> ~/team_ws/tools/hil/
dronehacks-course-v3.obj (+ .mtl)                 -> ~/team_ws/course/
```

Do **not** copy:
- `px4_msgs` (the organizers' guide says it must not be in the team workspace);
- `tools/sil` (laptop-only simulator);
- `docs/`.

Record checksums so you can confirm the files on the Jetson match:

```bash
cd dronehacks_autonomy && git log -1 --oneline && git status --short
```

```bash
find ros2_ws/src tools/hil -type f -name "*.py" -o -name "*.xml" -o -name "*.yaml" | sort | xargs sha256sum > /tmp/dh_manifest.sha256
```

---

## A. Read-only environment inspection (G0; **reads only**)

Run on the Jetson host first, then inside the organizers' container (`host/run.sh shell` per their repo). Nothing here starts a node or publishes.

| # | Command | Expected (from the organizers' repo; **verify**) | If not |
|---|---|---|---|
| A1 | `cat /etc/nv_tegra_release; cat /proc/device-tree/model` | Jetson Orin Nano, L4T R36.x (repo: R36.5.0) | Note the actual version; do not upgrade anything |
| A2 | `lsb_release -a` (host and container) | Ubuntu 22.04 in the container | Record it; Humble needs 22.04 |
| A3 | `echo $ROS_DISTRO; ros2 --version 2>/dev/null \|\| ros2 doctor --report \| head -20` (container) | `humble` | Stop: our packages target Humble |
| A4 | `cat /etc/skybeta/drone.env` | `ROS_DOMAIN_ID`, `COMPETITOR_PKG`, `COMPETITOR_LAUNCH`, camera mount | Ask the organizers |
| A5 | `docker ps; docker images \| head` | The organizers' image(s) | Use their documented start command only |
| A6 | `lsusb \| grep -i -E "03e7\|luxonis"` | Movidius / Luxonis device (OAK-D S2) | Reseat USB; ask the organizers |
| A7 | `ls /dev/ttyTHS1 /dev/video* 2>/dev/null` | `ttyTHS1` (Pixhawk link) exists; **do not open it** | — |
| A8 | `ros2 pkg list \| grep -E "skybeta\|depthai\|visual_slam\|px4_msgs\|message_filters\|tf2_ros\|launch_testing"` | skybeta_*, depthai_ros_driver(_v3), isaac_ros_visual_slam(_interfaces), px4_msgs, message_filters, tf2_ros | Missing `isaac_ros_visual_slam_interfaces`: the mission then just warns that VO status isn't monitored. Missing `message_filters` or `tf2_ros`: stop and ask |
| A9 | `python3 -c "import numpy, cv2; print(numpy.__version__, cv2.__version__)"` | numpy < 2 (repo pins it), OpenCV 4.x | Our code was tested on numpy 1.26.4 + OpenCV 4.10 and on numpy 2.1 + OpenCV 5.0 |
| A10 | `python3 -c "import scipy" 2>&1` | Either way is fine | scipy is optional; do not install it |
| A11 | `ip -br addr; echo $ROS_LOCALHOST_ONLY $SKYBETA_DDS_SCOPE` | Know which interfaces DDS uses | Keep DDS local-only unless debugging from the laptop |
| A12 | `nvpmodel -q; tegrastats --interval 1000 & sleep 3; kill %1` | Power mode, CPU/GPU load | Record it; do not change the power mode without asking |

If anything required is missing, **stop and report**. Do not install system packages or change firmware or PX4 parameters.

---

## B. Build (G0; writes only inside `~/team_ws`)

Inside the organizers' container (the repo bind-mounts `~/team_ws` at `/root/team_ws`):

```bash
cd /root/team_ws && sha256sum -c /root/team_ws/dh_manifest.sha256   # if you copied the manifest
```

```bash
cd /root/team_ws && colcon build --symlink-install --packages-select dh_core dh_autonomy 2>&1 | tail -20
```

```bash
source /root/team_ws/install/setup.bash && ros2 pkg executables dh_autonomy
```

- **Expected:** `dh_autonomy line`, `localizer`, `mission`, `sitl_odom`, `stereo_scan`.
- **If it fails:**
  - missing `ament_python` deps: use the organizers' `team_build.sh` / `host/run.sh build` path;
  - never add `px4_msgs` to the workspace.

Laptop-equivalent tests (no hardware, no ROS graph):

```bash
python3 /root/team_ws/src/dh_core/test/test_core.py
```

Expected: 17 × `PASS`.

---

## C. Sensor bring-up (G1; camera + cuVSLAM **only**, no PX4 bridge)

Start **only** the camera + visual-SLAM launch from the organizers' bring-up package (repo: `oakd_vslam_bringup/launch/oakd_vslam.launch.py`). Do **not** start `px4_bridge.launch.py`, the MicroXRCE agent, or `skybeta_flight_interface`. With those absent, nothing on the Jetson can reach the Pixhawk. If the organizers' `host/run.sh stack` always starts all windows, ask them how to start the sensor part alone. Reading their repo is fine; editing it is not.

Checks (all **read-only**):

| # | Command | Expected | Common failure → action |
|---|---|---|---|
| C1 | `ros2 topic list \| grep -E "^/oak\|^/visual_slam\|^/tf"` | `/oak/left/image_raw`, `/oak/right/image_raw`, `/oak/*/camera_info`, `/oak/imu/data`, `/visual_slam/tracking/odometry`, `/visual_slam/status`, `/tf`, `/tf_static` | No `/oak`: USB/driver → organizers |
| C2 | `ros2 topic hz /oak/left/image_raw` (and right) | ~30 Hz (repo: 1024×640 @ 30) | ~23 Hz → XLink bandwidth; report it |
| C3 | `ros2 topic echo --once /oak/left/image_raw --no-arr \| grep -E "encoding\|height\|width\|step\|frame_id"` | `bgr8` (repo) or `mono8`, 640×1024, a frame_id | Other encodings: ours accepts mono8/mono16/bgr8/rgb8/bgra8/rgba8; anything else is rejected with a log |
| C4 | `ros2 topic echo --once /oak/left/camera_info`, then `/oak/right/camera_info` | `k` non-zero; `width/height` match the images; **`p[3]` of the RIGHT camera non-zero** (= −f·B) | `p[3]` = 0 → the stereo node uses TF (C6), else assumes 0.075 m and warns |
| C5 | `ros2 topic delay /oak/left/image_raw` | Small, stable (stamps = capture time on the ROS clock) | Large or negative → the driver stamps on another clock: **HARDWARE VERIFICATION FAILED**, see §Known risks |
| C6 | `ros2 run tf2_ros tf2_echo <right_optical_frame> <left_optical_frame>` (frame names from C3/C4) | Translation ≈ 0.075 m along x | Missing → rectification falls back to the 0.075 m assumption (biased if the pair is rotated) |
| C7 | `ros2 run tf2_tools view_frames` (writes a PDF; read-only) | base_link ↔ camera frames connected | Note the frame names for `base_frame` |
| C8 | `ros2 topic hz /visual_slam/tracking/odometry` and `ros2 topic echo --once ... --no-arr` | ~30 Hz; `header.frame_id: odom`; stamp non-zero; covariance present or zero | Different frame_id → set `odom_frame`; zero stamps → map fixes cannot be synchronised (our node warns) |
| C9 | `ros2 interface show isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus` and `ros2 topic echo /visual_slam/status --field vo_state` | `vo_state` field present; value while tracking (expect 1) and when the lens is covered (repo guide: 2 = lost) | Different numbering → set `LocHealthParams.vo_lost_states` |
| C10 | `ros2 topic info -v /visual_slam/tracking/odometry` and `ros2 topic info -v /oak/left/image_raw` | Shows each publisher's QoS reliability | Our subscribers are best-effort: compatible with both |

---

## D. Perception, stationary (G1; our perception nodes only; **no mission node yet**)

```bash
ros2 run dh_autonomy stereo_scan --ros-args --params-file /root/team_ws/install/dh_autonomy/share/dh_autonomy/config/competition.yaml
```

```bash
ros2 run dh_autonomy line --ros-args --params-file /root/team_ws/install/dh_autonomy/share/dh_autonomy/config/competition.yaml
```

Read-only checks with the drone on a stand, facing a wall at a measured distance:

| # | Command / action | Expected | Failure → action |
|---|---|---|---|
| D1 | Stereo node log | `rectification ready (camera_info\|tf\|baseline): f=… B=0.075` | `baseline` source → depth may be biased; check D3 carefully |
| D2 | `ros2 topic hz /dh/stereo_scan` | ≥ 8 Hz (SGBM 512×320 on the Orin CPU, **unmeasured**) | Much lower → set `scale: 0.4` or `num_disparities: 48` |
| D3 | Hold a textured board at 1.00 m, 1.50 m, 2.00 m (tape measure from the lens): `ros2 topic echo /dh/forward_clearance --field range` | Within ±5 % (synthetic tests: 1–3 %) | Systematic error → `camera_pitch_deg`, `camera_x`, baseline (C4/C6) |
| D4 | Point at a blank, untextured wall, or cover one lens | `range: .nan` (unknown), **never** `inf` | `inf` here = BUG: report it |
| D5 | Point at open space with texture > 4 m away | `range: inf` (measured clear) | `nan` → lighting or texture too poor; record it |
| D6 | Cover both lenses / switch the lights off | `nan`, log `image too dark for stereo` | — |
| D7 | `ros2 topic echo --once /dh/stereo_scan --no-arr` | `header.frame_id: base_link`; stamp = image stamp | — |
| D8 | `ros2 topic delay /dh/stereo_scan` | Processing latency (expect 50–150 ms) | > 0.5 s → scans are rejected as stale; tune `scale` |
| D9 | Lay the guideline tape (organizers' floor line) under the camera: `ros2 topic echo /dh/line_obs` | lateral ≈ measured offset, z (confidence) ≥ 0.5 | Nothing → tune `dark_delta` (black-hat threshold) |

Measure and record: `camera_pitch_deg`, `camera_x`, `camera_z` from `/etc/skybeta/drone.env`, and `robot_radius` (half tip-to-tip span + margin).

---

## E. Stationary localization (G1; full stack in DRY RUN)

Place the drone **on the launch mark of the course, facing north** (or on a bench, if the course isn't available: then only noise and latency are meaningful).

```bash
ros2 launch dh_autonomy autonomy.launch.py course_obj:=/root/team_ws/course/dronehacks-course-v3.obj align_mode:=first_odom
```

`dry_run` defaults to `true`. Confirm the log line `DRY RUN: no publisher to /skybeta/* or /fmu/*`.

```bash
python3 /root/team_ws/tools/hil/stationary_report.py --period 2 --duration 120
```

| # | Expected | Failure → action |
|---|---|---|
| E1 | `odom ~30 Hz`, `stereo_scan ≥ 8 Hz`, `pose_map ~30 Hz`, `map_fix` > 0 Hz on the launch mark | `map_fix` 0 Hz → `ros2 topic echo /dh/loc_health`: `[ok, inlier, rms, eig, latency]` |
| E2 | Stationary `pose_map` xy std < 2 cm, span < 5 cm over 2 min | Larger → VO noise or map-fix fighting; record a bag (H) |
| E3 | `pose_map` ≈ the launch-mark coordinates printed in the mission log | Offset → launch heading or `launch_yaw_map` wrong |
| E4 | `health: OK fc=CLEAR\|MAPPED_WALL` | `HOVER odometry stale` → odometry rate or stamps (C8) |
| E5 | Cover the lenses for 5 s | Health `HOVER visual tracking lost` (if VO status is available), clearance `nan`; recovers when uncovered | No change → C9 |
| E6 | `ros2 topic echo /dh/mission_state` | `WAIT_START` (no start sent) | — |

---

## F. Command-path isolation (G1; **reads only**)

With stage E running:

```bash
python3 /root/team_ws/tools/hil/check_command_isolation.py --wait 5
```

- **Expected:** `RESULT: PASS - no dh_* publisher or client on /skybeta/* or /fmu/*`, and `/dh/dry_run/cmd_vel publishers: ['dh_mission']`.
- **If it fails:** stop everything (`Ctrl-C`) and report it. Exit code 1 means one of our nodes owns a flight-control endpoint, which the code is supposed to make impossible.

Independent confirmation from the ROS CLI:

```bash
ros2 topic list | grep -E "^/skybeta|^/fmu" || echo "no flight-control topics exist at all (bridge not running) - expected in this guide"
```

```bash
ros2 node info /dh_mission | sed -n '/Publishers:/,/Service Servers:/p'
```

Expected publishers: `/dh/dry_run/cmd_vel`, `/dh/dry_run/events`, `/dh/pose_map`, `/dh/plan`, `/dh/mission_state`, `/dh/health`, `/rosout`, `/parameter_events`. **Nothing** under `/skybeta` or `/fmu`.

Then exercise the command generation while stationary. This publishes only to our own `/dh/start`, never to a flight interface:

```bash
ros2 topic pub --once /dh/start std_msgs/msg/Empty {}
```

```bash
ros2 topic echo /dh/dry_run/cmd_vel --field twist
```

Expected:
- dry-run authority is granted (`/dh/dry_run/events`);
- TAKEOFF commands point up (+z, ≤ 0.6 m/s);
- because the drone does not move, the follower's error grows and the reference clock freezes at 0.35 m.

That is the correct behaviour for a stationary drone, not a defect. The command norm never exceeds `v_cap` (1.0 m/s). Re-run `check_command_isolation.py` after the start: it must still PASS.

---

## G. Hand-carried motion (optional, needs G2; props removed, flight battery out)

The only way to exercise localization **in motion** without flying:
1. Carry the powered drone slowly (≤ 0.5 m/s) along the planned route at about 0.85 m height, through the maze, the tube and the window. Record a bag (H).
2. Watch `stationary_report.py`, `/dh/health`, `/dh/map_fix` and `/dh/line_obs`.

Expected:
- `pose_map` follows the true position;
- map fixes keep arriving except in the tube;
- `fc=INVALID` inside the tube is accepted as expected there;
- the line fixes apply in the first corridor.

This validates sensing and localization only; it does not validate flight. Dry run stays on, so the commands remain on `/dh/dry_run/cmd_vel`.

## H. Recording and playback

Record (organizers' tool: `host/run.sh bag`, or):

```bash
ros2 bag record -s mcap -o /root/team_ws/bags/hil_$(date +%H%M%S) /oak/left/image_raw /oak/right/image_raw /oak/left/camera_info /oak/right/camera_info /tf /tf_static /visual_slam/tracking/odometry /visual_slam/status /dh/stereo_scan /dh/forward_clearance /dh/pose_map /dh/map_fix /dh/loc_health /dh/health /dh/line_obs /dh/mission_state /dh/dry_run/cmd_vel
```

Playback, on the Jetson or any Humble machine. Stamps must be replayed with the bag clock, so set `use_sim_time`:

```bash
ros2 bag play <bag> --clock --topics /oak/left/image_raw /oak/right/image_raw /oak/left/camera_info /oak/right/camera_info /tf /tf_static /visual_slam/tracking/odometry /visual_slam/status
```

```bash
ros2 launch dh_autonomy autonomy.launch.py course_obj:=... use_sim_time:=true align_mode:=first_odom
```

Then analyse:

```bash
python3 tools/analyze_bag.py <bag> out
```

Rule 6.4: bags are for debugging and tuning only. Never load data recorded on the course as an input to a later run.

---

## Known risks to check on the hardware (none can be verified on the laptop)

1. **Clock domain.** All synchronisation assumes the OAK image stamps, cuVSLAM odometry stamps and the node clock are the same ROS clock (C5, D8). If the driver uses device time, delayed-scan handling breaks: the localizer rejects scans as `future` or `stale` (counters in `LocalizerNode.rejected`).
2. **Stereo calibration source** (C4/C6) and SGBM rate on the Orin CPU (D2): unmeasured.
3. **`vo_state` semantics** (C9): taken from the organizers' guide (2 = lost), unverified.
4. **`/skybeta/state` fields** (`offboard_active`, `frozen`): from the organizers' guide. They matter only when `dry_run:=false`.
5. **Real wall texture and lighting**: all perception tests so far used rendered images.
6. **Landing / CP5 and the return**: the Sky Beta interface cannot land (0.3 m floor). This is unchanged and needs an organizer decision.
