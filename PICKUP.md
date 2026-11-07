# Pick up here

Where the drone work stands and what to do next. How the system works: `SYSTEM.md`.

## 1. Next biggest fix: depth makes VIO too slow for PX4

**Status: blocking.** With the depth team image on, PX4 cannot hold a position, so the drone cannot
fly autonomously.

### What happens

The mission sits in `wait` with `not ready ...: estimator unhealthy`. Its health check needs PX4 to
have a local position, and PX4 has none: VIO reaches PX4 at 30 Hz, but
`/fmu/out/vehicle_local_position` shows `xy_valid: false`, `heading_good_for_control: false` and a
climbing `xy_reset_counter` (13 when checked). PX4's estimator rejects the vision samples because
they are too old: it can only compensate a vision delay up to `EKF2_EV_DELAY`, which is at most
300 ms, and the organisers' value assumes ~60–70 ms.

### Measurements (2026-11-06, drone on the bench, disarmed)

`timestamp - timestamp_sample` on `/fmu/in/vehicle_visual_odometry`, removing our nodes one at a time:

| Running | VIO age at the bridge |
|---|---|
| everything (fly.sh session, camera page open) | 480–900 ms |
| camera page stopped | ~300 ms |
| map, bag, Foxglove stopped too | ~290 ms |
| mission stopped too (only the organisers' stack) | 280–294 ms, steady |

The organisers' probe (`~/skybeta/tools/latency_probe.py --seconds 20`, copied into the container),
stack only:

| Leg | Now | Organisers' baseline (`~/skybeta/CLAUDE.md`) |
|---|---|---|
| camera -> depthai -> ROS publish (driver leg) | p50 **277 ms**, p95 291 | 36.5 ms |
| full chain to the bridge | p50 **290 ms**, p95 302 | p50 60–70 ms, p95 ~100 |

Almost all the extra delay is in the **driver leg**, before cuVSLAM and before any of our code. The
only change there is our depth override (`skybeta_overrides/oakd_vslam.yaml`). Separately, our camera
page added several hundred ms of queueing while it ran (§2.6).

### Most likely cause (not yet proven)

The OAK's USB link tops out near 143 MB/s. The two eyes cuVSLAM uses go over it as bgr8 (3 bytes per
pixel from a mono sensor), ~118 MB/s at 1024x640 @ 30 Hz (organisers' table). Depth adds ~10 MB/s, so
the link runs at its ceiling. Two signs point to queueing, not slow computation: the left camera fell
from 30 Hz to ~24 Hz, and the added delay is a steady ~240 ms, about 8 frames (8 x 33 ms) waiting
in a queue.

### The rule from now on

After **any** camera or pipeline change, run the organisers' latency probe and check PX4's
`xy_valid`. Frame rates alone are not enough: that is how this slipped through.

```sh
docker cp ~/skybeta/tools/latency_probe.py skybeta-stack:/tmp/ && \
docker exec -it skybeta-stack bash -ic "python3 /tmp/latency_probe.py --seconds 20"
docker exec skybeta-stack bash -ic "ros2 topic echo --once --qos-reliability best_effort \
  --qos-durability transient_local /fmu/out/vehicle_local_position --field xy_valid"
```

Accept a change only if the full chain stays near **100 ms or less (p50)**, the left camera stays at
30 Hz and `xy_valid` is `true`. Do **not** raise `EKF2_EV_DELAY` to ~290 ms to compensate: that is at
PX4's limit, and a position estimate that old makes hover sluggish and unstable.

## 2. Fixes, cheapest first

| # | Fix | What changes | USB bandwidth | Risk |
|---|---|---|---|---|
| 2.1 | **Depth off: confirm the baseline** | `rm ~/team_ws/Dockerfile && docker rm -f skybeta-stack` (on the ground, drone still). The stack falls back to the stock image | ~118 MB/s | None, it is the organisers' setup. But the mission cannot arm without depth |
| 2.2 | **Smaller eyes** | In `skybeta_overrides/oakd_vslam.yaml`, left/right 1024x640 -> **960x600**, which the organisers' table lists as "clean, more margin". Rebuild the team image, restart the stack | ~104 + 10 MB/s | Small: VIO gets ~12% fewer pixels |
| 2.3 | **Shorter queues** | Make the driver drop old frames instead of queueing them | same | Lower latency but more dropped frames; treats the symptom |
| 2.4 | **Mono8 rectified eyes from the OAK** (NVIDIA's reference setup) | In depth mode the OAK can publish rectified left/right as mono8 (1 byte per pixel). Point cuVSLAM at those (with their rectified camera_info) instead of the bgr8 raw images, and stop publishing bgr8 | **~50 MB/s total** | The proper fix and what NVIDIA's `isaac_vslam.yaml` does, and the organisers' notes point at it ("go to 1280x800 only if you first find a way to get mono8 off the device"). But it rewires the organisers' VIO input: **ask them first** |
| 2.5 | **Depth computed on the Jetson** | Leave the camera stock. Rectify `/oak/{left,right}` images with their camera_info and run OpenCV StereoSGBM at ~320x200, 5 Hz, publishing a 16UC1 depth image | none | No GPU stereo package in the image, so CPU next to cuVSLAM; worse depth than the OAK's; more code. Fallback if the organisers say no to 2.4 |
| 2.6 | **Camera page off the GPU topic** (needed with any option) | `camera_web` subscribes to `/oak/left/image_mono`, the output of the organisers' GPU converter that also feeds cuVSLAM; a plain subscriber makes it copy every frame back to the CPU. Switch it to `/oak/left/image_raw` (straight from the driver), or drop the live camera view and keep depth + map | none | Measure it like the others |
| 2.7 | **Latency gate in `fly.sh`** | `fly.sh start` runs the probe and refuses to start when the full chain p50 is over ~120 ms or `xy_valid` is false | none | None |

### Suggested order

1. **2.1**: depth off, re-run the probe, confirm ~60–70 ms and `xy_valid: true`.
2. **2.2**: 960x600 + depth, probe again. Keep it only if it passes the rule in §1.
3. If 2.2 is not good enough: ask the organisers about **2.4**.
4. **2.5** if they do not want their VIO input touched.
5. **2.6** and **2.7** whichever way depth goes.

## 3. Also open

- `route.json` is in explore mode. Switch to a hover for first flights:
  `{"route": [], "params": {"explore": false, "height_m": 1.0, "hover_s": 5.0}}`.
- Nothing has flown; the arm -> Position -> Offboard handover is untested. Props off first.
- The depth override has not been cleared with the organisers.
- The yaw offset between VIO and PX4 is unmeasured (organisers' guide §1).
- The mission does not plan with the map yet.
- Drone password was shared in chat: move to SSH keys (`ssh-copy-id`) and change it.

## 4. State of the drone right now

- The stack runs on the depth team image (`skybeta-team:0.3.2`), so the problem in §1 is live.
- No `fly.sh` session is running: all five windows were stopped one by one for the measurements and
  the empty session closed. `fly.sh start` works again.
- Last bag: `~/team_ws/bags/20261106-232603/`.
- Code everywhere (laptop, GitHub `jetson`, drone) is the same commit.
