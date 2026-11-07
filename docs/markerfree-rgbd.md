# Marker-free RGB-D PX4 experiment

This branch runs the Gazebo Harmonic course with a forward-facing rendered RGB-D camera and one downward range sensor. The camera is 384×256, 90° horizontal field of view, 20 Hz, mounted 0.34 m forward of the PX4 x500 body. The ToF emulation is one downward 1×1 range ray. Artificial markers are disabled. Each RGB-D capture first attempts point-to-plane depth registration against the supplied static CAD survey; if that is unobservable, it tries previous-frame depth registration, then RGB feature registration. The downward range assists surveyed depth matching. The resulting visual estimate is sent to PX4 EKF2. GPS, barometer, range-aiding, and optical-flow fusion are disabled in the SITL configuration. PX4 estimates, camera captures, and independent Gazebo truth/contact observations are logged separately.

The navigation-source audit checks the running world/model sources, absence of artificial landmarks, and zero publishers on `/model/course_quad/odometry_with_covariance`. It records `simulator_position_used_for_control: false`. `/world/dronehacks/dynamic_pose/info` remains available to the separate truth observer for scoring; that observer does not feed the estimator, controller, or policy.

## Start, view, and record

Start the dedicated marker-free PX4/Gazebo container from the repository root. This starts the simulator but does not arm or fly it:

```sh
./runtime/run-markerfree.sh
```

In a second host terminal, start the local viewer:

```sh
python3 -m web.server
```

Point the viewer's `current` link at the run directory, then open [http://127.0.0.1:8083](http://127.0.0.1:8083):

```sh
ln -sfn hover-02 artifacts/markerfree/current
```

To start a bounded evidence-recorded hover, open a shell in the dedicated simulator container and run:

```sh
docker --context colima-dronehacks-sim exec -it dronehacks-markerfree-sim-1 bash
cd /workspace
python3 -m mission.markerfree_evaluate \
  --output artifacts/markerfree/hover-02 -- \
  --task hover --launch-position -2.5146 0.9144 0.24 --launch-yaw-deg 90 --use-px4-tilt
```

Use a fresh output directory for each run. The evidence runner starts the camera recorder and independent truth/contact observer before it starts the mission. The recorder samples the RGB and depth streams at up to 5 Hz sim time, saves bounded JPEG RGB and uint16 PNG depth frame pairs with a timestamped `frames.jsonl` index, and writes latest camera/depth previews. The archive can be exported to a timestamped RGB/depth replay with `python3 -m tools.render_flight_video artifacts/markerfree/course-NEW` after the run. To run the full course after a completed hover at the same surveyed launch pose:

```sh
python3 -m mission.markerfree_evaluate \
  --output artifacts/markerfree/course-NEW -- \
  --task course --course mission/course-window-x500.json --heading-lookahead 1.8 \
  --hover-summary artifacts/markerfree/hover-02/telemetry.summary.json \
  --launch-position -2.5146 0.9144 0.24 --launch-yaw-deg 90 --use-px4-tilt
```

After a run, score its independent truth and contact logs offline:

```sh
python3 -m tools.score_markerfree artifacts/markerfree/course-NEW \
  --output artifacts/markerfree/course-NEW/score.json
```

The score is an offline estimate/error, route, proxy-clearance, contact-observation, and landing report. Contact sensors report observations; missing contact messages do not prove the aircraft was collision-free.

## Current evidence and limits

**Course-08 is the first completed marker-free full-route baseline.** The waypoint controller reached 15/15 waypoints; PX4 confirmed landing and disarm 83.000 simulated seconds after arming. Independent scoring used 12,950 truth samples and verified the route, the full modeled x500 envelope crossing the window once in the forward direction, and the modeled touchdown inside the pad. Minimum window aperture clearance was 0.106299 m with the score's 15 mm support inflation. Horizontal modeled-envelope clearance inside the 0.762 m-radius pad was 0.313969 m. The CAD-proxy path check certified takeoff, transit, and landing clear with the same 15 mm inflation.

The navigation audit passed for course-08: no artificial landmark models, zero publishers on `/model/course_quad/odometry_with_covariance`, and the recorder states simulator truth was not used for navigation. Truth from `/world/dronehacks/dynamic_pose/info` was consumed by the independent observer for scoring only. This was the waypoint **baseline**, with zero learned-policy actions. Policy-01 failed preflight with `invalid_policy_exchange_record:response.json`; elapsed flight time was zero and it did not arm. No learned-policy flight occurred, so its result remains unverified.

Contact coverage was partial. The observer received base-link contact messages and advertised rotor sensor publishers, but observed no rotor messages. The score records no unexpected contact observation; it does not claim zero collisions. Do not interpret contact-sensor silence as proof of no collision.

Earlier evidence remains relevant: course-02 lost trackability after arm and landed after fault 14.66 simulated seconds after arm; course-03 also failed its tracking observability gate and landed after abort at 55.148 simulated seconds; course-04 reached window alignment then landed after an observability fault. These attempts did not complete the route. Short-run estimate errors measured in ideal rendered scenes are not hardware accuracy or accumulated localization-drift claims.

The trainer is analytic teacher-supervised imitation with student rollouts (DAgger-style), using proxy course geometry. It is not a learned world model, Dreamer implementation, or policy trained from Gazebo flight data. The policy request/response service is implemented separately and can be started on the host CPU with a validation-selected checkpoint:

```sh
python3 -m learning.inference \
  --checkpoint artifacts/learning/RUN/frozen.pt \
  --exchange artifacts/markerfree/policy-exchange
```

A complete learned-policy PX4 flight has not yet been verified. Copy a selected checkpoint to a frozen file and point both the service and flight command at that copy. Replace `RUN` with its directory; use a model trained for the same route, mount, dynamics, and heading profile. Use `--profile x500-transfer` for training; the checkpoint records the route, heading, airframe, camera and control contract. The legacy sphere-model checkpoint is rejected by the flight runner. The flight runner requires the checkpoint hash explicitly and rejects mismatched camera geometry:

```sh
CHECKPOINT=artifacts/learning/RUN/frozen.pt
CHECKPOINT_SHA=$(shasum -a 256 "$CHECKPOINT" | awk '{print $1}')
python3 -m mission.markerfree_evaluate \
  --output artifacts/markerfree/policy-NEW -- \
  --task policy --policy-dir artifacts/markerfree/policy-exchange \
  --policy-sha256 "$CHECKPOINT_SHA" --policy-depth-mode raw --heading-lookahead 1.8 \
  --hover-summary artifacts/markerfree/hover-02/telemetry.summary.json \
  --launch-position -2.5146 0.9144 0.24 --launch-yaw-deg 90 --use-px4-tilt
```

The primary scorer derives five body/gear boxes and four swept propeller cylinders from the pinned x500 SDF, applies 15 mm support inflation, and accounts for recorded attitude. The old 0.36 m sphere remains a legacy diagnostic; the older training experiment also used a 0.28 m sphere and faster response, so those checkpoints have a known transfer mismatch. See [x500-envelope.md](x500-envelope.md) for model geometry and limits. These are simulation-model bounds, not verified physical Skybeta body/propeller measurements. The course window is 0.9144 m wide. The estimator uses a supplied static survey and has no general global relocalization or validated real-camera uncertainty model. Gazebo depth is easier than physical stereo. The separate SAND.O checkout is not integrated; its ROS 2, Gazebo Classic, PCL, DecompROS2, and Gurobi dependencies are not part of this run.

The [course-08 replay](media/window-baseline.mp4) and [window-crossing still](media/window-baseline.png) show simulated sensor captures. The compact score and source hashes are in [markerfree-results.json](markerfree-results.json). Physical-camera depth capture prerequisites and verified driver semantics are documented in the [OAK metric-depth handoff](oak-depth-handoff.md).

The optional `--use-px4-tilt` adapter brackets capture-time PX4 roll and pitch within 60 ms. It adds no position or yaw pseudo-measurement and keeps the depth observability gate. Because PX4 attitude can correlate with visual position through EKF state updates, it is treated as a conservative prior, not an independent camera observation. PX4's [EV fusion bitmask](https://docs.px4.io/v1.16/en/advanced_config/parameter_reference.html#EKF2_EV_CTRL) enables position and yaw here.

The SITL power watchdog directly reads battery and system-power uORB publications on a background thread. It requires advancing timestamps before arming and rejects stale or invalid feeds in flight. This was added after an earlier run exposed stale simulator power publications; resetting the dedicated container restored updates, but the underlying scheduler failure has not been isolated. No PX4 battery check was disabled.
