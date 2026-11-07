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

Use a fresh output directory for each run. The evidence runner starts the camera recorder and independent truth/contact observer before it starts the mission. The recorder samples the RGB and depth streams at up to 5 Hz sim time, saves bounded JPEG RGB and uint16 PNG depth frame pairs with a timestamped `frames.jsonl` index, and writes latest camera/depth previews. It does not encode a video. To run the full course after a completed hover at the same surveyed launch pose:

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

`artifacts/markerfree/hover-01/telemetry.summary.json` records one complete marker-free hover: 17.0 simulated seconds, takeoff/hover/landing completed, disarmed, 441 visual measurements, and no simulator pose used for control. The subsequent `hover-02` also completed with the optional inertial tilt adapter enabled: 17.996 simulated seconds through landing/disarm. Independent scoring certified the modeled path clear and found 0.352 m of conservative landing-pad clearance. Neither hover proves course navigation.

The recorded `course-02` run armed, then lost trackability in the survey/depth and RGB fallback and landed after the fault 14.66 simulated seconds after arm. It did not complete the route. Its post-run ideal-rendered-scene comparison reports 2.36 mm RGB-D estimate RMSE and 1.88 cm PX4 position RMSE over the matched samples; these numbers describe that short run and the supplied Gazebo scene, not hardware accuracy or accumulated localization drift. Check the per-run summary and score before quoting results. `course-03` also failed at the tracking observability gate. Its telemetry records PX4 touchdown and disarm after abort at 55.148 sim seconds; it did not complete the route. Its score is marked failed, with no valid truth landing position. These two course runs are failures, not full-course demonstrations.

The trainer is analytic teacher-supervised imitation with student rollouts (DAgger-style), using proxy course geometry. It is not a learned world model, Dreamer implementation, or policy trained from Gazebo flight data. The policy request/response service is implemented separately and can be started on the host CPU with a validation-selected checkpoint:

```sh
python3 -m learning.inference \
  --checkpoint artifacts/learning/RUN/best.pt \
  --exchange artifacts/markerfree/policy-exchange
```

A complete learned-policy PX4 flight has not yet been verified. Copy a selected checkpoint to a frozen file and point both the service and flight command at that copy. Replace `RUN` with its directory; use a model trained for the same route, mount, dynamics, and heading profile. To train or warm-start for this camera mounting position, use `--camera-forward 0.34`; a checkpoint records its camera offset and source hash. The flight runner requires the checkpoint hash explicitly and rejects mismatched camera geometry:

```sh
CHECKPOINT=artifacts/learning/RUN/best.pt
CHECKPOINT_SHA=$(shasum -a 256 "$CHECKPOINT" | awk '{print $1}')
python3 -m mission.markerfree_evaluate \
  --output artifacts/markerfree/policy-NEW -- \
  --task policy --policy-dir artifacts/markerfree/policy-exchange \
  --policy-sha256 "$CHECKPOINT_SHA" \
  --hover-summary artifacts/markerfree/hover-02/telemetry.summary.json \
  --launch-position -2.5146 0.9144 0.24 --launch-yaw-deg 90 --use-px4-tilt
```

The primary scorer now derives five body/gear boxes and four swept propeller cylinders from the pinned x500 SDF, adds a 15 mm allowance, and accounts for recorded attitude. The old 0.36 m sphere remains a legacy diagnostic; it overstated vertical size while understating diagonal propeller reach. The older training experiment used a 0.28 m sphere and faster response, so its checkpoint has a known transfer mismatch. The new route centers the maze corridors and raises the window crossing to 0.90 m; nominal offline dynamics checks passed, while its PX4 course test is still in progress. See `x500-envelope.md` for the geometry and limitations. The course window is 0.9144 m wide. The marker-free estimator uses a supplied static survey and has no general global relocalization or validated real-camera uncertainty model. Gazebo's ideal depth is easier than physical stereo. The separate SAND.O checkout is not integrated; its ROS 2, Gazebo Classic, PCL, DecompROS2, and Gurobi dependencies are not part of this run.

The optional `--use-px4-tilt` adapter brackets capture-time PX4 roll and pitch within 60 ms. It adds no position or yaw pseudo-measurement and keeps the depth observability gate. Because PX4 attitude can correlate with visual position through EKF state updates, it is treated as a conservative prior, not an independent camera observation. PX4's [EV fusion bitmask](https://docs.px4.io/v1.16/en/advanced_config/parameter_reference.html#EKF2_EV_CTRL) enables position and yaw here.
