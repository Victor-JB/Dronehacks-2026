# Forward-camera continuous physics demonstration

This is local development evidence, **not flight qualification**. Results and
media are generated under `outputs/continuous_demo` outside Git. Earlier
`outputs/judge_demo` evidence remains intact. Consult the generated REPORT.md
for actual outcomes; do not infer maze completion from the presence of a video.

## Startup correction and limits

The previous gate coupled a 300ms live-depth deadline to a 500ms lifetime for
every free voxel. A roughly 19s forward-camera yaw scan could never accumulate
enough evidence. Moreover, the team mapper's downsampled rays leave unobserved
sampling holes, and the camera has genuine vertical/near-field blind spots.

The existing team log-odds VoxelMap remains the mapper. The planning adapter
now optionally retains **observed** free cells with capture timestamps,
exponential confidence decay, an explicit confidence threshold and a hard
age cap (maximum configurable 60s). Fresh depth300ms/odom400ms/TF/map-update500ms
checks remain independent. Stale free evidence becomes blocked unknown;
positive occupied evidence stays blocked. This conservative occupancy policy
can retain a removed obstacle longer than necessary; it never frees one on age.

Optional dense carving projects whole voxel bounding spheres into the
capture-time calibrated depth image. A complete pixel rectangle must be valid
and its nearest surface farther than the entire voxel plus error bound. It
never interpolates invalid pixels or clears a positive hit. It fills sampling
holes, not camera blind spots. It is disabled in physical/default profiles.

An explicitly verified launch-clearance cylinder is a separate expiring
certificate, not log-odds evidence. Only whole voxels inside it qualify;
positive measurements veto it. Visualizations distinguish it in purple.
No route is supplied from the course geometry. The initial scan and subsequent
route-heading observations command zero translation and require fresh sensors,
healthy localization, valid TF and a safe full inflated rotation envelope.
Pilot takeover permanently aborts progression, including during scans.

Reference-time advancement is scaled with a reduced safe stopping-sweep speed;
unknown stopping sweeps still veto motion. A safety-held reference is discarded
after1s, followed by a guarded quarter-turn observation and replanning with the
existing A*/clothoid planner. No route is followed outside observed free space.

Active perception may select an already-observed reachable target outside the
current camera heading, then rotate before translating. Heading uses a geometric
250mm lookahead; a time-only lookahead near rest can point at a tiny cell stub.
The adapter removes A*'s redundant starting-cell-centre waypoint before adding
the measured start, avoiding a backwards sub-cell leg that the live FOV gate
could never authorize. The complete resulting segment still passes the dense
collision/unknown-space check. Default passive selection remains forward-only.

The depth corridor now solves the swept cylinder/point contact distance instead
of subtracting the complete radius at every lateral edge of a rectangle. This
preserves the full circular envelope and the independent SafetyEvaluator's
existing radius/margin/braking subtraction. Regression tests verify contact
geometry on both sides of the boundary and immediate centreline collisions.

## Exact simulation assumptions

- MuJoCo4.0, 5ms rigid-body steps, gravity, mass1.222kg and team inertia/rotor
  geometry. Force/torque velocity servo, **not PX4**. The initial Euler/body-rate
  controller mixed torque frames; failed attempt02 is retained. Corrected
  torques and angular velocities share the world frame.
- Original STEP meshes for visuals/depth; convex collisions with CoACD tube/
  window decomposition. Approximate collision decomposition is not CAD-exact
  certification. Altered variant rotates two walls by±5deg.
- Cylinder radius.30m/half-height.12m contains the model's complete prop envelope.
  Horizontal margin.10m, position3sigma.015m plus voxel discretization.
- Vertical margin.02m is **simulation only**, justified by bounded depth error
  ±9mm, exact camera extrinsics/attitude and bounded position error±15mm.
  Physical/default vertical margin remains.10m. The tighter simulated bound
  must not be copied to hardware. Drone dimensions are not reduced.
- Verified placement centre is original mark +.05m east,+.15m north atz.85m.
  Initial certificate radius.53m/half-height.27m, expires30s. Physics checks
  radius.552m/half-height.285m to include bounded per-axis position error against every static collision
  geom before granting it. This explicitly verified clearance is limited to
  the initial launch region; no adjacent route is pre-certified.
  Early runs checked radius.545m; the later independent placement recheck
  correctly accounts for sqrt(2)*15mm XY error and15mm Z error and still passes
  with4.4mm model clearance. This is not a physical placement tolerance claim.
- 320x200 calibrated forward pinhole, vertical FOV53.4deg, team camera offset
  [.103,-.0375,.043]m. These are simulated calibration, not a live CameraInfo
  measurement. Rendered z-depth, not disparity-based stereo reconstruction.
- Depth10Hz, odometry/control20Hz, depth delay50ms, command delay150ms, velocity
  servo time constant200ms, acceleration limit.8m/s². Policy speed.3m/s,
  acceleration.4m/s², latency budget.8s, braking.6m/s² remain conservative.
- Depth noise sigma3mm clipped±9mm,1%invalid pixels, valid range.35–6m; carving
  range2.5m, existing rays4m. No-return pixels never certify free space.
- Odometry uses physics pose plus sigma5mm noise clipped±15mm, exact orientation
  and healthy synthetic VO. **No SLAM reconstruction, drift or real DDS**.
- Static-map age30s, confidence time constant30s, minimum free log-odds.20.
  The full vertical body band must be certified before XY inflation.
- Height.85m and simulated centre band.35–1.3m, below course walls. These are
  labelled simulation limits, not established competition/hardware permissions.
- RGB/depth registration is exact in this renderer. Existing HSV/depth pad
  detector and bounded PadTrack receive only matching captured images/TF.
  A confident detection biases local observed-free goals; no descent/landing
  command exists. Full-maze tunnel/window scoring is not thereby demonstrated.

## Reproduction

Use isolated Windows Python3.12 and `tools/judge_demo/requirements-continuous.txt`.
Set `DH_TEAM_SOURCE` to an exported team source containing `dronehacks_nav` and
`refnav`; existing fake ROS installs only an in-process message boundary. The
exported main5d416b5 mapper/depth sources are unchanged in inspected main11280a8.
Latest main PICKUP.md still records a real VIO/depth latency concern; it must
remain a separate hardware blocker, not be concealed by these simulated rates.

```powershell
$env:DH_TEAM_SOURCE='C:\path\to\verified-team-source'
$env:OPENBLAS_NUM_THREADS='1'
$env:OMP_NUM_THREADS='1'
python tools/judge_demo/physics_scene.py --assets C:\path\to\course --out C:\path\to\scene --variant baseline
python tools/judge_demo/continuous_run.py --scene C:\path\to\scene\baseline.xml --out C:\path\to\NEW-run --seconds 180 --record
```

Use `--variant altered` for changed walls and `--yaw 1.5707963` for a north
start. Runs stop on collision, bounded timeout or25s without meaningful progress
after45s. Initial-place rejection is separate. Full uncut frames are rendered
during physics execution at10fps, with live RGB/depth/map. Sensor delay remains
visible; third-person view is labelled simulator truth with boundary-net cutaway.
It is not a static image sequence or scripted path replay.

## Physical status

NO-GO for flight. Dry launch remains permanently hard-locked; physical profile
is untouched and disabled. Startup geometry/launch clearance, dimensions,
altitude permissions, error bounds, RGB registration and timing/braking need
physical verification. Actual Humble/DDS and frozen Phase2 retest still require
their separate gates. No SSH, deployment, physical command, main/drone push or
teammate Gazebo worktree change was made. No autonomous landing authority exists.
