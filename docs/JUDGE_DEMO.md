# Recorded judge demonstration — actual results, explicit limits

The user's autonomous-execution request supersedes the prior simulation pause.
All execution/rendering is local Windows software. No physical-flight settings
or autonomy source under ros2_ws changed from738fdc3. No Jetson access, main push,
drone remote, promote.sh or teammate/Gazebo worktree modification.

## Delivered media

`outputs/judge_demo/main_demo.mp4`:30s,1920x1080,H.264/MP4,20fps.
`outputs/judge_demo/highlight.mp4`:14s,same format.
Seven1920x1080 screenshots in `screenshots/`. Full videos decode without errors;
first/middle/last frames and screenshot dimensions verified. These are labelled
Blender cutaway REPLAYS of recorded execution data, not Gazebo screen captures.
The software generates actual plans/commands before visualization; animation
never supplies waypoints or navigation decisions. No audio or voiceover added.

## Actual pass/fail results

| Recording | Actual result | Interpretation |
|---|---|---|
| Provided FINAL Course v3 STEP + forward depth | BLOCKED,0m,0replans; startup refuses; depth-loss injection also holds | Safety gate works; course navigation FAILS to start. No full-course success claim. |
| Existing ideal panoramic local-policy diagnostic |2.644087864m,5replans,obstacle stop observed,0graded contacts; sampled horizontal hull clearance>=.3942m | PASS for this component fixture only. Not equivalent to forward OAK-D or full-course flight. |

Neither mission completes. Neither recording validates tunnel/window traversal,
blue-square recognition, landing or physical flight readiness. The highlight
explicitly says it uses ideal panoramic coverage. Do not remove those labels
or present it as a successful physical-camera course run.

The course recording executes the unchanged StationaryLocalPolicyNode, including
native Image/CameraInfo/odometry/VO callbacks, team mapper, calibration/history,
startup guard and dry backend. On this Windows host those ROS message interfaces
are emulated by the EXISTING tools/sil/fake_ros adapter. Actual ROS DDS/Humble,
Gazebo and PX4 are unavailable and were NOT run. This is a limitation against
the requested full-stack demonstration, not an equivalent replacement claim.

The diagnostic executes actual LocalPolicy -> existing team VoxelMap -> existing
ClearancePlanner A* -> clothoid/time parameterization -> TrajectoryFollower ->
independent SafetyGate -> DryRunBackend with the existing sensor/grader fixture.
It starts with a zero/unknown map, receives only sensor rays and odometry, and
has no access to the world's box list. The sensor producer/grader alone sees
geometry. The inserted obstacle is placed on the actual current reference.
The plant is kinematic Euler at.1s with no command delay/PX4 dynamics. Braking
performance and localization error are not established by this fixture.

## Course, airframe, sensing and visuals

The supplied513344-byte STEP was converted with Gmsh4.15.2/OpenCASCADE, setting
Geometry.OCCTargetUnit=M before import. The result has31 labelled parts,
6.096m-square floor, angled walls, the window, oval tube and finish/launch
markings. SHA256 and triangle metadata are in assets/course/course.json.
Conversion method: [official Gmsh documentation](https://gmsh.info/doc/texinfo/).
The STEP/meshes are used only by the sensor producer, collision grader and
renderer. They are never given to the autonomous planner or mapper as a map.
Source geometry remains unmodified. No CAD-derived route or goal enters policy.

Sky Beta visual geometry is reused from the team's origin/sim702104c SDF, with
actual model boxes, rotor positions and7in prop discs. It is visual geometry,
not a validated rigid-body dynamics model. Both recordings start at an assumed
airborne cruise pose; autonomous takeoff is NOT demonstrated. The sensor uses
the observed mono K scaled to320x200 as a projection model, zero-degree test
mount and an ideal BVH depth image. It is NOT a calibrated on-device depth or
SGBM/noise model. Static course pose permits reuse of an identical BVH capture
with fresh synthetic frame stamps; depth is then deliberately removed.

Course colors are presentation materials: orange floor and a blue square
overlay on the original CAD finish markings. No blue-pad detection is implied.
Nets/ceiling are hidden in the labelled cutaway camera but remain in the course
depth geometry. Replays use recorded poses, paths and discovered voxel data;
the map expires old free evidence just as the policy does. The rendered safety
ring shows radius+margin+3sigma; additional voxel inflation is not drawn.

The course's sampled cylinder-hull clearance is.1364m. A conservative enclosing
sphere certifies .1109m positive clearance at the stationary pose. This is not
dynamic collision evidence. The diagnostic uses its original cylinder/AABB
grader on every sample. Clearances from the two graders are different metrics
and must not be treated as interchangeable exact swept-body distances.

## Blocker and debugging

Actual forward sensing leaves the current full-body volume unobserved. The
source policy therefore refuses start. The500ms free-evidence lifetime and
near-field blind regions remain; geometry at the launch area is also close
relative to configured margins. No free bubble, panoramic data, scripted flight
or relaxed clearance was inserted into the forward-camera course case. A360
yaw scan alone would not prove all vertical/near-body clearance. The simulation
exposes an unresolved startup/observability gap, rather than concealing it.

The teammate Gazebo launcher was inspected/read only. It requires the unavailable
Linux/Colima/Docker/ROS stack. Blender4.3.2 was available, so existing local SIL
execution was recorded and rendered. No new flight simulator or SLAM was built.
Initial dependency selection tried PyBullet, but no cp312 Windows wheel was
available; native build was cancelled and PyBullet is not used/installed.
The first bulk trimesh ray attempt consumed about8.77GB and was interrupted.
It was replaced by Blender's bounded BVH ray casting, which produced64000
forward pixels without the bulk allocation. Both failures are documented;
failed attempts are not counted as completed runs.

## Evidence and verification

`evidence/course/` and `evidence/diagnostic/`: per-frame NPZ grids, occupied
points, trajectory, pose, sensor PNG, trace.json and result.json.
`evidence/path_and_commands.csv`: actual poses and commands.
`VERIFICATION.json`: full MP4 decode, screenshot hashes and metric reconciliation.
`MEDIA_MANIFEST.json`: frame counts/resolution and replay provenance.
`SOURCE_MANIFEST.json`: Git/source/asset hashes. `SHA256SUMS`: deliverable hashes.
CPU/wall times are local process measurements including recording I/O, not
Jetson real-time or flight benchmarks. Source and results label absent ROS,
PX4, Gazebo and hardware execution. Frozen Phase2 artifacts remain unchanged.

## Reproduce (local-only Windows)

Use Python3.12, Blender4.3.2 and a separate virtual environment. Install the
pinned tools/judge_demo/requirements.txt using prebuilt wheels. Set DH_TEAM_SOURCE
to the git-exported unmodified team fixture included under repro/team_source.
From the autonomy repository root, using an owned output path:

```powershell
$env:DH_TEAM_SOURCE = '<judge_demo>/repro/team_source'
python tools/judge_demo/convert_course.py --step '<FINAL Course v3.step>' --out '<judge_demo>/assets/course'
& '<blender.exe>' --background --factory-startup --python tools/judge_demo/capture_depth.py -- --assets '<judge_demo>/assets/course'
python tools/judge_demo/record_execution.py --assets '<judge_demo>/assets/course' --out '<judge_demo>/evidence'
& '<blender.exe>' --background --factory-startup --python tools/judge_demo/render_replay.py -- --demo '<judge_demo>' --scene course
& '<blender.exe>' --background --factory-startup --python tools/judge_demo/render_replay.py -- --demo '<judge_demo>' --scene diagnostic --stride 2
python tools/judge_demo/export_media.py --demo '<judge_demo>'
python tools/judge_demo/verify_delivery.py --demo '<judge_demo>'
```

The renderer is downstream of execution. Replacing the visuals cannot change
the evidence. No script starts ROS/PX4, sends physical commands, SSHs, deploys
or runs promotion. The existing actual ROS/physical qualification gates remain.

## Judge-friendly paragraph

We implemented a depth-driven navigation stack that builds an observed map,
selects local free-space targets, plans with A* and clothoid smoothing, and tracks
the resulting trajectory through an independent safety filter. In the recorded
ideal-coverage diagnostic, the actual code travels2.64m, replans five times and
stops for a newly detected obstacle with zero graded contacts. The supplied-course
forward-camera test also shows the current limit honestly: it holds rather than
entering unobserved space, so full-course navigation and physical flight readiness
are not yet validated.
