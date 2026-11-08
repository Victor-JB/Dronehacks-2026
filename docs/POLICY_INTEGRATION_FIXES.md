# Integration fixes after e42e5a3 — stationary verification candidate

All work is local on saketh/autonomy. No Jetson connection, deployment, physical
publisher, Gazebo run, new planner or mapping framework. The existing working
mission, its entrypoint, local_policy.launch.py and local_policy.yaml are unchanged.
This report supersedes the earlier failed-scenario/startup status in MINIMAL_POLICY.md.

## Corrections and actual evidence

The old obstacle fixture inserted a box0.9m EAST of the vehicle, irrespective of
its route. At insertion the vehicle was at[-2.8491,.5535,1], yaw1.4476rad, and
travelling north toward[-2.625,1.875,1]. The box was outside that route, so the
required post-insertion stop was never exercised. The corrected focused test
places the obstacle on the active trajectory, asserts that intersection and
asserts the drone is not already colliding at insertion. The original movement,
stop and zero-collision assertions remain. The single regression passes:
2.644m command-path length,5 replans, obstacle stop observed,0 graded collisions.
This reuses the existing ideal sensor/grader fixture; it is not Gazebo, actual
forward-camera acceptance, PX4 or hardware evidence. No additional layouts built.

A real integration defect was corrected: cached depth points were in the capture
heading frame but clearance was queried using the current heading. They now
transform capture-level FRD -> odom ENU -> current-level FRD, accounting for
translation. FOV is checked against the capture heading. A2m-east obstacle,
.5m-east vehicle displacement and90deg yaw correctly becomes1.5m to the right.
Trajectory and commanded velocity remain ENU world axes; no body-frame rotation
is applied at the backend. Tests include a northward world command at90deg yaw.
The inspected organizer interface converts world ENU velocity to NED by swapping
x/y and negating z; yaw rate is negated. Reference source7efe7e4 and prior0.3.3
image inspection support this contract; this policy has not executed in that image.

Physical command construction now requires an explicit reviewed-configuration
flag. Before publishing, normal mode requires exactly one flight_interface and
exclusive ownership of both Sky Beta setpoint topics. A second mission publisher
prevents readiness; losing ownership after handover aborts. The independent
safety gates and pilot-takeover latch remain. The permanently dry factory ignores
false dry_run overrides and creates no physical command publishers/clients.

Local verification:50 core/safety pytest cases,11 focused fake-ROS component
checks and1 corrected existing obstacle regression pass. Python3.10 grammar,
launch/profile static checks and shell syntax pass. Actual ROS2 Humble is NOT
available: no ros2/docker executable and WSL reports no installed distribution.
Fake ROS/collection stubs are not ROS startup or DDS evidence.

## Exact stationary command — after separate installation/operation approval

In the already installed/sourced organizer Humble environment:

```sh
ros2 launch dh_autonomy local_policy.launch.py
```

It consumes the existing /oak/stereo/image_raw and camera_info,
/visual_slam/tracking/odometry, /visual_slam/status and measured TF. It starts
no flight_interface, camera, estimator or legacy mission. It reuses the team
MapNode in process, so do not run a second standalone map_node for this test.
Output is only /dh/dry_run/cmd_vel and map/status products. No /dh/start request
is needed to inspect startup coverage. Observe:

```sh
ros2 topic echo /dh/local_policy/status
```

Default unverified geometry/legal flags intentionally HOLD. First verify the
actual image units, frame, calibration and rules using existing evidence, then
use a reviewed copy of the parameter file; never set verification flags merely
to get READY. Confirm graph publishers/clients have no /skybeta or /fmu endpoints.
The live-test domain must match the existing sensors; domain78 is ONLY for the
isolated synthetic DDS test below. Do not publish fake odometry into the live domain.

## Startup HOLD: diagnosed, not bypassed

The policy now refuses /dh/start unless fresh health, verified geometry/rules,
cruise height and the full inflated starting volume are valid. Status reports
WAIT_MEASURED_FULL_START_VOLUME rather than silently accepting start. It does
not clear unknown cells, seed a free bubble, move blindly, or increase expiry.

A static forward camera cannot see its own nearby surrounding volume. A360deg
yaw scan adds azimuth coverage but leaves close vertical blind regions; the
current500ms free-voxel lifetime also expires early views before a normal turn
finishes. Therefore a360 scan is a proposed supervised startup observation,
NOT an implemented/validated automatic escape from HOLD. A snaking corridor
reduces exploration complexity but does not resolve this sensing blind region.

Safe evidence procedure for a later authorized props-off stationary test:
verify disarmed state and dimensions, collect measured depth/TF/odometry while
hand-carrying through the intended starting area, and inspect current startup
readiness at the reviewed cruise height. Only actual freshly observed free
voxels count. If coverage cannot satisfy the500ms window, record NO-GO; do not
extend freshness, fabricate free space or attempt autonomous takeoff/yaw to fix
it. An approved observation-retention/initial-volume method still needs validation
before flight. This is the principal unresolved operational blocker.

## Installation, Humble and Phase2 gates

dh_autonomy explicitly depends on the installed team dronehacks_nav package
and sensor_msgs_py. The new local_policy ROS launch test exercises installed
entrypoint startup, native16UC1 depth, calibration, capture-time TF, truthful
unknown-start HOLD and actual DDS endpoint isolation even with dry_run false.
It is prepared, NOT executed. In a separately approved isolated Humble environment:

```sh
export ROS_DOMAIN_ID=78 ROS_LOCALHOST_ONLY=1
bash tools/hil/verify_local_policy_ros.sh /absolute/new/evidence-directory
```

It checks dependencies, builds both packages, proves collection, forces pytest,
checks actual runner command/JUnit and rejects missing/skipped/failed required
case evidence. It installs no dependencies and does not SSH. Missing Isaac
tracking-status package is a blocker; it is not replaced by fabricated health.

The separate frozen Phase2 candidate remains f85c492 and unchanged: ZIP SHA256
26ae7537e1021f643ad73406967ea319301911ce13fd0e0851d71b7a27700025.
Its two original required integration cases and stationary checker must execute
under that candidate's existing fail-closed controls. They have not been rerun
on the Jetson. Passing the new test cannot replace those gates, or vice versa.

## Values and first-flight GO/NO-GO

| Item | Existing evidence / required action |
|---|---|
| Native depth | Observed320x20016UC1 mm and CameraInfo; RGB-labelled header versus unaligned-depth configuration remains unresolved. Verify correct optical frame/rectification; never relabel without proof. |
| Camera mounting | Original forward mount, factory TF and7.53cm stereo baseline observed. Use timestamped TF, not ideal zero-degree substitution. Mono K is NOT native-depth/RGB calibration. |
| ROS QoS | Historical sensor/odom publishers reliable/volatile; sensor best-effort consumers compatible, inherited odom reliable10 matches. New node DDS execution still pending. |
| Drone envelope | radius.27m,half-height.12m are placeholders. Measure entire propeller/gear/camera envelope about base_link, including offsets. |
| Clearance | margin.10m plus3sigma positional uncertainty plus voxel discretization. Validate depth/calibration error and actual dimensions. |
| Altitude | legal min/max remain UNSET0/0; cruise.85m is a proposal relative to odom origin. Organizer0.3–3m clamp is not competition permission. Confirm below-wall height/floor reference/body allowance. |
| Timing | depth300ms,odom400ms,map/free500ms guards retained. Existing VIO delay can exceed them. Measure capture-to-actuation worst-case latency; do not relax guards to hide delay. |
| Braking | .6m/s² minimum assumption and .8s latency budget now configurable. Must establish conservative real bounds. At.3m/s stopping travel=.315m; default radius/margin/3sigma add.46m, before voxel inflation. |
| Startup | Complete fresh inflated body volume and safe short observed route required; static forward sensing has not demonstrated this. NO-GO while HOLD persists. |
| Authority | one existing flight_interface, no competing mission/setpoint publishers; fresh zero pre-Offboard twist, progression only after handover, immediate permanent takeover abort. Source/fake tests pass; actual ROS/new-policy test pending. |

GO for flight requires ALL rows resolved plus actual ROS tests/isolation,
obstacle-stop acceptance on the representative team environment, reviewed
configuration and separate organizer/team authorization. Current result is
NO-GO for flight; candidate ready only for reviewed stationary diagnostics.

## Separate physical selection and team workflow

local_policy_physical.launch.py and local_policy.physical-review.yaml are
review templates, not activated. Launch defaults reject activation before
constructing nodes; profile also has physical_configuration_reviewed:false.
By default it reuses the running organizer flight_interface. Optional startup
of that interface includes the organizer competitor launch with competitor:false;
only its owner may select it after checking no existing instance. Legacy mission
must be stopped deliberately before selecting this policy; no landing/takeoff
mechanism is invented. The dry launch remains byte-for-byte unchanged.

For eventual policy integration, create policy/<name> from latest main in a
clean isolated checkout, apply the NEW additive package patch, and commit it.
Keep `mission = dronehacks_nav.skybeta_node:main` untouched; add local_policy
entrypoints and select the new launch explicitly. The older review ZIP's
mission-replacement patch is superseded and must not be used. fly-sim's default
session still selects the legacy mission: the teammate must deliberately wire
the local_policy executable/profile and suppress its duplicate standalone map
node for that selection. A branch name/mode3 alone does not select this policy.
Do not execute fly-sim or promote in this task. Promotion requires later exact
policy-commit simulator evidence and team review; never push main/drone here.
