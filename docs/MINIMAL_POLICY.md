# Executable minimum local navigation policy

Implemented on saketh/autonomy. No CAD input, no takeoff/landing/return mission,
no direct PX4 commands, no new simulator. Later user instructions paused all
simulation runs and advanced feature work. The team's proven hardware bring-up
is accepted; the outstanding checks concern this new policy's integration.

## Run after separately authorized installation

The default launch is hard-locked to dry output:

```sh
ros2 launch dh_autonomy local_policy.launch.py params_file:=/absolute/path/local_policy.yaml
```

The installed entrypoint is `local_policy_stationary`; output is
`/dh/dry_run/cmd_vel`, never `/skybeta/*` or `/fmu/*`. `local_policy` is the
future normal entrypoint, also dry by default. No physical launch is authorized.
The launch starts only the policy. It starts no flight_interface, SLAM or camera.
Do not launch the team's mission.launch.py alongside an already running
flight_interface. This policy subclasses the existing MapNode; do not also run
a duplicate standalone map_node. No arbitrary remapping of dry command topics.

Configuration intentionally leaves `depth_geometry_verified:false` and
`below_wall_height_verified:false`, legal min/max unset. They must be backed by
actual calibration/rules, not changed merely to get movement. There is no
automatic takeoff: the aircraft must already be at the reviewed cruise height
under a separately authorized pilot/organizer procedure. `/dh/start` requests
local navigation; the trajectory clock still waits for fresh Offboard. Never
send this start request to hardware in the current task.

## Existing interfaces and exact integration gaps

Audited team main481cbf8: `dronehacks_map` is the ROS node name, implemented in
`dronehacks_nav/map_node.py`. Its `VoxelMap` in `mapping.py` stores3D log-odds
and explicit unknown/free/occupied data in xyz order, default20x20x4m at.1m.
Inputs are native depth, CameraInfo, odometry and base<-optical TF. It publishes
`/map/occupied` PointCloud2 and `/map/image` bgr8 for visualization, not a
planner-ready occupancy grid. Occupied-only cloud omits known-free information;
the image slice marks a column free when ANY voxel is free and flips the y axis.
Neither can be used directly as a full-aircraft planning map.

The minimum implementation reuses that **same mapper instance in process**
rather than creating another mapping pipeline or inferring free from its cloud.
`TimedTeamMap` calls its existing integrator and adds per-voxel capture ages plus
an immediate positive-hit veto. These address demonstrated missing freshness
and delayed dynamic-hit confidence; they do not replace its mapping algorithm.
`from_team_map` converts only freshly observed-free *entire height bands* into
the existing Grid2D, with correct +y rows (transpose, no display flip), aircraft
radius/height,10cm margin,3sigma position uncertainty and voxel discretization.
Unknown, old free, positive uncertainty and outside-map cells are blocked.

`planner_astar.ClearancePlanner` already accepts Grid2D independently of CAD.
The adapter validates actual endpoints before calling it to prevent its legacy
nearest-free/clipped-endpoint behavior from silently relocating the vehicle.
It then reuses `timing.fillet_path(transition='clothoid')`, `time_parameterize`
and `tracker.TrajectoryFollower`. No planner or tracker kernel is rewritten.
The legacy `route.plan_mission` and `mission_node` require Course/launch/pad
geometry at construction, so they are bypassed by this small executable shell.
The original legacy nodes/files remain intact.

Inputs consumed:

| Topic | Type / contract |
|---|---|
| `/oak/stereo/image_raw` | Image16UC1 millimetres or32FC1 metres, little-endian rectified optical depth |
| `/oak/stereo/camera_info` | matching CameraInfo, finite calibrated K, same raster/frame, rectified D |
| `/visual_slam/tracking/odometry` | Odometry odom->base_link, finite covariance/unit quaternion, same ROS capture clock |
| `/visual_slam/status` | Isaac VisualSlamStatus, monotonically fresh tracking state1 |
| `/tf`, `/tf_static` | measured base_link<-depth optical at capture time, verified original forward mount |
| `/skybeta/state` | existing backend's fresh state/authority contract; normal mode only |
| `/dh/start` | Empty; requests policy start, never arms or changes mode |

Depth/CameraInfo/VO use sensor-data QoS; the inherited team's odom subscription
uses reliable depth10. Verify publisher compatibility in real ROS; no local DDS
claim. Capture-time poses use the already implemented rigid history/Slerp,
without future extrapolation. Depth waits in the team's existing bounded queue.
Independent clearance reuses `refnav.depth.decode_depth/level_points/clearance`.
No second stereo matcher or sensor-fusion system is introduced. Default config
integrates10Hz, plans at most once/second, commands20Hz. Map/pose/TF health gates
apply before any usable output. Status is `/dh/local_policy/status` JSON.

## Selected behavior and safeguards

Choose a useful forward goal .6–1.5m away in observed reachable space; penalize
the last8 reached goals. Ask the existing A* for candidate reachability. This
is a short local policy, not full maze exploration. If no candidate is safe,
hold. If a new map invalidates the remaining path, zero the command and replan
on a later tick. Never drive an old trajectory indefinitely. A fixed reviewed
cruise height eliminates intentional climb-over-wall or emergency vault behavior.
The numeric legal below-wall height is unknown and blocks current defaults.

The team's existing independent `safety_filter.py` and SafetyGate are reused
via a provenance-preserving cherry-pick of6c3a0a9 (local0bf8559). The policy
sets conservative parameters: .3m/s max, .4m/s² reference/output slew cap,
.8s latency budget, .6m/s² assumed braking,10cm margin plus3sigma uncertainty.
At .3m/s, model stopping travel is .315m; adding default radius/margin/3sigma
requires at least .775m measured corridor distance, with map discretization
inflation in addition. Latency/braking/dimensions are **unmeasured policy
parameters**, not established real-airframe performance. Stronger hazard stops
override slew limiting. Every output is hard-clamped after the follower's P term,
and a map-certified full-body stopping sweep is checked independently.

Missing/NaN/low-coverage/stale depth, stale odometry, failed TF, old map voxels,
malformed control inputs or localization loss zero movement and freeze progress.
Depth age300ms and odometry400ms remain conservative; existing bag latency can
exceed these, which may hold this policy until integration/timing is resolved.
No timeout was relaxed to force motion. Pose jumps abort. Loss of authority
after Offboard permanently aborts; no automatic resume. Future normal mode
uses fresh zero twists for pre-Offboard readiness and only SkybetaBackend;
health abort requests the existing /skybeta/hold while authority remains active.
The stationary factory cannot construct physical publishers/clients even if
dry_run is overridden false. No arm/mode/FMU/duplicate flight_interface calls.

## Actual validation and unfinished work

Lightweight local checks:50 core/safety pytest cases and7 focused fake-ROS
component checks pass. Actual team MapNode source was exported from main481cbf8
with source hashes and imported unchanged for the latter. Those checks cover
startup, native-depth/TF integration, malformed depth rejection, unknown-height
and legal-height rejection, bounded commands, new-obstacle veto, stale/missing
inputs, hard-locked dry isolation, and the actual new node's pre-Offboard/
clock/takeover contract. Fake ROS is not DDS or hardware execution.

An earlier development sensor/grader run initially exposed a zero-reference
startup deadlock; the adapter now evaluates the configured speed ceiling before
the follower and the targeted bounded-command regression passes. Its next run
generated3.26m of commands,3 replans and0 graded collisions, but FAILED the
required post-insertion stop assertion (the inserted fixture obstacle was not
guaranteed to intersect the active route). It is retained as a **failed,
nonqualifying development scenario**, not reported as obstacle-simulation success.
The user then stopped all simulations; no further scenario/Gazebo/fly-sim run
was performed. Targeted software obstacle-mask vetoes pass independently.

The optional earlier prototype atafd8b80 remains preserved: forward-camera
HSV blue/orange detection, bounded pad memory and aperture classification.
These advanced features, SFC/MPL optimization and broad benchmarking are paused.
The minimal executable can plan through same-height openings when the full
volume is observed and large enough, but it has no validated semantic tunnel/
window traversal or integrated finish behavior. It neither lands nor declares
maze completion. The team main contains its own downward-twist landing code;
that is not proof of organizer approval or policy-specific landing safety.

Hard blockers to recommending supervised flight: actual ROS/Humble build/DDS
tests of this node; corrected frozen Phase2 execution; actual depth frame/
rectification and capture-time TF confirmation; reviewed legal numeric altitude
and propeller envelope; measured latency/braking; and a safe initialization
observation sufficient for the full-body volume. A fresh frontal camera map
may correctly hold because nearby surrounding space is unobserved. Do not seed
it free from CAD or silently waive the guard. This is a policy-integration gap,
not a request to repeat the organizer's already successful basic bring-up.

Estimated focused work remaining (not an authorization or guarantee): package/
policy-branch review30–60min; real ROS import/QoS/launch and safety execution
30–60min once an approved ROS environment is available; measured rule/calibration/
timing and safe-start acceptance depend on team evidence, with no honest fixed
flight date. The runnable local code exists; these gates must not be bypassed.

## Team policy workflow (prepared, not executed)

`fly-sim.sh` was inspected from origin/sim. It is a macOS/Colima/Docker launcher,
archives the exact --code ref, requires root dronehacks_nav, builds that package,
starts its mission plus a standalone map node, and records exact commit/result.
It cannot consume the nested autonomy package automatically. No compatible
current mode proves this local policy: mode1 hover,2 blind box,3 predefined box
with depth,4 explicit wall vault (forbidden here),5 surveyed route,6 record-only.

The accompanying policy integration patch is against main481cbf8, selects this
policy's entrypoint and packages the selected existing Python modules. Keep
mode3 only as a runner preset; its generated waypoints are ignored by this
policy and must not be described as a CAD-free policy result by themselves.
The simulator runner must skip its additional map_node when this policy is
selected and wire /dh/start plus a reviewed simulation-only command sink/config.
All verification flags remain false/dry by default in the prepared package.
Current user instructions explicitly prohibit running fly-sim/Gazebo now.
Never run promote.sh or push main/drone. No policy branch/main worktree was changed.

## Frozen Phase2 remains separate

Corrected source f85c49218f971ff52c53219a1960cd4887cfc3d4 is already published.
The retest ZIP/archive hashes remain frozen, without this policy/dependency.
Missing tests_require=[pytest] explained Colcon's unittest selection; actual
Colcon selector and local stub-only pytest collection verified the fix. Required
collection, real pytest command, no-skip JUnit and executed-isolation gates remain.
The previous hardware run still failed its zero-test gate; no new hardware pass
is claimed. No SSH, Jetson writes or physical operations in this development task.
