# CAD-free forward-camera prototype

Current scope: local prototype on saketh/autonomy, not flight-qualified. The
camera remains forward-facing. The cancelled 15deg mount is not a runtime
rotation or configuration default. Runtime geometry uses measured timestamped
base_link-to-optical TF, CameraInfo and full capture-time odometry interpolation.
The nominal zero-degree transform exists only in synthetic sensor fixtures.
The existing CAD-based nodes/route anchors remain regression fixtures; this
new navigator never imports Course, CAD geometry, simulator truth or waypoints.

## Shared pipeline and reuse

Organizer odometry, VO health, DepthAI depth and measured TF -> observed 3D
log-odds map -> legal/unknown/full-body clearance constraints -> bounded
frontier exploration / observed passage objectives / confirmed blue-pad goal
-> shared 3D A* -> existing clothoid (default), SFC or MPL kernels -> common
retiming, closed-loop follower, swept stopping-volume check and backend gate.

`team_mapping.py` preserves the team's main481cbf8 vectorized mapper. The new
wrapper expires measured free space, keeps unknown blocked and makes new
positive returns override old free confidence immediately. No-return/NaN/inf
does not clear a ray. Map memory is bounded (default16x16x4m, .2m voxels); the
map is not rolling. Outside its extent is blocked. CPU search is bounded but
Python planner latency is not certified real-time. A single ROS worker runs
planning; health callbacks and trajectory timing stay on the ROS thread.

The three native smoothers remain horizontal; common rest-to-rest 3D stages
handle altitude transitions with full-volume recertification. This is not a
new fully coupled 3D SFC/MPL implementation. There is no silent planner fallback.
The MPL lattice is scaled at confined .2m/s because its previous .225m/s minimum
nonzero velocity could not move at that cap. Confined paths use .2m/s limits.

## Course rules and passage objectives

Both tunnel and window traversal are required by the user's confirmed rule.
They are independently configurable, enabled by default and tracked by the
shared navigator. Seeing the finish cannot skip unfinished passage objectives.
A completed crossing needs an aligned approach, an observed-free centreline
and arrival on the opposite side. A pose correction discards map, pad track,
pending plans and passage progress; manual disarmed reset is required.

The actual legal numeric altitude range and odom-frame floor reference remain
unverified. `online.yaml` leaves both verification flags false and limits unset.
The online node therefore refuses motion. Supply reviewed *centre-height*
bounds that account for the aircraft envelope if organizer rules constrain the
whole aircraft. These must be below permitted wall tops; the organizer's
0.3–3m setpoint clamp is not a legal flight envelope. No climb escape exists.
Observed obstructions above the verified floor block planning above them;
only a positively bounded aperture can whitelist its local free volume.
The guard conservatively also blocks over-furniture shortcuts. Unobserved
course space/boundaries remain blocked. This is not a substitute for obtaining
the legal envelope before any physical qualification.

Passage classification searches observed vertical wall planes at 30deg normal
increments. Unknown boundary pixels cannot form a wall ring. A closed rectangular
hole is a window candidate; a roughly circular hole needs occupied-ring/free-core
evidence persisting along its axis to be labelled tunnel. Estimated centre,
normal, width/height, radius, length, entry and exit are shared by every planner.
The detector is limited to approximately horizontal passages and quantized
wall normals. Sparse, oblique, cluttered, damaged or sloping tubes may be missed.
Circular appearance alone does not authorize traversal. Unknown exits and
partial obstructions cannot be crossed; reachable frontiers may be explored
incrementally instead. This is prototype classification, not a reliable general
cylinder reconstruction system. The original CAD-derived tube/window route code
is preserved, not imported as hidden geometry into online navigation.

## Finish and landing

OpenCV HSV blue segmentation, orange surround, convex quadrilateral filtering,
metric square dimensions, depth coverage, plane residual and horizontal normal
checks reject mono imagery, posters, bad depth, partial boundary clipping and
comparable ambiguous squares. Capture-time depth-to-color registration uses
both cameras' intrinsics/extrinsics and z-buffering; matching pixel coordinates
or an RGB-labelled depth frame is not proof of alignment. When a floor reference
is verified, a candidate must agree with it within .1m. Three consistent views
confirm an odom-frame centre and four boundaries. Memory lasts at most2s and
expires earlier with localization uncertainty; long loss needs reconfirmation.

Crucial current input gap: the observed left/right bgr8 streams contain mono
pixels. No usable RGB stream or RGB calibration was observed. `/oak/rgb/*` is a
required interface placeholder, not a claim that it exists. Reuse/reconfigure
the existing OAK camera pipeline only after separate review; no driver changes
were made. Genuine color and verified depth geometry are needed for blue/orange
detection. Do not infer blue from grayscale.

Measured LEFT mono K at960x600: fx595.1951,fy595.3043,cx487.3531,cy305.8477.
Using it as a **test projection**, forward-facing visibility is approximately
77.71deg horizontal,27.19deg upward,26.22deg downward. Nearest visible floor is
2.031 times camera height in front of the camera (1.320m at .65m). Nadir is
outside the image. These are not actual RGB coverage measurements. Oblique
forward color/depth tests detect the square, but final visual contact is lost.
The approach model explicitly demonstrates a safe uncertainty hold before
alignment, not an accurate completed landing.

Current Sky Beta source/documentation has no verified autonomous landing
mechanism. The node can plan an approach at .65m above the measured pad and
evaluate boundary/uncertainty/descent-clearance readiness. It never sends a
landing or descent command. Missing fresh vision, uncertainty or clearance
holds. No autonomous landing is claimed. Existing odometry may support bounded
memory, but current tests do not establish final landing accuracy.

## ROS interfaces and physical boundary

`ros2 launch dh_autonomy online.launch.py planner:=clothoid` selects the
hard-locked `online_stationary` executable. It starts no camera, SLAM, estimator,
flight_interface or physical backend. `planner:=sfc|mpl` uses the same shell.
`online_mission` is a future normal entrypoint, also dry by default; explicit
physical mode supports Sky Beta only, never direct FMU. No such mode is deployed
or authorized in this task. Stationary isolation is checked even with false/PX4
parameter overrides. Never run alongside another dh_mission instance.

Inputs (sensor-data best effort QoS, configurable names):

| Input | Contract |
|---|---|
| `/visual_slam/tracking/odometry` | nav_msgs/Odometry; odom -> base_link, capture clock, finite covariance/unit quaternion |
| `/visual_slam/status` | Isaac VisualSlamStatus; fresh tracking state1 required; missing dependency holds |
| `/oak/stereo/image_raw`, camera_info | Image16UC1 millimetres or32FC1 metres; optical z; verified rectified frame/intrinsics/raster |
| `/oak/rgb/image_raw`, camera_info | genuine bgr8/rgb8; rectified calibrated color, paired within5ms; currently unverified |
| TF | measured base_link <- each optical frame at capture stamp; no nominal fallback |
| `/dh/start`, `/dh/reset_online` | Empty; reset in future physical mode requires fresh disarmed state |

Odometry is bracketed without future extrapolation (history4s,max gap.3s).
Depth freshness .5s; odometry .4s is unchanged. Full XYZ/yaw/quaternion jumps,
stale tracking or excessive uncertainty stop progression; prolonged loss aborts.
Attitude covariance above .2rad standard deviation rejects odometry. Zero
position covariance is not trusted as zero error (minimum .03m configured).
These conservative thresholds need validation, not relaxation to hide latency.

Outputs: `/dh/dry_run/cmd_vel` (TwistStamped), `/dh/online_status` (String JSON),
`/dh/online_plan` (Path), `/dh/landing_pad` (PoseWithCovarianceStamped),
`/dh/landing_pad_bounds` (PolygonStamped), optional PointCloud2 occupied/free
and JSON map metadata. Map axes are xyz/ENU/odom/metres; unknown is the remainder
of the advertised extent, not the absence of occupied points. Map publication
is disabled by default to bound bandwidth.

Future Sky Beta contract: fresh zero ENU twist before handover, movement only
after fresh confirmed Offboard, permanent takeover latch, no automatic resume,
no duplicate flight_interface, no arm/mode/FMU calls. Fake-bus tests exercise
this source behavior; they are not DDS, device or flight validation.

## Integration and remaining critical path

The strict full-body unknown-space guard currently holds from a fresh map with
the actual frontal camera model: its limited cone cannot certify the surrounding
starting/stopping volume. Do not replace that hold with invented free space.
A reviewed initialization/observation strategy using existing sensing is still
needed before this is a functional end-to-end physical-camera navigator. The
panoramic benchmark isolates algorithm behavior and is **not** the physical OAK.
Unknown-map benchmark finish observations are an ideal simulated landmark;
real HSV processing is tested separately. These tests cannot prove a complete
forward-camera tunnel+window+pad mission. A*+clothoid is the critical path;
alternatives remain available without separate perception systems.

Claude's Gazebo adapter must expose the above timestamps/TF/intrinsics, true
color and calibrated optical-depth geometry, odometry covariance/VO health,
explicit simulated rule/floor settings and the hard-locked dry command sink.
It must render the orange floor/blue square, finite-height walls, tunnel/window,
partial occlusions and changed wall angles. Any dry-command-to-SITL bridge must
be separately reviewed and simulation-only. No Gazebo files were modified here.

Latest inspected team main481cbf8 uses root dronehacks_nav; `fly-sim.sh --code`
exports the exact selected Git ref and builds that root package. It does not
automatically build these nested ros2_ws packages. A policy branch created from
main does not include autonomy history by itself. Review package integration,
explicit dependency/build/startup wiring and the simulation command sink before
claiming a policy test. `promote.sh` checks main ancestry/tests/exact simulation
evidence and merges main; never execute it or a drone push in this local task.

Frozen Phase2 candidate f85c492 remains separate, unchanged and awaiting approval.
No SSH, remote writes, Docker launch, main/promotion or physical operation is
part of this prototype. See the final validation report for actual counts,
failures, metrics and exact source commits; unit/SIL is not ROS or hardware.
