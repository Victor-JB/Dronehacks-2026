# Autonomy status — Codex takeover

Branch: `saketh/autonomy`. Local development is complete, checkpointed in `893dbfd` plus the final documentation commit. Ready only for the stationary verification boundary. No push or hardware access has occurred.

## Git and team ownership

Claude's `02f6209`, `b40ea66`, `46925ee` are preserved. Merge `4e2866b` joins the team's latest user-pulled main `e9532fa` without overwriting either history. The verified original checkout at `C:/Users/saket/OneDrive/Documents/GitHub/Dronehacks-2026` remains clean on main. This working checkout is under this chat's `work/dronehacks_autonomy`. `origin` is the team URL; `handoff` is the original bundle. No upstream team feature branch is assumed; fetch/compare authenticated remote refs before any later publishing.

Only `shared-context/saketh-desu.md` was edited. The other four files and shared-context README remain intact. User instructions to stay on the autonomy branch and avoid publishing override the shared folder's direct-main push convention. Claude's Gazebo worktree was never inspected or modified.

## Runtime interfaces

| Input/output | Contract |
|---|---|
| OAK Image + CameraInfo | raw left/right, sensor-data QoS; pair <=5 ms; calibrated rectification and timestamped optical-to-body TF required by default |
| VO odometry | nav_msgs/Odometry, `odom` -> `base_link`, nonzero monotonic capture stamp, finite pose/unit quaternion/covariance; sensor-data QoS |
| VO status | isaac_ros_visual_slam_interfaces/VisualSlamStatus; fresh state 1 required by default; unknown/lost/stale inhibits motion |
| Stereo scan / clearance | LaserScan + Range in base_link at image capture time; +inf measured clear, NaN unknown, -inf/short finite stop |
| Map / line fixes | absolute course PoseWithCovarianceStamped and body Vector3Stamped evaluated against historical odometry; duplicate/stale/frame/innovation gates |
| Commands | default dry-run TwistStamped to `/dh/dry_run/cmd_vel`; no flight-control publishers/clients |
| Sky Beta, non-dry mode only | ENU odom TwistStamped; only zero readiness before fresh authority; nonzero motion/progression after Offboard; stale/lost/frozen state latches abort. Never arms/lands or publishes FMU |
| Raw PX4 | simulation-only; not authorized for hardware |

## Completed changes

- Unknown stereo always stops; removed creep and tube-geometry exceptions. Missing bracketing capture pose/view changes also hold. Disabling the stereo node no longer disables the guard automatically.
- Historical pose/height and capture-time TF throughout camera-derived processing. Replay cannot refresh health. Default measured TF mount and no automatic nominal-baseline fallback.
- Freshness/uncertainty/XYZ/yaw discontinuity gates; pause + confirmed map recovery or abort. Bounded, observable map matching and a final mission innovation gate.
- Source-verified zero-twist startup handshake, strict boolean authority parsing, frozen-interface protection and permanent takeover abort. Dry-run isolation stays default.
- Illuminated nominal tube and height-aware offline camera renderer. Fixed the false full-height obstacle extrusion that blocked the real OpenCV loop. A* + clothoid planning and tube-axis anchors remain unchanged.
- Added real ROS launch/DDS/TF/image tests, fake-bus regressions, stationary guide and persistent AGENTS instructions preserving team rules.

## Evidence and remaining boundary

Detailed commands, results, prior failures, source revisions and limitations: `docs/LOCAL_VALIDATION.md`.

- Unit tested: 17 core tests, 30 preserved team tests; 88 organizer pure Python checks pass with two ROS/PX4-dependent checks skipped.
- Fake-bus node integration: 12 existing + 12 safety/OpenCV tests pass.
- Synthetic camera processing: 12 stereo scenarios, 14 line scenarios pass assertions.
- Python ray-cast SIL: 4/4 route/seed runs reach pad hover with zero contacts; fake-PX4 return reaches DONE. These are not actual PX4 SITL or Gazebo.
- Rendered images through actual OpenCV: 4/4 window/gate seeds 0/1 reach pad hover, zero contacts, 2.2-3.1 cm pad error and <=11.0 cm maximum pose error. Source/seed details are in the validation record.
- Organizer-compatible library pair on local Windows: numpy 1.26.4/OpenCV 4.10 also passes 47 unit, 24 fake-bus and all stereo/line scenarios. Not ARM/ROS qualification.
- Actual ROS 2 integration: NOT TESTED. Both launch tests are written/syntax-checked and require isolated domain 77.
- Jetson hardware, real cameras, motion/flight: NOT TESTED. No SSH, physical commands, flight battery or propellers involved.

Next operation boundary: stop before SSH/hardware. The later procedure is `docs/JETSON_VERIFICATION.md`: user/organizer approval, Nano sensor-only environment, build + isolated ROS launch tests, real camera/TF/time checks, stationary dry-run and command-isolation checks. No flight or arming is part of it.

Known limitations: event image and numpy<2/ARM untested; measured mount/radius/resting height and Nano CPU throughput needed; limited stereo height coverage; small synthetic clearance margins; conservative timing can slow or hold; Sky Beta cannot land/disarm; substantial localization resets may require restart. Older planner-suite scores remain historical.

## Read-only Jetson compatibility audit — 2026-11-07

The user authorized read-only SSH to skybeta-02. Inspection completed and the audit connection closed; no remote installs, deployment, service/config changes, feature autonomy launch or physical command publication. Full report: JETSON_COMPATIBILITY_REPORT.md. The earlier no-SSH boundary was superseded for inspection only. The baseline 0ec8c81 branch was published in the prior task.

Observed: Orin Nano Super reference kit, 7.4GiB RAM, L4T36.5.0, Ubuntu22.04.5/ARM64. Existing skybeta-team:0.3.3 container: Humble/domain2, Python3.10.12, numpy1.26.4, OpenCV4.5.4, cuVSLAM3.2.6, depthai3.10.0. Live raw/mono960x600, on-device depth320x200, IMU, VO odom->base_link/status1, Sky Beta state and factory TF7.53cm baseline match expected message contracts. No missing core ROS dependency identified. Raw receipt pairing/load and VO age spikes require validation; max observed VO age422ms exceeds our400ms hold guard, which remains unchanged. Downward dist_bottom_valid=false.

Local fixes: dedicated stationary_mission hard-locks DryRunBackend despite false/px4 overrides; stationary.launch.py and unit02 mono/mount profile; bounded16 camera/scan retention for delayed odometry without relaxing capture slop/age; graph checker requires positive mission and dry publisher evidence. Normal A*+clothoid planner and tracker remain unchanged. Profile/launch is not deployed or executed on hardware.

Validation: 47 unit, 12 existing fake-bus, 16 safety/compatibility tests, 12 stereo scenarios and14 line scenarios pass in the isolated numpy1.26.4/OpenCV4.10 environment; standalone current-runtime core/safety checks also pass. Python3.10 grammar/compile checks pass. Real ROS observation/TF lookup was exercised read-only on hardware; feature launch/build/processing/integration and physical command isolation have NOT been tested there. Exact OpenCV4.5.4 processing remains pending.

Next boundary: controlled-deployment review, not permission to deploy. Reconfirm physical props/battery conditions and controller ownership, review package transfer/build and isolated-domain synthetic ROS tests, then a separately authorized live stationary run with stationary.launch.py. No flight/arming or physical zero-command test is authorized.

## Phase 1 stationary preparation — 2026-11-07

Source reviewed at eab363673fbb6b5272ab1d54115e77499cba22ca; two autonomy packages remain unchanged. Latest origin/main context 9186743 read, all five files preserved. STATIONARY_DEPLOYMENT_PLAN.md provides exact proposed commands, expected outputs, rollback, changed remote paths and phase-specific acceptance gates. Offline build/inspect controls are under tools/hil/stationary_phase2. Proposed isolated host workspace: /home/dronehacks/codex_stationary/eab3636-phase2; proposed separate container uses existing image by immutable ID, network none/private IPC, no devices/host workspace mounts, nonroot/read-only root, capped resources. Neither has been created or checked remotely in Phase 1.

Source findings: stationary entrypoint always selects DryRunBackend despite false/px4 parameters. No imported startup/launch path starts a driver/bridge/physical backend. ROS remaps could create forbidden endpoints before the post-construction check, so exact reviewed invocations and independent endpoint inspection remain necessary; this is not an arbitrary-code security guarantee. Existing ROS synthetic tests use normal dry-run mission; proposed sandbox also checks installed stationary_mission with false/px4 parameters.

Local verification: 16 safety regressions pass under numpy1.26.4/OpenCV4.10; Bash syntax and Python3.10 grammar pass; container inspection fixture accepted and17 unsafe variants rejected; pinned git archive51 safe members match source. First sandboxed local Bash check hit Windows signal-pipe permission failure; syntax-only retry passed. No feature ROS build/launch, Docker sandbox, remote connection/write or hardware operation performed in this phase. Actual target build/tools/permissions/launch/processing remain untested. Explicit Phase 2 approval required before transfer/build/synthetic execution; Phase 3 live perception and Phase 4 stationary autonomy each require separate later approval. No flight commands, including physical zero commands, are authorized.

## Phase 2 control safety hardening v2 — 2026-11-07

Independent review requested automatic deadline/resource monitoring and optimization-proof validation before authorization. Only offline controls/plan/docs changed; dh_core/dh_autonomy and eab3636 source archive remain unchanged. Proposed outer GNU timeout PID1 enforces TERM600s/KILL605s, with executable/cmdline/GNU/graceful/forced-probe checks before build. Host supervisor verifies exact stopped sandbox and uses its full ID for start/stop/KILL; monitors RAM/disk/cgroup memory/OOM, elapsed time and existing organizer identity/restarts. Read-only host lease prevents build before complete telemetry and expires after12s on monitor/disconnect failure. No privileges/hardware/organizer mounts added. Camera/SLAM health remains explicitly unmonitored unless a separately approved existing read-only source is supplied. Details: revised STATIONARY_DEPLOYMENT_PLAN.md and docs/PHASE2_SAFETY_HARDENING.md.

Local mock suite17/17 ordinary Python and17/17 python -O;46 unsafe configuration variants rejected per run. Bash/Python3.10 syntax and unchanged source/hash checks pass. Earlier local fixture/temp failures recorded in change notes. These are not actual Docker/Linux namespace/cgroup/ROS/Jetson tests; mandatory runtime probes gate future build. No SSH, container start/deploy/build or physical operation executed. Phase2 remains UNAUTHORIZED pending user review of phase2_safety_review_v2.zip; Phases3/4 remain separately gated. Shared context preserved; latest team-main9186743 read.

## Authorized Phase 2 attempt — BLOCKED — 2026-11-07

User confirmed organizer permission and explicitly authorized the reviewed v2 Phase2 procedure; no further routine approval is needed within that scope. Package verification passes (7 payload/54 packaged-file hashes, exact c54471a controls, unchanged eab3636 archive). Latest origin/main afc008f team context read. One SSH attempt to dronehacks@skybeta-02.local failed before authentication with `Could not resolve hostname skybeta-02.local: No such host is known.` Required preflight failed, so stopped without alternate-address retry/workaround. No remote shell command, writes, Docker/monitor startup, build, ROS tests or physical operations occurred. Current target resources/image/stack/sensor health and timeout compatibility remain unverified; no runtime/peak measurements exist. Details and exact command: docs/PHASE2_EXECUTION_REPORT.md. Restore target connectivity/name resolution, then retry already-authorized preflight. Phases3/4 remain prohibited; source/control package unchanged.

## Phase 2 actual run — FAIL; export and cleanup verified — 2026-11-07

User supplied192.168.68.77 after .local resolution still failed; known skybeta-02 host key and remote identity verified. Followed reviewed c54471a controls/eab3636 source exactly. Target Docker isolation verifier passed; runtime GNU timeout identity/graceful/forced probes passed before work; host lease/resource/organizer monitoring active. Both ARM/Humble packages built in9.76s;17 native core tests passed. dh_autonomy selected `/usr/bin/python3 -m unittest -v` and ran0 tests. Required JUnit DDS-case validation raised RuntimeError, ending sandbox with exit1 after26.52s; no DDS integration or separate stationary graph check executed. No retry, source fix, resource/permission relaxation, Phase3/4 or physical commands. Sampled memory peak206.78MiB; host available RAM stayed>=4.26GiB, no OOM events. Peak sandbox CPU was not captured during active work. Organizer ID/image/PID3256/start time/restart0 unchanged; sensor processing health not independently monitored. Full report: docs/PHASE2_RESULTS.md.

User then explicitly authorized narrow cleanup after complete verified local export. All239 regular-file hashes,76 directories and5 symlink records verified, gzip CRC/TAR contents checked and readable logs/XML saved on Windows. Complete export SHA2568ffc8c6898a6d013e6f141209f6e732771d14426c1b69db4167e06a5f5aec4f8. Revalidated literal canonical UID1000-owned path, contents/ownership/hashes, stopped container identity/config/mounts and inactive monitor. Removed only container60357d35ddd41b563a96b06206168cf64e1ab1831b588bdfa41ecab3d8e77e1b and /home/dronehacks/codex_stationary/eab3636-phase2. Confirmed both absent, organizer unchanged and monitor inactive; SSH closed. No outside-directory cleanup/prune/images/teammate files affected. Evidence retained locally in outputs/phase2-complete-export.tar.gz and phase2-jetson-evidence. Acceptance failed: review pytest/launch-test registration and revised package before another run; do not advance Phase3.

## Corrected Phase2 runner candidate — local only — 2026-11-07

Added tests_require=[pytest] to dh_autonomy/setup.py. Actual Colcon0.20.1 default selector returns false before/true after. Existing ROS test semantics unchanged. Real pytest collection with import-only ROS stubs finds both mandatory methods plus shutdown (3 cases), NOT real ROS execution. Added collection/JUnit/actual-command/final executed-isolation gates, explicit colcon pytest and failure return code. Automatic verified export/narrow cleanup prepared; no SSH. Candidate generation reads only frozen git-show/archive commit, rejects old TAR/hash, emits source_commit.txt/new manifests. PHASE2_RETEST_PLAN.md preserves reviewed limits/isolation/lease.

Local47 unit/12 fake-bus integration/16 safety/12 stereo/14 line pass;10 gate/export and17 control tests pass normally/-O. Humble unavailable locally; no DDS/Jetson success claim. Freeze candidate before navigation changes. Main481cbf8 inspected: mapping/refnav/sim policy scripts available; shared-context removed there, so preserve autonomy branch's five files and existing local teammate context. Later blue-pad/15deg/CAD-free changes must stay outside this frozen candidate.

## Forward-camera prototype checkpoint; integration audit takes priority

Frozen corrected Phase2 commit f85c49218f971ff52c53219a1960cd4887cfc3d4 and generated retest package remain unchanged. No SSH or hardware operation in this task. Optional online prototype is preserved separately: team mapper reused, full-body unknown/legal/over-wall gates, shared 3D search and existing smoothers/follower, blue/orange OpenCV pad track and bounded aperture classifier. Camera remains forward-facing; actual measured TF is authoritative. Both tunnel/window required, actual legal numeric envelope/floor and RGB stream/calibration unverified, defaults hold. No autonomous landing mechanism. 91 local unit tests and7 fake-ROS adapter tests pass; not real ROS, Gazebo or hardware. Observed-voxel passage fixtures are not end-to-end tunnel/window camera runs. Actual frontal-view startup remains safely blocked by uncertified surrounding volume. Detailed limitations in docs/ONLINE_AUTONOMY.md.

Latest user explicitly prioritizes a focused interface audit and smallest reuse-first A*+clothoid integration over more infrastructure. Existing main481cbf8 map node publishes occupied cloud/display image but retains full tri-state voxel data privately; no planner-facing grid/capture-time/freshness contract yet. Preserve optional prototypes; do not make them a prerequisite. Existing planner_astar.ClearancePlanner, timing.fillet_path/time_parameterize, tracker.TrajectoryFollower and SkybetaBackend should be reused directly in the minimum adapter. Original CAD remains only sensor/test geometry. Further test results and exact commits will be recorded after this checkpoint.

## Minimum executable reuse-first policy

Latest user supersedes simulation/advanced-feature work: no further simulations. New local_policy_node subclasses the EXISTING team dronehacks_nav.map_node.MapNode (main481cbf8); TimedTeamMap calls its integrator and adds ages/immediate positive-hit veto. live_map_adapter converts full fresh vertical bands to existing Grid2D/ClearancePlanner, existing clothoid/time parameterization/follower, existing independent SafetyGate/filter. Safety filter imported with cherry-pick provenance6c3a0a9 ->0bf8559; local policy explicitly uses latency.8s/braking.6m/s²/speed.3m/s/accel.4m/s²/odom400ms. No planner/SLAM/flight controller rewritten. Short observed forward goals, bounded recent-goal penalty, path invalidation/replan and stopping-sweep checks. Fixed reviewed below-wall height only; defaults unverified and hold. No takeoff, return, landing or full-maze completion claim. Launch local_policy.launch.py hard-locks local_policy_stationary dry backend; no drivers/SLAM/flight_interface startup. Needs installed dronehacks_nav dependency; root-package integration patch prepared separately.

Lightweight checks:50 existing core/safety pytest cases +7 focused fake-ROS checks pass, including actual team mapper source import/native depth-TF path, malformed/new invalid coverage, full-band unknown/altitude rejection, bounded commands/obstacle mask veto, stale inputs, actual new node readiness/Offboard/takeover and hard-locked stationary isolation. Not actual ROS/DDS, Gazebo or hardware. Initial development scenario startup deadlock fixed by evaluating configured speed ceiling (not zero initial reference) before existing follower. Subsequent scenario generated3.26m/3replans/0contacts but FAILED its required post-insertion-stop assertion; it is retained as failed/nonqualifying. User stopped simulation work; no additional scenario runs. No Jetson access/deployment/physical commands. Main/Gazebo/teammate worktree untouched.

Details, exact inputs, launch command, source audit, estimates and blockers: docs/MINIMAL_POLICY.md. Full-body near-start observability, actual legal altitude/prop envelope, depth-frame/rectification, measured latency/braking and real ROS execution are unresolved policy-specific flight gates. Existing successful organizer hardware testing is accepted, not repeated. Optional advanced prototype preserved atafd8b80; not default or prerequisite. Frozen Phase2 f85c492 remains separate and unchanged.

## Integration correction from e42e5a3 — 2026-11-08

Existing obstacle-stop assertion root cause proven: obstacle was inserted east while the route travelled north. Corrected existing fixture proves route intersection and noncollision at insertion; original stop/motion/collision assertions all pass (2.644m,5 replans,0 contacts). No new simulator. Real defect fixed: depth clearance now transforms capture-heading points through odom ENU into current heading/position; FOV uses capture heading. ENU world velocity at90deg yaw regression passes.50 core/safety +11 focused fake-ROS checks +1 corrected obstacle regression pass. Existing two ROS cases+shutdown collect3 with import stubs; new installed-policy Humble/DDS smoke case collects1. Collection is not execution. Actual Humble unavailable: local WSL has no installed distribution; no ros2/docker executable. No actual ROS/Jetson pass claim.

Startup now refuses /dh/start until verified geometry/rules, height, fresh health and full measured body volume; reports explicit WAIT_MEASURED_FULL_START_VOLUME. No free bubble, expiry relaxation, blind takeoff/yaw or automatic360. Static forward FOV plus500ms evidence lifetime can still cause permanent HOLD: remains first-flight NO-GO, despite the corridor being a simple snaking path. Safe observation/retention must be validated before flight; stationary diagnostics may record this hold truthfully.

Permanent dry launch/config and existing working mission unchanged. Separate default-disabled physical review launch/profile prepared; physical constructor requires review before creating endpoints. Exactly one flight_interface and exclusive setpoint-topic ownership required before publishing. sensor_msgs_py dependency explicit. New tools/hil/verify_local_policy_ros.sh requires isolated Humble/domain78, proves actual pytest runner/JUnit/no skips on the new smoke case. Not executed here. Frozen Phase2 f85c492 ZIP/TAR hashes unchanged; original integration/isolation gates pending separately authorized execution.

New additive root-package patch supersedes prior mission-replacement patch: retain team mission entrypoint and select local_policy explicitly. No main/Gazebo/teammate-worktree changes, SSH, deployment or physical commands. Full configuration/measurement and GO/NO-GO list and exact stationary command: docs/POLICY_INTEGRATION_FIXES.md.

## Judge demonstration — local execution and media

User authorized a complete local simulation/recording workflow, superseding prior simulation pause. Imported the supplied FINAL Course v3.step into31 labelled metre-unit meshes. Reused team origin/sim702104c Sky Beta SDF visual geometry. Existing Gazebo requires unavailable Linux/Colima/Docker/ROS; used existing fake-ROS/SIL boundary and sensor/grader plus Blender4.3.2 BVH capture/cutaway replay. Production ros2_ws and frozen Phase2 controls unchanged from738fdc3. No simulator truth passed to navigation.

Actual forward-depth course run: unchanged StationaryLocalPolicyNode processes native simulated Image/CameraInfo/odom/VO/TF, starts unknown, refuses /dh/start due full starting-volume observability.0m/0replans, no contacts at stationary pose (conservative sphere clearance .1109m), depth-loss also holds. Course mission did NOT complete. Existing ideal panoramic component diagnostic: actual mapper/A*/clothoid/follower/independent filter/DryRunBackend,2.644m/5replans/obstacle stop/0gradedcontacts, minimum configured cylindrical horizontal clearance .3942m. Kinematic Euler/noPX4 dynamics; not physical forward-camera equivalence and not a full-course success. Windows fakeROS, not actual DDS/Humble/Gazebo/PX4. Tunnel/window/blue-pad recognition/landing unvalidated.

Produced outputs/judge_demo/main_demo.mp4 (30s,1920x1080,H264,20fps), highlight.mp4 (14s),7annotated1920x1080 screenshots, per-frame maps/plans/poses/images, command CSV, full decode verification and judge summary. Media explicitly labelled recorded-execution REPLAY and ideal-sensor diagnostic where applicable. Metrics reconciled to traces. Full source/reproduction/limitations in docs/JUDGE_DEMO.md; scripts under tools/judge_demo. Initial pybullet source dependency build cancelled; initial bulk mesh-ray allocation interrupted at~8.77GB; replaced with fast bounded Blender BVH. No safety guard weakened to produce movement. Full-course startup blocker remains NO-GO.

## Continuous forward-camera policy — 2026-11-08

Published source8e2a594d478f56ad071cbfdf4acb99ac49563c0a on saketh/autonomy;
normal push and remote head verified through the authenticated team checkout.
No main or teammate working-tree changes. Existing planner, clothoid smoother,
trajectory follower, team voxel mapper and independent filter are retained.

Startup fixes separate fresh live sensors from bounded decaying map evidence,
add optional conservative whole-voxel depth-image carving, and permit an
explicit physics-verified expiring launch-clearance certificate. Positive hits
always veto it; it certifies no route. Guarded stationary yaw/route-heading
observation, stalled-reference recovery and observed side targets are optional.
Removed the redundant A* start-cell-centre stub that could point backwards and
deadlock the live FOV gate; complete path validation remains. Swept cylinder
clearance uses exact circular lateral geometry with the existing braking filter.
Physical/default geometry, limits, launch/profile and dry lock remain unverified
and unchanged; simulator-only bounds must not be copied to hardware.

Actual tests:94 core pytest,11 existing fake-ROS policy checks,10 new focused
evidence/startup/geometry checks,16 safety regressions,12 rendered OpenCV stereo
cases all pass. Corrected existing obstacle regression passes original motion,
stop and collision assertions:0.764m/3replans/0contacts with ideal panoramic
fixture, not a forward-camera course claim. Actual DDS/Humble/Gazebo/PX4 and
Jetson validation were NOT performed. WSL has no Linux distro; teammate Gazebo
requires unavailable Docker/Linux/ROS. Local fallback is MuJoCo4.0 rigid-body
force/torque servo with rendered forward z-depth, bounded noisy pose sensor and
emulated ROS boundary, explicitly no SLAM reconstruction or PX4.

Latest three bounded policy cases: original geometry3.686m/4replans/0contacts
then navigation deadlock; north start3.313m/9replans/0contacts then deadlock;
altered walls15.714m/19replans/0contacts over179.95s then timeout with sustained
HOLD. Minimum hull clearances respectively.160m,.154m,.153m. Altered run
geometrically crosses the complete tube (simulator grader only); semantic
passage recognition/window traversal/blue-pad discovery/landing/full-maze
success are not demonstrated. Modelled height remains.85m, below walls.

The current-envelope unknown veto remains a real limitation after long pauses:
for the original case the nearest unknown column is.418m away versus required
inflated radius.433m; the policy cannot waive that missing evidence. This is
safe failure, not completed navigation. Source8e2a594 adds occupied-confidence
aging to blocked unknown while preserving the prior frontier mask and all
collision masks. Final reproduction20 finished at184.25s with15.714m/19replans/
0contacts and NAVIGATION_DEADLOCK. All five recorded source hashes match8e2a594;
the shared1800-frame position AND command trace matches attempt19 exactly.

All failed logs and recordings from attempt04 onward are preserved separately
under outputs/continuous_demo; bootstrap attempts01–03 have traces/logs only.
Original marker-band placement fails the conservative launch certificate;
moving cases use an explicitly model-verified +.05m east,+.15m north placement.
Stricter per-axis-error placement recheck passes with4.4mm model clearance;
this is not a physical placement tolerance claim. Main120s, highlight25s with
complete tube crossing, uncut184.3s and tracking/first-person MP4s pass full
sequential decode.15 extracted1080p frames verified; actual sensor-map/physics
track figure and all20attempt metrics saved locally. Final classified band
coverage14.73%, cumulative sampled lower bound46.70%; not maze completion.
See docs/CONTINUOUS_DEMO.md and docs/CONTINUOUS_DEMO_RESULTS.md.
Frozen Phase2 f85c492 ZIP/TAR hashes are unchanged. NO-GO for physical flight:
new-policy ROS execution, live timing/compute/TF/calibration, measured envelope/
altitude/braking/launch clearance, representative robust navigation and approved
landing authority remain gates. Latest main11280a8 still lists a depth/VIO
latency issue to reconcile with the actual tested hardware configuration.
No SSH, deployment, physical publishers, main/drone push or promotion occurred.
