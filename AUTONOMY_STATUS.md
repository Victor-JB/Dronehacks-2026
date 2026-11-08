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
