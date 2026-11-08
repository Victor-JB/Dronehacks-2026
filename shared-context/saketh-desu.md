# Saketh Desu — Agent Context

**Role:** Dead Reckoning

_Update this file every time your agent context changes. See `shared-context/README.md` for the rules._

---

## Current autonomy state — 2026-11-06, Codex

- Owner: Saketh / Codex on `saketh/autonomy`. Local development complete; stopped before SSH or hardware. Claude independently owns Gazebo on `saketh/gazebo`; its worktree was not touched.
- Restored and preserved all Claude commits through 46925ee. Merge 4e2866b joins user-pulled team main e9532fa and the handoff history. Implementation commit 893dbfd; final documentation/status commit follows. Original local team checkout remains clean on main. Nothing pushed and no Git credentials changed.
- Unknown/stale/invalid stereo stops every phase and freezes the reference. Removed lit-tube blindness/creep assumptions. Historical odometry/height and timestamped TF; measured TF mount/calibration by default; bounded observable map fixes; VO freshness/covariance/discontinuity holds and recovery/abort. Sky Beta zero readiness before authority, motion only after fresh Offboard, no resume after loss/frozen state. Default A* + clothoid unchanged.
- Corrected the offline camera renderer's false full-height obstacle extrusion. Real OpenCV closed-loop matrix: window/gate, seeds 0/1 all reach pad hover, zero contacts, 2.2-3.1 cm pad error, <=11 cm max pose error. Ray-cast matrix 4/4 clean; fake-PX4 return DONE. These are Python simulations, not Gazebo or real PX4 SITL.
- Tests: 17 core + 30 team unit pass; 12 integration + 12 safety on fake bus pass; 12 stereo and 14 line scenarios pass. Same component suites pass under numpy 1.26.4/OpenCV 4.10 on Windows. Organizer pure contract tests 88 pass/2 skipped. Actual ROS 2 DDS/launch tests and all Jetson/real-camera/motion/flight checks are NOT RUN.
- Source interfaces inspected: organizer fleet-ready 7efe7e4, NVIDIA release-3.2 e31f4cc. Event image still needs stationary verification. Sky Beta cannot land/disarm; raw PX4 stays simulation-only.
- Details: AUTONOMY_STATUS.md, docs/LOCAL_VALIDATION.md, docs/JETSON_VERIFICATION.md. Next session is sensor-only Nano, props off and flight battery absent, after separate approval; verify ROS/TF/timestamps, run isolated real ROS tests, then stationary dry-run isolation and board/lens checks. No arming/flight or AGX access.
- Preserve all five shared-context files. User's branch-only/no-publish instructions override shared-context README's direct-main push workflow; this file is updated locally on the autonomy branch. Other teammates' context files are unchanged.

## Read-only Jetson audit in progress — 2026-11-07

User authorized read-only SSH to skybeta-02, superseding the prior SSH stop. Observed Humble/Python3.10/numpy1.26.4/OpenCV4.5.4, 960x600 camera streams, active VO and a 7.53 cm TF baseline. No remote installation/deployment/settings changes/autonomy launch/physical commands. Local stationary hard-lock and bounded timing buffers under test; report will distinguish observed data from remaining calibration/performance checks.

## Audit checkpoint complete — 2026-11-07

Read-only hardware audit completed; separate audit SSH session closed. No remote install/deploy/settings/service changes, feature launch or physical commands. Details: JETSON_COMPATIBILITY_REPORT.md and AUTONOMY_STATUS.md. Humble/domain2/960x600 stereo/mono/TF7.53cm and VO contracts observed. OpenCV is4.5.4; real processing still pending. Existing on-device depth uses RGB-labelled frame and was not substituted for raw stereo. VO age spikes422ms and limited received raw pairs are pending performance risks, not grounds to loosen safety.

Local stationary backend hard-lock, optional unit02 mono/mount profile, bounded capture queues and positive graph isolation evidence are validated. Tests:47 unit+12 integration+16 safety; stereo12/line14 scenarios pass locally under numpy1.26.4/OpenCV4.10. No actual feature pipeline has run on the Nano. Normal branch push only; controlled deployment requires next-stage review.

## Phase 1 preparation complete — 2026-11-07

Reviewed eab3636 source and latest origin/main9186743 context; package code unchanged. STATIONARY_DEPLOYMENT_PLAN.md and offline build/container-verifier controls define a separate nonroot network-none/private-IPC sandbox, preserving existing stack/workspaces/image. Reviewed stationary backend is forced dry; arbitrary ROS remaps are outside that guarantee and remain prohibited. Local16 safety tests/Bash syntax/Python3.10 checks pass; approved Docker fixture accepted,17 unsafe variants rejected, pinned archive51 safe members matches source. No SSH, remote writes/builds/launches or hardware operations in Phase1. STOP: explicit Phase2 approval needed for proposed isolated transfer/build/synthetic tests; no live sensor startup. Phases3/4 each need separate approval. Push only saketh/autonomy; main and other four context files preserved.

## Phase 2 safety hardening v2 — 2026-11-07

Local-only control hardening after independent review: exact GNU timeout PID1 TERM600/KILL605, mandatory safe runtime probes before work, verified-full-ID host supervisor for RAM/disk/cgroup OOM/elapsed/organizer restart checks, read-only12s monitor lease failing closed on logout/loss, and explicit validation effective under -O. No hardware permissions or organizer changes; source archive/pin eab3636 and autonomy packages unchanged. Camera/SLAM health explicitly unmonitored absent separately approved existing status source.17 mock tests pass normally and under -O;46 unsafe config variants rejected each run; syntax/hash checks pass. No SSH, deployment, Docker start, ROS build or hardware operations. Awaiting explicit Phase2 authorization after v2 ZIP review; Phases3/4 separately gated. Only saketh/autonomy updated, other teammates/main untouched.

## Phase 2 authorized, execution blocked — 2026-11-07

Organizer permission and reviewed v2 Phase2 execution explicitly confirmed by user. Local ZIP/payload/control/source integrity passed. Latest team-main afc008f context read. SSH failed before authentication: skybeta-02.local hostname could not be resolved. Stopped on this preflight failure; no alternate-address retry, target commands/writes, Docker/monitor, build or ROS tests. No hardware metrics/current stack-health evidence obtained. Report: docs/PHASE2_EXECUTION_REPORT.md. Await restored connectivity/name resolution to retry authorized preflight; no extra routine authorization needed. Phases3/4 prohibited, eab3636 source/c54471a controls unchanged, other team files/worktrees preserved.

## Actual Phase2 attempt and authorized cleanup — 2026-11-07

User-provided192.168.68.77 matched saved skybeta-02 key/identity. c54471a isolation/deadline/lease controls ran unchanged: both packages built natively,17 core tests pass; dh_autonomy incorrectly used unittest and ran0 tests. Required DDS-case gate failed as intended; exit1 after26.52s, no DDS integration/stationary graph check and no retry/Phase3. Sampled peak memory206.78MiB, host RAM>=4.26GiB,0 OOM; active peak CPU not captured. Organizer ID/image/PID/start/restarts unchanged; sensor health not independently qualified. Full details docs/PHASE2_RESULTS.md. Source/planner/control code unchanged.

Additional user cleanup authorization executed only after full Windows export verified239 file hashes/76dirs/5links. Removed exact verified sandbox60357d35ddd41b563a96b06206168cf64e1ab1831b588bdfa41ecab3d8e77e1b and canonical /home/dronehacks/codex_stationary/eab3636-phase2; confirmed absent, monitor inactive, organizer preserved, SSH closed. No broad prune, outside deletion or teammate changes. Complete evidence retained locally. Test discovery needs review/fix/requalification before advancing; no claim Phase2 passed.

## Frozen runner correction before CAD-free work — 2026-11-07

Local pytest registration fixed; actual Colcon0.20.1 selector false->true, import-stub pytest collection required2+shutdown. Collection/JUnit/actual-runner/final-isolation gates and automatic verified export/narrow cleanup added.47 unit/12 integration/16 safety/12 stereo/14 line pass;10 gate/export+17 controls normally/-O. PHASE2_RETEST_PLAN.md and frozen-commit artifact generator separate this candidate from upcoming blue-square/orange-floor goal,15deg TF and unknown-map work. Main481cbf8/team map/refnav/sim scripts inspected; preserve all five context files despite removal on main. No SSH, deployment, main/promotion/drone push or Gazebo worktree edits.

## Minimum policy local checkpoint

Autonomy branch preserves f85c492 corrected Phase2 retest baseline unchanged. Latest scope is executable reuse-first A*+clothoid integration, no further simulations/Jetson access. Existing team main481cbf8 MapNode/VoxelMap reused in process; minimal age/full-height adapter -> existing ClearancePlanner/clothoid/follower -> existing Stage4 safety filter -> existing Sky Beta backend. Hard-locked dry launch, no duplicate flight_interface.50 core/safety +7 fake-ROS component checks pass; not DDS/Gazebo/hardware. Earlier custom scenario assertion failed and is not qualification. Advanced prototype retained/paused. Actual legal envelope/physical dimensions, safe initial observation, depth geometry and timing and ROS execution remain policy-specific blockers. See docs/MINIMAL_POLICY.md. Do not overwrite teammates' four context files or their dirty stage4-safety checkout. Latest main no longer has the five files; keep this branch's preserved context system.

Integration fixes from e42e5a3: corrected obstacle fixture now intersects actual path and passes original assertions; fixed captured-depth/current-heading clearance frame mismatch; ENU and startup/ownership regression coverage added.50 core/safety+11 fake-ROS checks+1 focused existing obstacle run pass. Actual ROS unavailable locally (no WSL distribution); new real Humble smoke and fail-closed runner prepared but not executed. Frozen Phase2 candidate unchanged. Full starting-volume coverage remains unresolved with static forward camera/500ms voxel freshness; startup explicitly refuses start, no blind360 or fabricated free bubble. Separate physical review selection default-disabled; permanent dry launch and legacy mission preserved. See docs/POLICY_INTEGRATION_FIXES.md. No hardware/main/drone changes.

Judge demo local deliverable: actual provided STEP converted to simulator-only geometry; unchanged738fdc3 forward-camera policy correctly HOLDs at startup (0m), not full-course success. Separate clearly labelled ideal-panoramic diagnostic records real mapper/A*+clothoid/tracker/safety/dry-backend motion2.644m,5replans,obstacle stop,0contacts.30s main/14s highlight videos+7screenshots+trace/CSV evidence, decoded and reconciled. Native ROS/Gazebo/PX4 unavailable; message boundary emulated; no hardware claims. tools/judge_demo and docs/JUDGE_DEMO.md reproduce. Physical code and frozen Phase2 untouched; no SSH/main/drone/promotion or teammate worktree changes.
