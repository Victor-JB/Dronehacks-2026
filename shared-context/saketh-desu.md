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
