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
