# Instructions for coding agents

Read `AI_CONTEXT.md`, `CLAUDE.md`, and the files you will edit before acting.
This is the team's persistent DroneHacks companion project. Treat existing
mission code, notebooks and team instructions as team-owned work.

- Use ordinary ROS 2 / `px4_msgs`, camera calibration and TF interfaces.
- Keep simulator truth, private host paths, lab APIs and reset commands out of
  flight algorithms. A passing simulator test is not hardware qualification.
  Exception: custom simulated sensor **models** may use `dronehacks_sim` as
  documented in `SIM_SENSORS.md`. Keep the oracle producer separate from the
  portable navigation code and label assumed sensor errors and oracle use.
- Explain a flight plan and obtain the team's approval before arming. Inspect
  fresh telemetry; stop publishing if authority is lost or telemetry is stale.
- Preserve the sensor/estimator checks. Never invent a position fix, weaken a
  grader, or change PX4 parameters merely to make a failing mission pass.
- Run relevant project tests first; record the exact code, settings, outcome
  and evidence for moving tests. Preserve failures and explain corrections.
- Use the current editor's identity. Save and stop detached agents before
  handing editing control to another teammate. Never store credentials in Git.

The student's team owns mission approval. These participant instructions do
not override the simulator operator's separately granted SITL permissions.

After using keyboard RC, land and release RC, then use **Reset world** before
running an automated program, including a notebook or native terminal script.
The cloud follow-up run without Reset failed; the fresh-world native run passed.
The web Run button enforces this qualification boundary; terminal Python remains
ordinary ROS and does not enforce it for you. This is not a VIO fix.


# Autonomy development instructions

Read `CODEX_HANDOFF.md`, `AUTONOMY_STATUS.md`, `README.md`, `STRATEGY.md`, the files being edited, and the latest team context on `origin/main` before substantial work. Preserve existing instructions and the team's five-file shared context system; do not replace those files with autonomy-specific notes.

- Work only on `saketh/autonomy`. Preserve all history and uncommitted work. Never force-push or push feature code to main. Fetch and compare the remote branch before publishing; merge histories safely if needed.
- Claude owns Gazebo on `saketh/gazebo` in a separate worktree. Do not modify that worktree or Gazebo files as part of autonomy work.
- Physical flight is prohibited. Do not SSH to either Jetson or operate hardware in this task. Stop before the first hardware connection. The drone has no flight battery or propellers; sensors can be powered independently.
- Keep `dry_run: true` by default. Stationary verification must create no flight-control publishers or service clients. Do not arm, change PX4 parameters/firmware, or send hardware `/skybeta/*` or `/fmu/*` commands.
- Use portable ROS 2 interfaces, measured calibration and timestamped TF. Simulator truth belongs only in sensor models and test tools, never mission algorithms. Never invent fixes or weaken safety/tests to make a simulation finish.
- Keep A* + clothoid as the default planner. The tube is illuminated: never infer sensor blindness or waive safety from its geometry.
- Missing, stale, invalid or uncalibrated clearance is unknown and stops progression. Localization freshness must use capture stamps as well as receipt time.
- Check the actual organizer source contract and record its revision. Distinguish source-verified compatibility from the event image, which needs stationary verification.
- Run relevant local tests, preserve failures, and update `AUTONOMY_STATUS.md` with settings, evidence and blockers. Label unit tests, Python SIL, actual ROS 2 integration and hardware validation separately.
- Keep credentials out of files and Git. Use the current editor identity. Obtain team approval before any later arming mission; that permission is outside this local task.

## Read-only compatibility audit authorization (2026-11-07)

The user explicitly superseded the earlier SSH stop boundary for read-only inspection of skybeta-02. Read-only SSH/ROS graph and sensor observations are authorized. Installing, deploying files, changing system/PX4 configuration, starting autonomy or publishing physical command topics (including zeros) remain prohibited. Never save credentials. Local fixes/tests and normal pushes to saketh/autonomy are authorized. The stationary entrypoint must remain hard-locked to DryRunBackend regardless of parameter overrides.

## Staged stationary integration gate (2026-11-07)

Current authorization is Phase 1 local preparation only. Follow STATIONARY_DEPLOYMENT_PLAN.md. Stop for explicit phase-specific user approval before any remote write, deployment, build, service change or autonomy launch. A Phase 2 approval covers only the reviewed separate network-none/private-IPC synthetic sandbox, not live sensing; Phases 3 and 4 need separate approval. Do not treat domain77 alone as physical isolation. No physical commands (including zero), arming/disarming/mode changes or system upgrades are authorized. Preserve all existing running services/workspaces and record actual execution separately from proposed commands.

The latest safety-hardening task authorizes local Phase 2 controls/docs/tests only and prohibits all Jetson commands, including SSH. Autonomy packages/source pin remain unchanged. The v2 plan requires verified PID1 GNU timeout, an automatic host resource/organizer-liveness monitor and a read-only expiring lease; no build/test work may begin with missing telemetry or unverified timeout. Phase 2 is still unauthorized until the user explicitly approves the v2 review.

## Phase 2 execution authorization (2026-11-07; supersedes prior stop)

User confirmed organizer permission and authorized the reviewed v2 isolated Phase2 procedure, including SSH preflight and exact dedicated transfer/container/build/synthetic tests at c54471a controls/eab3636 autonomy pin. Stop on any failed prerequisite; never broaden privileges/resources/isolation or retry through an improvised workaround. Phases3/4 and physical commands remain prohibited. First SSH attempt failed local hostname resolution before authentication; no target commands/writes occurred. Once user restores skybeta-02.local connectivity, retry the already-authorized preflight without asking again for routine actions. Record actual results separately from proposed steps.

Phase2 was subsequently executed using the user-provided192.168.68.77 (saved host key/remote identity verified), then stopped on missing DDS test discovery. Build/core tests passed; overall acceptance failed. User separately authorized complete evidence export and narrow cleanup; both verified and completed, dedicated container/workspace absent, SSH closed. Do not automatically retry or advance Phases3/4. Review docs/PHASE2_RESULTS.md; test-runner registration requires a revised reviewed package before another execution. Preserve local exported evidence and all organizer/team resources.

## Current local development scope — supersedes SSH authorization

User authorized local runner correction/frozen retest package and then CAD-free online mapping/exploration/planning. STOP before all Jetson access; retest needs separate approval. Keep frozen Phase2 source/control commit separate from later navigation/pad changes; never package the moving branch accidentally. CAD fixture is regression-only; new navigation requires no course OBJ/truth. Reuse organizer odometry/depth/TF and team mapping/planners/safety. Finish is a blue square on orange floor; planned15deg camera tilt must be verified with measured TF, never silently assumed. Current Sky Beta has no verified autonomous landing authority: no invented land command, direct FMU or duplicate flight_interface. All planners share perception/map/goal/tracking. No main/promotion/drone push or Gazebo worktree modification.

Latest user corrections: camera stays FORWARD-FACING; 15deg modification cancelled. Use measured TF. No over-wall routes; numeric legal altitude/floor reference unconfirmed, never substitute the Sky Beta clamp. Both tunnel and window are required scoring objectives. Original CAD is a representative sensor/test environment only. Prioritize the minimum live-map -> EXISTING ClearancePlanner/clothoid -> EXISTING follower -> Sky Beta integration first. Preserve optional advanced prototypes, but do not expand or promote them as a prerequisite. Audit actual team map products before introducing adapters. No new mapping/SLAM/control/planning kernel without demonstrated incompatibility. No Jetson access, main changes or deployment.

Latest milestone restriction: STOP simulation runs/Gazebo/fly-sim and advanced feature work. Finish the executable minimum A*+clothoid policy using existing team MapNode, depth projection, planner, follower, independent safety filter and Sky Beta backend. Only lightweight targeted software checks now. Preserve earlier simulation failures and optional prototypes; do not present them as qualification. Normal code may describe future Sky Beta integration, but no physical publisher is executed; supplied launch hard-locks dry mode. Preserve f85c492 frozen Phase2 candidate and separate approval gate.
