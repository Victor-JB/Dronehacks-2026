# Stationary deployment plan — Phase 1 — 2026-11-07

**Status: preparation only. No remote write, build, deployment, service change or autonomy launch has been executed. Phase 2 requires explicit approval of this plan and its isolated location/container. Phases 3 and 4 each need separate later approval.** Physical flight remains prohibited. User reports props and flight battery absent; reconfirm those physical conditions before later operations. Never send physical zero commands, arm/disarm, change modes, run motors or change PX4 parameters.

## Reviewed baseline and source findings

- Repository Victor-JB/Dronehacks-2026, branch `saketh/autonomy`; autonomy source pinned to **eab363673fbb6b5272ab1d54115e77499cba22ca**. Phase 1 adds documentation and offline build-control tools only; the two autonomy packages are unchanged. Latest fetched team-main is `9186743`; all five teammate context files read, only Saketh's file updated. No merge into main.
- Read `JETSON_COMPATIBILITY_REPORT.md`, current package source/manifests/launch/tests, persistent repository rules and status. The prior audit supplies target observations; no repeat SSH audit is needed in Phase 1.
- `setup.py` maps `stationary_mission` to `mission_node:stationary_main`; this calls `main(node_class=StationaryMissionNode)`. Its class flag is `force_dry_run=True`. `MissionNode.__init__` computes `self.force_dry_run or bool(dry_run_parameter)` before backend construction. Therefore this entrypoint always selects `DryRunBackend`, even for `dry_run=false` and `backend=px4` or `skybeta`. The physical classes' constructors are not invoked along that branch.
- `DryRunBackend` creates only `/dh/dry_run/cmd_vel` (TwistStamped) and `/dh/dry_run/events` (String). It subscribes read-only to `/skybeta/state`. `hold`/`send_position` log dry events; inherited arm/disarm/land/request_offboard do nothing and return false. Shutdown calls only this dry backend. It creates no control-service clients. Mission also publishes `/dh/{pose_map,plan,mission_state,health}`.
- `verify_isolation` checks resolved publishers/clients at constructor completion and raises on `/skybeta` or `/fmu` prefixes. **This is a startup check after endpoint creation, not a security sandbox:** arbitrary ROS remaps could create a forbidden endpoint before the check rejects it. No remaps, namespaces, alternate executables or unreviewed parameter/launch files are allowed in this procedure. A bridge external to the node could relay dry output; independent live-graph/transport review is still required before Phase 4. The guarantee is the reviewed entrypoint/backend and exact invocation, not arbitrary modified Python/ROS environments.
- `stationary.launch.py` loads installed `autonomy.launch.py` using `runpy`, then passes fixed `mission_executable="stationary_mission"`. No launch argument can replace that executable. The launch contains only mission, stereo, line and localizer actions and argument declarations; no driver, bridge, container, shell command or flight interface is started. The default course path points at the team's workspace; later commands must override it with a reviewed path in the isolated workspace.
- Imported module/package initializers have no ROS node construction, publisher creation, device open, process launch or hardware write at import. Backends defines a QoS object, classes and functions; core defines numerical constants (including an optional planner's small Hessian). Course loading/planning, TF listeners and timers happen in constructors. `main()` calls are protected by `if __name__ == "__main__"`. `setup.py` performs normal build metadata/file installation only during build. Optional raw-PX4 message imports occur inside the physical constructor, not at stationary import. Third-party import behavior is not a substitute for container isolation.
- Stereo/line/localizer create only measurement/health endpoints; TF listeners subscribe and may create normal ROS infrastructure endpoints. No socket/serial/process control path was found in these nodes. Normal ROS parameter/rosout infrastructure is expected; it is not a flight-control client. A* + clothoid, safety thresholds and existing planner remain unchanged.
- The graph checker requires both `dh_mission` and its dry command publisher, checks all discovered `dh_*` publishers/clients for forbidden prefixes, returns 0 pass / 1 forbidden / 2 incomplete. It assumes reviewed node names; renames, duplicate node names or relay nodes must be independently investigated. An empty graph never qualifies as success. The checker itself uses normal rclpy infrastructure, including default rosout; it publishes no flight commands.

## Dependencies and package layout

Both packages are `ament_python` 0.1.0, with resource markers and setup.cfg installing scripts to `lib/<package>`. `dh_core` installs the pure Python `dh_core` package; `dh_autonomy` installs its Python module, `share/dh_autonomy/{launch,config}` and console entrypoints. Only these two packages are transferred, plus the read-only checker and phase-control files. No `px4_msgs`, simulator, organizer packages, binaries, virtualenv or Git credentials are transferred.

| Requirement | Source / treatment |
|---|---|
| Build | Existing Humble, ament_python, setuptools, colcon and its ROS/Python/pytest extensions. Verify availability; never install to resolve a failure automatically |
| Core | python3-numpy; SciPy optional, not required |
| Autonomy | rclpy, dh_core, std_msgs, std_srvs, geometry_msgs, nav_msgs, sensor_msgs, builtin_interfaces, message_filters, tf2_ros, launch, launch_ros, ament_index_python, python3-opencv, python3-numpy |
| Tests | python3-pytest, launch_testing, launch_testing_ros and pytest launch plugin; tf2_ros static_transform_publisher executable |
| Live health | isaac_ros_visual_slam_interfaces required by the live profile, intentionally not auto-installed by manifest; synthetic tests disable VO status only in their fixtures |
| Raw PX4 | Not needed or invoked in the stationary path. Use no vendored package, XRCE agent or flight backend |

Prior audit observed Python3.10.12/numpy1.26.4/OpenCV4.5.4, ROS Humble, domain2/Fast DDS in existing `skybeta-stack`, image `skybeta-team:0.3.3`. Most runtime imports were observed; **colcon tooling, launch_testing_ros, unprivileged image-path access and actual build remain unverified**. Missing tools or permissions stop Phase 2; no apt/pip/rosdep, sudo, package upgrades or image rebuilds.

Existing organizer `/home/dronehacks/skybeta`, `/root/skybeta_ws`, host team `/home/dronehacks/team_ws`, container team `/root/team_ws` and all running services remain untouched. The proposed new host directory is **`/home/dronehacks/codex_stationary/eab3636-phase2`**, with a separate non-restarting container **`codex-autonomy-eab3636-phase2`**. Neither existence nor permissions has been checked remotely in Phase 1; collision/ambiguity is a stop, never overwrite/reuse/delete automatically.

## Phase 1 local preparation and evidence

Created locally with `git archive` from the exact reviewed commit, not from loose edited source:

```powershell
git archive --format=tar --output=../../outputs/stationary-reviewed-eab3636.tar eab363673fbb6b5272ab1d54115e77499cba22ca ros2_ws/src/dh_core ros2_ws/src/dh_autonomy tools/hil/check_command_isolation.py
```

Archive SHA256: `da31d4b63ed05099d2be592d292c05cc4be142a699a96dbe15387f68db3d4f70`.
External course input `C:\dhdl\obj\dronehacks-course-v3.obj` SHA256: `e610bcf9666422e2a0a78cb46f542fc8c4d5a7ef9b15f6123f1abe40f5b8e9e1`. This is a synthetic test fixture in Phase 2; it establishes no real pose/course alignment.

Controls are `tools/hil/stationary_phase2/container_build.sh` and `verify_container.py`; copies and SHA256SUMS are delivered beside the plan. Review their full contents with this plan before approval. They are newly prepared controls, not part of the previously reviewed autonomy commit. No command in the remaining sections was executed during Phase 1.

## Phase 2 proposal — approval required BEFORE the first remote write

Approval scope must explicitly include the new host directory, source transfer, new network-isolated container, bounded build/import/synthetic tests and temporary synthetic/stationary nodes **inside that container only**. It does not permit using `docker exec` in the existing stack to build or run features. Reconfirm independent power, absence of props/battery, controller ownership and adequate compute/storage headroom. Use existing Docker access without sudo. If Docker permissions are missing, stop.

### A. Read-only preflight after approval

SSH normally as `dronehacks@skybeta-02.local` without storing credentials. Record commands/stdout locally. On the host:

```bash
root=/home/dronehacks/codex_stationary/eab3636-phase2
name=codex-autonomy-eab3636-phase2
test "$HOME" = /home/dronehacks
test ! -e "$root"  # MUST succeed; never overwrite an existing attempt
test -z "$(docker ps -aq --filter name=^/${name}$)"
id
df -h /home/dronehacks
free -h
docker ps --no-trunc
image=$(docker inspect --format '{{.Image}}' skybeta-stack)
docker image inspect "$image" --format '{{.Id}} {{.Architecture}} {{json .Config.Volumes}}'
docker info --format '{{json .Runtimes}}'
```

Expected: collision tests success; existing skybeta-stack remains running; image architecture arm64 and a locally present sha256 ID; runc runtime available. Record that immutable image ID. Require at least 4 GiB disk free and 2 GiB available RAM before this attempt; stop if existing sensors/SLAM are unstable or team declines resource use. Unexpected image change since audit requires review, not automatic use. This is a focused preflight, not repetition of the full audit.

### B. Transfer ONLY these approved files

On host (first writes, only after approval):

```bash
mkdir -p /home/dronehacks/codex_stationary
mkdir "$root"
mkdir "$root/payload" "$root/workspace"
chmod 700 "$root" "$root/workspace"
```

On laptop PowerShell, with outputs from this Phase 1:

```powershell
$out = 'C:\Users\saket\Documents\Codex\2026-11-06\you-are-taking-over-autonomy-software\outputs'
$dest = 'dronehacks@skybeta-02.local:/home/dronehacks/codex_stationary/eab3636-phase2/'
scp "$out\stationary-reviewed-eab3636.tar" "$out\container_build.sh" "$out\verify_container.py" "$out\SHA256SUMS.stationary-phase2" $dest
scp 'C:\dhdl\obj\dronehacks-course-v3.obj' "${dest}course.obj"
```

On host, stop on ANY nonzero result:

```bash
cd "$root"
sha256sum -c SHA256SUMS.stationary-phase2
tar --no-same-owner --no-same-permissions -xf stationary-reviewed-eab3636.tar -C payload
cp course.obj container_build.sh payload/
find payload -type l -print  # expected empty
find payload -type f | sort > transferred-files.txt
chmod -R a+rX payload
```

Expected: all four checksum entries OK (archive, course and two controls; no self-hash). Do not extract if checksum validation fails. The local archive is checked for regular files/directories and relative safe paths before approval. Payload contains source `ros2_ws/src/{dh_core,dh_autonomy}`, the checker, course and build control; no organizer data is edited.

### C. Create a STOPPED sandbox; inspect it before start

```bash
cd "$root"
owner="$(id -u):$(id -g)"
test "$(id -u)" != 0
docker create --name "$name" --pull=never --runtime=runc \
  --network none --ipc private --read-only --cap-drop ALL \
  --security-opt no-new-privileges:true --user "$owner" \
  --cpus 1 --memory 1536m --memory-swap 1536m --pids-limit 192 \
  --restart no --workdir /audit_ws \
  --tmpfs /tmp:rw,nosuid,nodev,noexec,size=268435456 \
  --mount "type=bind,src=$root/payload,dst=/reviewed,readonly" \
  --mount "type=bind,src=$root/workspace,dst=/audit_ws" \
  --entrypoint /usr/bin/env "$image" \
  -i PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  HOME=/audit_ws/home ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 \
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOG_DIR=/audit_ws/log/ros \
  PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  /bin/bash --noprofile --norc /reviewed/container_build.sh --approved-isolated-build
docker inspect "$name" > container-before-start.json
python3 verify_container.py "$root" "$image" "$owner" < container-before-start.json
```

Expected verifier PASS, container still `created`. Do not start if verification fails or if UID cannot read the existing image's /opt/ros dependencies. No GPU/device requests, host networking/IPC/PID, privileged mode, host sockets, ports, organizer workspaces or `/root` mounts are permitted. Extra image-declared volume mounts also cause rejection. Do not fix an access/import failure by switching to root or mounting the running stack without renewed review.

This uses the **same local immutable image** with its entrypoint replaced; it never commits/builds/updates the image. Only a new writable workspace, bounded tmpfs and Docker's new-container metadata/logs are written. Read-only rootfs and nonroot UID prevent package/config mutation. The only network interface must be loopback; IPC and /dev/shm are private, so Fast DDS shared-memory transport cannot reach the live stack. No serial/USB/GPU devices, XRCE transports, Docker socket or host filesystem-control channel are exposed. The clean `env -i`, explicit Humble source and new working directory exclude inherited DDS peer/profile variables, overlays and autostart scripts. Domain77/localhost-only are additional defenses, not the isolation boundary. Docker/kernel isolation is assumed; this is not a claim of protection against a compromised Docker daemon/kernel.

CLI references reviewed: [Docker container options](https://docs.docker.com/reference/cli/docker/container/run/) and [colcon test options](https://colcon.readthedocs.io/en/released/reference/verb/test.html). Installed target versions must still accept the proposed flags; an unsupported option is a stop, not permission to weaken isolation.

### D. Run bounded build and synthetic tests, then STOP

```bash
docker start "$name"  # only after verifier PASS and Phase 2 approval
docker logs --follow "$name"  # follow is read-only; may be interrupted without stopping container
docker inspect "$name" --format '{{json .State}}'
docker stats --no-stream "$name" skybeta-stack
```

Use a second host shell to inspect `docker stats --no-stream` every 10 seconds during execution and observe the existing camera/VO health without starting new providers. Record samples locally. If existing stack health worsens, host available RAM drops below 1 GiB, sandbox OOM occurs, or total run exceeds 10 minutes, abort only this named sandbox using `docker stop --time 10 "$name"`. Resource caps/timeouts are deliberately conservative; do not silently expand them or weaken tests/timeouts to pass.

The reviewed `container_build.sh` performs, in order:

1. Asserts domain77, localhost-only, loopback-only network and nonroot identity. Sources only `/opt/ros/humble/setup.bash`, records versions, verifies build/test imports. Missing dependencies fail before tests; no installation is attempted.
2. `colcon list` must identify exactly the two packages; builds them sequentially to `/audit_ws/{build,install,log}`. Imports installed modules without constructing nodes.
3. Runs both package test suites sequentially with a 300s bound. The existing real ROS tests publish **synthetic** mono images/CameraInfo, TF, test odometry and `/dh/start` only inside this sandbox. They launch no camera/SLAM/flight interface/agent. They use normal mission with explicit dry-run true, not a physical backend. VO checks disabled in these synthetic fixtures do not change the reviewed live profile.
4. Requires colcon success, actual JUnit evidence of both named DDS/OpenCV and dry-run graph cases, and no skips/failures/errors. Skipping the ROS tests is a failure of this phase, not qualification.
5. Starts installed **stationary_mission** in a dedicated process group, with deliberately false dry_run and px4 backend parameters but only test sensor topic settings. No `/dh/start` is sent in this extra check. The read-only graph checker retries only incomplete discovery; forbidden endpoints or exceptions fail immediately. Requires positive mission/dry publisher evidence, stops only this process group, then exits.

Expected: stopped/exited container with ExitCode0, OOMKilled=false, all JUnit suites pass including `test_commands_only_on_dry_run_topic` and `test_real_images_clearance_and_isolation`, positive stationary graph PASS and final `PHASE 2 SYNTHETIC CHECKS PASSED; STOP BEFORE LIVE SENSOR STARTUP`. Build completion alone is insufficient. A failed constructor, incomplete graph, missing checker evidence, unexpected node/transport, skipped test, timeout or nonzero exit is failure/incomplete; preserve logs, report and stop.

Collect state/results without modifying the running stack:

```bash
cd "$root"
docker inspect "$name" > container-after-test.json
docker logs "$name" > container.log 2>&1
tar -czf phase2-results.tar.gz container-before-start.json container-after-test.json \
  container.log transferred-files.txt workspace
```

Then laptop `scp dronehacks@skybeta-02.local:/home/dronehacks/codex_stationary/eab3636-phase2/phase2-results.tar.gz <approved-local-output-path>`. Record exact commands/results, update AUTONOMY_STATUS.md and own shared context, commit/push to saketh/autonomy only after fetch/divergence check. **STOP for Phase 3 approval. No live sensor node has been started by this phase.**

## Changed files, rollback and failure handling

Phase 1 changes only this plan, status/context/rules and two offline control tools; reviewed autonomy source is unchanged. Local deliverables include pinned archive, controls, checksum manifest, plan and local validation evidence. No remote files changed in Phase 1.

Phase 2 would create only `$root` (uploads, payload, checksums, workspace build/install/log/home/evidence and results) plus the uniquely named Docker container, its private writable metadata/logging and temporary tmpfs. No persistent services, installed host/container packages, images, firmware, launch configuration or existing source/workspaces change. ROS nodes in the sandbox create only isolated DDS endpoints and temporary build/log files. Installed launch entrypoints remain inside the new workspace, not added to live startup.

Rollback: stop only `codex-autonomy-eab3636-phase2` with `docker stop --time 10` if still running, save inspect/log/evidence, then `docker rm codex-autonomy-eab3636-phase2` after verifying its labels/name/image/mounts against recorded JSON. Leave `$root` as inert evidence by default. Do not recursively delete unknown/computed directories or use prune/compose-down/pkill/service restarts. If later authorized to remove this attempt's data, first verify `realpath "$root"` equals exactly `/home/dronehacks/codex_stationary/eab3636-phase2`, inspect contents and remove only this dedicated directory. No rollback command touches skybeta-stack, team workspaces or the image. A paused/failing attempt is never resumed with altered permissions or broader mounts without review.

## Phase 3 and 4 approval boundaries

Phase 3 is **not authorized** by approval of Phase 2. The isolated sandbox cannot access live data and must not be repurposed by adding host networking/devices. Submit a new exact live-perception invocation and inspect its publisher/client/transport plan after synthetic results. Use only reviewed stereo/line/localizer nodes as appropriate, not mission, driver, SLAM, bridge or flight interface. Reconfirm live camera/VO health, unit02 mono topics, measured TF/CameraInfo fallback and 5ms capture pairing. Measure complete processing rate, received/processed/rejected counts, stamp-based latency p50/p95/max, odometry ages, CPU/memory and frame drops under load. No 400ms odometry-threshold relaxation.

Physical depth accuracy needs user-supplied measured target distances/geometry and stable placement. Proposed board trials at 0.5, 1, 1.5 and 2m are **not measurements**. Before trials agree distance origin (base_link/optical), target dimensions, illumination and acceptance tolerance (proposed max(0.05m,10%)); record actual tape measurements and finite/unknown fractions. Blank/invalid/stale inputs must produce unknown/hold, never false clear. Use recorded-data/isolated injection for stale/tracking-loss cases; do not stop/reconfigure existing camera/SLAM services. Floor-line testing requires a suitable known reference and measured camera height. Report omissions/failures honestly and STOP for Phase 4 approval.

Phase 4 is **not authorized** now. Before its live launch review all resolved endpoints and absence of relay paths, duplicate node names/remaps, and confirm stationary entrypoint. Use only reviewed jetson_02 config, keep clearance and VO health required. On a bench, leave `align_mode=start` and send no start command: do not use `first_odom` to invent a course pose. Course alignment and calculated trajectory commands require explicit placement/heading/height evidence or an agreed non-course test frame plus a separate specific `/dh/start` approval. Require graph PASS with dh_mission and dry publisher positively identified and no physical clients/publishers, then measure live health/planning exclusively on `/dh/dry_run/*`. Shut down only owned test process groups. No physical command publication, arming/disarming/mode changes/flight operation is included in any phase.

## Phase 1 validation and stopping point

Local safety regressions rerun under task-local numpy1.26.4/OpenCV4.10: **16 passed, 0 failed**, including stationary false/px4 and false/skybeta overrides, incomplete graph rejection, measured calibration and delayed camera pose. This is fake-bus/local validation, not actual ROS or Jetson execution. Local Bash `-n` passes; Python3.10 grammar checks pass; the container verifier accepts the approved fixture and rejects 17 unsafe variants; all 51 archive members are safe relative regular files/directories and reviewed file bytes match current unchanged package source. The first sandboxed Git Bash syntax-check attempt failed to create a Windows signal pipe; the same syntax-only command passed with local execution permission. No remote command was involved. Target Docker/colcon/runtime checks remain pending. Previous 47 unit/12 fake-bus/12 stereo/14 line results are preserved audit evidence, not newly executed hardware tests.

No SSH connection, remote write, build, feature-node launch or hardware operation was executed in Phase 1. Approval requested only for the concrete Phase 2 sandbox above. User's explicit staged gate is the reason for stopping; elapsed time or earlier read-only SSH permission does not authorize deployment.

PHASE 1 COMPLETE — AWAITING APPROVAL FOR ISOLATED JETSON BUILD.
