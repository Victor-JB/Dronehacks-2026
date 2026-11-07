# Stationary deployment plan v2 — safety hardening — 2026-11-07

**PROPOSED ONLY. Phase 2 is not authorized. No SSH, remote writes/builds or container startup occurred during hardening.** Approval must cover this revised plan and scripts. Phases 3 (live perception) and 4 (stationary mission) each need separate later approval. Physical commands, including zero commands, arming/disarming, mode changes, motors, flight and PX4/system changes remain prohibited.

## Pinned scope and source review

Repository Victor-JB/Dronehacks-2026, branch saketh/autonomy. Autonomy source remains **eab363673fbb6b5272ab1d54115e77499cba22ca**. No dh_core/dh_autonomy/planner/flight-control code changed. Latest team-main context read: 9186743; other four teammate context files preserved. Preserve the existing image, services, `/home/dronehacks/skybeta`, `/home/dronehacks/team_ws`, `/root/skybeta_ws` and `/root/team_ws`.

The stationary entrypoint calls StationaryMissionNode; force_dry_run=True always selects DryRunBackend despite false/px4 parameters. The two synthetic ROS tests use normal mission with explicit dry-run true. Both paths create only dry command outputs and measurement/health topics, never physical backend constructors. The additional stationary constructor check sends no start command. ROS remaps are forbidden: the startup endpoint checker runs after endpoint creation and is not a sandbox by itself. Reviewed imports/launch start no driver, bridge, agent or physical interface. Full project dependency closure is included in the immutable source TAR and readable ZIP source copy.

Existing dependencies: Humble, ament_python, colcon/ROS/Python/pytest extensions, setuptools, numpy, OpenCV, rclpy, message_filters, tf2_ros/static_transform_publisher, std_msgs/std_srvs/geometry_msgs/nav_msgs/sensor_msgs/builtin_interfaces, ament_index_python, launch/launch_ros, pytest, launch_testing/launch_testing_ros. Live VO status additionally requires isaac_ros_visual_slam_interfaces. No raw PX4 backend, vendored px4_msgs, GPU or hardware device is needed. Target imports/build tooling/permissions still require actual verification; missing dependencies stop the attempt, never trigger installs/upgrades.

## Controls and automatic protections

1. **Container deadline:** `/usr/bin/timeout --signal=TERM --kill-after=5s 600s` is the container entrypoint/PID1. The verifier checks its exact executable, arguments and absence of an alternate init. TERM occurs at 600s from container startup, with forced KILL fallback five seconds later. This bounds build, tests, graph calls and cleanup independently of SSH, monitor or Python progress. The separate 300s test timeout is retained. No auto-remove or restart: evidence remains.
2. **Runtime verification before work:** container_guard.py verifies `/proc/1/exe` and cmdline, GNU coreutils identity, a graceful timeout probe and a TERM-ignoring forced-kill probe. Failure/missing mechanism prevents build/import/test startup. Only harmless timeout probes run before this verification. Under the trusted Linux PID namespace, PID1 exit kills remaining descendants, including processes that detached into other sessions; timeout process-group cleanup alone is not the guarantee. [GNU timeout](https://www.gnu.org/software/coreutils/manual/html_node/timeout-invocation.html), [Linux PID namespaces](https://man7.org/linux/man-pages/man7/pid_namespaces.7.html).
3. **Automatic host monitor:** monitor_phase2.py verifies the stopped sandbox first, then monitors host RAM/disk, sandbox cgroup memory/allocation/OOM events, Docker OOM state, elapsed time and organizer container identity/liveness/restart count/start time/PID. A missing/unreadable/unsupported required metric is failure, not healthy. It records JSONL samples about every2s, with Docker RPCs individually bounded to4s.
4. **Thresholds:** starting headroom >=2GiB available RAM and4GiB disk; abort below1GiB available RAM, below2GiB disk, sandbox memory at90% of1536MiB, any cgroup OOM/allocation-failure event, wrong effective memory limit, Docker OOM, elapsed>=600s, organizer stop/pause/OOM/replacement/restart, or telemetry failure. The monitor never changes organizer processes.
5. **Fail-closed monitor lease:** host writes atomic `/payload/monitor_lease.json`, mounted read-only inside the sandbox. Build permission stays false until a complete healthy post-start sample, including effective cgroup limits. The guard polls every0.25s and rejects expired (>12s), malformed, revoked or wrong-token leases. If monitoring stops on SSH logout, crash, kill or blocked RPC, the lease expires and terminates the sandbox work. No promise that nohup survives every host login policy; lease loss is the independent fail-closed response. Guard cleanup has bounded TERM/KILL waits; the outer deadline still applies even if the guard hangs.
6. **Exact cleanup identity:** after verification the monitor retains the full64-hex sandbox ID and run-token label. All start/stop/kill mutations use that ID, never a lookup by unchecked name. Stop uses5s grace and an8s RPC bound, then a4s KILL RPC fallback. If Docker is unavailable, the lease/deadline do not depend on it. No other container is a cleanup target.
7. **Camera/SLAM health:** no approved read-only health-file path is currently established. Default logs explicitly say sensors NOT MONITORED; organizer liveness is not sensor health. Optional `--health-file` accepts only an explicitly approved existing JSON status source with `stamp_unix`, boolean `camera_ok`/`slam_ok`; age>5s, future>0.1s, missing/invalid/false aborts. Do not invent a producer or start/modify existing ROS services to provide it. Review/approve actual source semantics before using this option.
8. **Isolation unchanged:** network none, private IPC/PID/cgroup namespaces, no hardware/GPU/host sockets, no organizer mounts, nonroot, runc, all capabilities dropped, no-new-privileges, read-only rootfs, exact two mounts, bounded tmpfs/CPU/memory/PIDs and clean environment. Domain77/localhost-only supplement the boundary. No Docker socket is mounted in the sandbox. The host monitor uses only the user's pre-existing Docker access.

Safety-critical controls use explicit validation exceptions, not assert. Verification remains effective under python -O. The verifier rejects wrong namespaces, mounts/propagation, root/device/socket/extra-capability access, changed limits/restarts/logging, image environment/label drift, different entrypoint/deadline/arguments and inherited health commands. Unsupported target Docker flags/configuration cause a stop, never relaxed settings.

Assumptions: trusted image/Docker daemon/Linux kernel and normal scheduling/signal delivery. This is not real-time certification or a VM security attestation. No isolated container has yet been run on the Nano. Cgroup read access, GNU timeout behavior, Docker version flags and installed imports remain runtime gates; they have not been established by local mocks.

## Local review payload and integrity

`phase2_safety_review_v2.zip` contains this plan, all five control/test files, revised checksum manifests, change/review notes, test logs, original immutable source TAR, course.obj and expanded project source. No credentials or image binaries are included. Runtime `/opt/ros/humble/setup.bash`, generated install hooks and system executables/plugins are external dependencies, not captured image snapshots.

- `stationary-reviewed-eab3636.tar`: SHA256 **da31d4b63ed05099d2be592d292c05cc4be142a699a96dbe15387f68db3d4f70** (unchanged).
- `course.obj`: SHA256 **e610bcf9666422e2a0a78cb46f542fc8c4d5a7ef9b15f6123f1abe40f5b8e9e1** (unchanged synthetic fixture).
- Revised `SHA256SUMS.stationary-phase2` contains those inputs and the five exact control/test files. Package-wide manifest additionally covers documentation/evidence/readable source. Original ZIP/manifest remain preserved as historical v1 artifacts.

## Proposed Phase 2 commands — do not execute before explicit approval

Approval must include new host directory `/home/dronehacks/codex_stationary/eab3636-phase2`, transfer, stopped container creation, host supervisor and isolated synthetic startup/tests. Confirm props/battery absent, independent sensor power and organizer controller ownership. No sudo, package install or service change. Collision/ambiguity is a stop. Use one host shell for variables; stop on every failed command. The supervisory invocation, not a manual docker start, is the only approved start path in this proposal.

### A. Focused read-only preflight, then approved writes

```bash
root=/home/dronehacks/codex_stationary/eab3636-phase2
name=codex-autonomy-eab3636-phase2
test "$HOME" = /home/dronehacks
test ! -e "$root"
test -z "$(docker ps -aq --filter name=^/${name}$)"
id
df -h /home/dronehacks
free -h
docker ps --no-trunc
image=$(docker inspect --format '{{.Image}}' skybeta-stack)
docker image inspect "$image" --format '{{.Id}} {{.Architecture}}'
docker info --format '{{json .Runtimes}}'
```

Expect running organizer stack, arm64 local immutable image ID, runc and adequate headroom. Review unexpected image drift. Only after approval and successful collision checks:

```bash
mkdir -p /home/dronehacks/codex_stationary
mkdir "$root"
mkdir "$root/payload" "$root/workspace"
chmod 700 "$root" "$root/workspace"
```

Laptop PowerShell, from the delivered v2 directory (not historical v1 controls):

```powershell
$out = 'C:\Users\saket\Documents\Codex\2026-11-06\you-are-taking-over-autonomy-software\outputs\phase2_safety_review_v2'
$dest = 'dronehacks@skybeta-02.local:/home/dronehacks/codex_stationary/eab3636-phase2/'
scp "$out\stationary-reviewed-eab3636.tar" "$out\course.obj" "$out\SHA256SUMS.stationary-phase2" "$out\container_build.sh" "$out\container_guard.py" "$out\monitor_phase2.py" "$out\verify_container.py" "$out\test_controls.py" $dest
```

Host, each step must succeed:

```bash
cd "$root"
sha256sum -c SHA256SUMS.stationary-phase2
tar --no-same-owner --no-same-permissions -xf stationary-reviewed-eab3636.tar -C payload
cp course.obj container_build.sh container_guard.py verify_container.py payload/
find payload -type l -print  # must be empty
find payload -type f | sort > transferred-files.txt
chmod -R a+rX payload
owner="$(id -u):$(id -g)"
test "$(id -u)" != 0
token=$(python3 -c 'import secrets; print(secrets.token_hex(16))')
docker image inspect "$image" > image-before.json
```

Checksums must all say OK before extraction; archive contains only reviewed safe relative regular files/directories. No sources from existing workspaces are overwritten.

### B. Create stopped container with exact verified deadline

```bash
cid=$(docker create --name "$name" --pull=never --runtime=runc \
  --network none --ipc private --cgroupns private --read-only --cap-drop ALL \
  --security-opt no-new-privileges:true --user "$owner" \
  --cpus 1 --memory 1536m --memory-swap 1536m --pids-limit 192 \
  --restart no --workdir /audit_ws --no-healthcheck --stop-signal SIGTERM --stop-timeout 5 \
  --log-driver json-file --log-opt max-size=5m --log-opt max-file=2 \
  --label "org.dronehacks.phase2.run=$token" \
  --tmpfs /tmp:rw,nosuid,nodev,noexec,size=268435456 \
  --mount "type=bind,src=$root/payload,dst=/reviewed,readonly" \
  --mount "type=bind,src=$root/workspace,dst=/audit_ws" \
  --entrypoint /usr/bin/timeout "$image" \
  --signal=TERM --kill-after=5s 600s /usr/bin/env -i \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  HOME=/audit_ws/home ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 \
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOG_DIR=/audit_ws/log/ros \
  PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  /usr/bin/python3 -u /reviewed/container_guard.py --token "$token")
printf '%s\n' "$cid" > sandbox-id.txt
printf '%s\n' "$token" > run-token.txt
docker inspect "$cid" > container-before-start.json
python3 verify_container.py --root "$root" --image "$image" --owner "$owner" \
  --token "$token" --image-json image-before.json < container-before-start.json
```

Expect verifier PASS and container still created. Stop on unsupported flags, extra image volumes, mismatched inspected fields or unreadable target dependencies. Do not bypass verifier, use root, add mounts or relax checks. No automatic resource mutation occurs in verify_container.py.

### C. Start only via automatic host supervision

```bash
nohup /usr/bin/env -i PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1 \
  /usr/bin/python3 "$root/monitor_phase2.py" --root "$root" --container-id "$cid" \
  --image "$image" --owner "$owner" --token "$token" --approved-isolated-build \
  > "$root/monitor-console.log" 2>&1 < /dev/null &
```

No optional health-file flag is authorized by default. A separately reviewed existing source may add `--health-file /approved/read-only/status.json`; do not supply an arbitrary command/ROS launcher. The monitor re-verifies the stopped container, acquires a one-attempt lock, snapshots the existing stack, checks host headroom and creates a false-permission lease BEFORE starting only the full verified sandbox ID. Effective cgroup/OOM telemetry must then pass before a true lease permits work. Absent telemetry means no build starts. Monitor may survive SSH loss via nohup; if it does not, stale lease stops the sandbox. No organizer/user service changes or linger configuration are needed.

Observation only, optional while supervisor runs:

```bash
docker logs --follow "$cid"
tail -f "$root/monitor.jsonl"
```

Neither observation command is needed for enforcement. Never manually docker-start the sandbox to bypass a failed monitor. Expected build actions remain sequential package imports/build, both real-DDS/OpenCV synthetic test suites with no skips, then the forced-stationary constructor/graph test. Captured phase2 success requires exited sandbox ExitCode0/OOMKilled=false, unchanged organizer identity, healthy recorded resources, both named JUnit test cases and positive stationary graph isolation. Any failure/deadline/OOM/incomplete evidence is not a pass. Live camera/SLAM processing is still not qualified.

### D. Collect evidence and stop before live sensing

The monitor retains source/workspace, per-sample JSONL, before/final inspect snapshots and logs; no auto-remove. If the monitor is killed abruptly, original Docker logs and workspace evidence remain, even if final JSON snapshots were not written. Later approved read-only collection:

```bash
cd "$root"
docker inspect "$cid" > container-collected.json
docker logs "$cid" > container-collected.log 2>&1
tar -czf phase2-results.tar.gz workspace payload/monitor_lease.json \
  monitor-console.log monitor.jsonl image-before.json container-before-start.json \
  stack-before.json container-collected.json container-collected.log \
  transferred-files.txt sandbox-id.txt run-token.txt
```

If startup fails before a named file exists, preserve available files and disclose missing evidence; do not manufacture success. Copy results to approved local outputs, document actual commands/metrics/failures in AUTONOMY_STATUS.md and own context, commit/push only saketh/autonomy after fetch/divergence check. STOP for separate Phase 3 approval.

## Writes, mutation scope and rollback

Host writes: dedicated mkdir/chmod/upload/extraction/copy, source inventory, image/inspect JSON, run IDs/tokens, monitor.lock, atomic lease replacement, monitor logs and results archives; sandbox build/install/log/home/evidence through the sole writable bind; private tmpfs/shm; Docker metadata/bounded logs. Normal SSH/account/Docker/system logging can cause incidental platform writes. No host packages, image content, services, existing source, firmware, camera config or PX4 settings change. No Docker socket/device mount or extra Docker privilege is introduced.

Docker mutations: create (stopped new container), start (monitor only, verified full ID), stop with5s grace and kill fallback (only same ID). Observer commands inspect/ps/info/stats/logs do not reconfigure resources. No build/pull/commit/prune/compose-down command is proposed. There is no automatic removal. Optional rollback removal requires rechecking recorded full ID/token/image/mounts, saving evidence, then `docker rm "$cid"` only on the stopped sandbox; never target skybeta-stack. Retain dedicated host directory unless its exact resolved path is verified and deletion separately authorized. A failed attempt/lock is never silently reused.

## Validation and remaining boundaries

Local mock safety suite: normal Python and python -O; verifier acceptance plus unsafe drift cases, deadline/lease loss, probe failure, bounded process/Docker cleanup, RAM/disk/OOM/memory/runtime/stack-restart policies and optional health-source failures. Logs and exact results are included in the v2 package. Bash syntax, Python3.10 grammar and unchanged source/archive checks are also included. Initial restricted-temp fixture tests hit Windows permission errors; local permission-enabled retry passed without any remote operation. These are local mocks/syntax checks, not actual Docker/Linux PID-namespace/cgroup/Jetson or ROS execution. Target runtime probes enforce the remaining prerequisites before build/tests can start.

Phase 3 and Phase 4 remain separately gated. Never attach this sandbox to host networking/devices for live tests. Do not weaken the400ms odometry gate, change sensor services or invent course alignment. Live perception needs measured target distances and timing/CPU/memory evidence; full stationary mission needs separately reviewed physical command isolation and specific start approval. Current task stops after local safety hardening and review ZIP delivery.

PHASE 2 SAFETY HARDENING COMPLETE — AWAITING AUTHORIZATION.
