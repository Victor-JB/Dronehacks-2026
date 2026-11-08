# Phase 2 results — FAIL; export and cleanup complete — 2026-11-07

The actual Orin Nano build succeeded, but **Phase2 did not pass**. Colcon selected unittest for dh_autonomy and executed zero tests. The explicit required-JUnit-case gate rejected the result. No synthetic DDS integration or stationary graph check ran. No retry, source fix, privilege/resource expansion, Phase3/4 or physical command occurred. After verified export, the user-authorized narrow cleanup removed the dedicated sandbox and workspace. SSH is closed.

## Source, transport and actual environment

- Repository/branch: Victor-JB/Dronehacks-2026, saketh/autonomy. Reviewed controls: c54471a5cab87df98c85fd0f7cc9140b7b56c437; autonomy/source archive: eab363673fbb6b5272ab1d54115e77499cba22ca, unchanged. Phase2 changes in Git are documentation only. Latest team-main context read at afc008f; other teammate files/worktrees preserved.
- Original .local SSH retry still failed name resolution. User explicitly supplied192.168.68.77; connected using HostKeyAlias=skybeta-02.local with StrictHostKeyChecking=yes. Saved host key matched, remote hostname was skybeta-02, model Orin Nano Super reference kit, ARM64/kernel5.15.185-tegra/L4T36.5.0. User-reported props-off/no flight battery/independent power premises retained; no physical condition was independently observable over SSH.
- Host Python3.10.12; Docker29.9.1; systemd cgroupv2/runc support. Existing image `skybeta-team:0.3.3`, immutable ID `sha256:439804b78f3365d2d84367dd1e8c2a5a28b399f7bf86c0784c3d293e6b232286`, ARM64, no image-declared volumes. Existing stack ID `87bc62da53fa43b2b1d078b5b239335cc77ddf1733f931855ff8895294adcbe1`.
- Target dependency imports passed: numpy1.26.4/OpenCV4.5.4/rclpy/message_filters/tf2_ros/message types/launch/pytest6.2.5/launch_testing/launch_testing_ros. No dependency/system install or image rebuild occurred.

## Commands and execution sequence

Exact reviewed scripts remain in phase2_safety_review_v2.zip. The full exported workspace contains the executed source/control files, source archive, manifests, Docker inspect configuration and colcon command/event logs. `phase2-command-evidence.json` additionally preserves terminal exchanges (ANSI/line wrapping included), including directory creation, Docker-create parameters, monitor invocation, observations, export and cleanup. Passwords are not recorded. Command highlights below are actual executions, not a future procedure.

1. Local package verification: ZIP SHA256 `958c4bbc32edb5d6b3565c19cf369961a073f4b1da87e4f86a2821f7e93368a8`, all54 package hashes and7 payload hashes, exact c54471a controls and unchanged TAR validated. The old local helper's static organizer-permission-blocked message was obsolete; current organizer permission was explicitly confirmed.
2. SSH transport used `ssh -T -o HostKeyAlias=skybeta-02.local -o StrictHostKeyChecking=yes -o ConnectTimeout=8 dronehacks@192.168.68.77 "env HISTFILE=/dev/null PYTHONDONTWRITEBYTECODE=1 bash --noprofile --norc"`. Read-only checks inspected hostname/model/kernel/L4T,id,free/df,host Python, Docker versions/runtime/flags, stack state and recent logs, and confirmed no workspace/container collision. A Go-template query for optional image.Config.Volumes failed; raw image JSON showed the field absent, without changing configuration. Existing repeated min-altitude clamping warnings were recorded; no new organizer action was taken.
3. Created only `/home/dronehacks/codex_stationary/eab3636-phase2/{payload,workspace}` and its containing directory as needed; chmod700 on attempt/workspace. SCP transferred exactly the approved archive/course/controls/checksums to that attempt. `sha256sum -c SHA256SUMS.stationary-phase2` passed before extraction; tar --no-same-owner --no-same-permissions, cp, source inventory and payload read permissions followed the reviewed plan.
4. Created the stopped sandbox with exact v2 arguments: network none; private IPC/cgroup/PID; runc; UID:GID1000:1000; read-only rootfs; all capabilities dropped; no-new-privileges; oneCPU/1536MiB/no extra swap/192PIDs; exact read-only payload and writable workspace mounts; no devices/GPU/socket/ports;256MiB noexec tmpfs; bounded Docker logs; no inherited health command/restart/alternate init; PID1 GNU timeout TERM600s/KILL5s fallback and clean environment. The target verifier passed before startup.
5. Launched reviewed host supervisor via nohup, using the verified full sandbox ID. It checked initial headroom/organizer identity, created the false-permission lease, started only that ID, confirmed effective cgroup memory/OOM telemetry and renewed build permission. The container guard's PID1/cmdline/GNU identity and graceful/forced timeout probes passed as prerequisites to work. The600s expiry itself was not reached or tested in this run.
6. Actual build: `colcon build --base-paths /reviewed/ros2_ws/src --packages-select dh_core dh_autonomy --executor sequential --event-handlers console_direct+`. Both packages built/installed in the isolated workspace; installed module imports and stationary hard-lock inspection passed.
7. Actual tests: `timeout --signal=INT --kill-after=10s 300s colcon test --base-paths /reviewed/ros2_ws/src --packages-select dh_core dh_autonomy --executor sequential --event-handlers console_direct+ --pytest-args -v`, then `colcon test-result --verbose` and explicit JUnit-required-case validation. The dh_core log invoked pytest; the dh_autonomy command log proves `/usr/bin/python3 -m unittest -v` and `Ran 0 tests`. Required DDS names were absent, causing RuntimeError and sandbox exit1. Host monitor detected failure and completed its sandbox-only stop/final evidence path. No retry was attempted.
8. Collected inspect/log/monitor/resource/workspace evidence into initial phase2-results.tar.gz, then a complete workspace export with per-file hash/type/ownership inventory after the additional cleanup authorization. Verified both remote and local full-archive SHA256 and all manifest members before removing anything.
9. Narrow cleanup revalidated canonical path/UID1000, every exported entry's ownership/mode/hash/link target, absence of nested mounts, exact container ID/name/label/image/config/mount identity and stopped state, inactive monitor and unchanged organizer identity. Docker's OomKillDisable false/null representation was normalized to enabled OOM killing; all other immutable fields matched. `docker rm 60357d35ddd41b563a96b06206168cf64e1ab1831b588bdfa41ecab3d8e77e1b` removed only that stopped resource, followed by symlink-resistant `shutil.rmtree` of the verified literal attempt path. Postchecks confirmed no target directory, no sandbox ID/name and no monitor, with organizer unchanged. No prune, broad cleanup, image deletion, organizer/team modification or deletion outside the attempt directory occurred. SSH session closed.

## Acceptance criteria and measured results

| Check | Actual result |
|---|---|
| Overall | **FAIL: mandatory DDS tests missing** |
| dh_core build | PASS,4.53s reported package time |
| dh_autonomy build | PASS,4.45s reported package time |
| Total colcon build |9.76s |
| Native ARM/Humble core unit tests |17 passed; pytest2.07s |
| Total colcon test phase |6.34s; dh_core4.55s, dh_autonomy1.04s |
| Real-DDS graph test | NOT EXECUTED |
| Real-DDS/OpenCV perception test | NOT EXECUTED |
| Separate installed stationary isolation checker | NOT EXECUTED; preceding gate stopped execution |
| Docker isolation configuration | PASS on target before startup |
| Timeout mechanism | Runtime identity and short graceful/forced probes passed; full600s expiry not exercised |
| Sandbox runtime |26.520525s; exit1; OOMKilled=false |
| Peak sampled sandbox cgroup memory |216,821,760bytes =206.78MiB, not continuous high-water measurement |
| Minimum sampled available host RAM |4,574,392,320bytes =4.26GiB |
| Minimum sampled free disk |144,138,170,368bytes =134.24GiB |
| OOM/allocation events sampled |0 |
| Peak sandbox CPU | NOT CAPTURED during active work; persisted docker-stats sample was after exit. OneCPU quota is a limit, not a measured peak |
| Existing organizer container | Same ID,image,PID3256,start time and restart count0 before/during/after; running/not paused/not OOM |
| Camera/SLAM health | NOT independently monitored; no approved health source. Container liveness is not sensor qualification |
| Final sandbox/monitor/workspace | Removed/inactive/removed after verified evidence export |

## Failure interpretation and next review

The observed failure is test discovery/runner selection, not a demonstrated planner, ROS contract, timeout or memory-limit failure. Source inspection shows dh_autonomy/setup.py lacks the pytest tests_require registration present in dh_core; that is consistent with colcon selecting unittest. This is a source-supported explanation, not a tested fix. Review explicit pytest/launch-testing registration before preparing a new pinned package and retry. Do not weaken the required-case gate or count zero tests as success. No source fix or second hardware attempt was made. Resource caps did not need or receive an increase.

Live camera processing, real sensor accuracy/throughput, actual DDS mission/perception integration and stationary mission isolation still await successful synthetic ROS testing and separate later approvals. Phase3/4 were not started. Nothing here qualifies flight or physical control compatibility.

## Local exports, verification and exact remote write inventory

- Complete export: `outputs/phase2-complete-export.tar.gz`,2,850,983bytes, SHA256 `8ffc8c6898a6d013e6f141209f6e732771d14426c1b69db4167e06a5f5aec4f8`. Full gzip CRC, TAR membership,239 file hashes/sizes,76 directories and5 symlink records verified against the remote manifest. Human-readable important logs/JSON/core JUnit parsed successfully on Windows under outputs/phase2-jetson-evidence. No symlinks were followed during local verification.
- Manifest SHA256 `a856901b017eb7fa3ecdee0b506527ca1fd780f6bcd47aac52922dae97dc88e0`. Exact names/types/modes/UID/GID/hashes/link targets of all320 exported entries are in remote-export-manifest.json. The manifest and complete export archive themselves are the two additional generated files excluded from that inventory to avoid self-reference. The dedicated root is the containing directory. All regular test/source/control/build/install/log data are preserved, including the earlier partial export.
- Remote writes were confined to the approved attempt: eight uploaded files, extracted/copied payload, workspace build/install/log/home/evidence, image/container/stack snapshots, source/file inventories, run-token/full-ID files, monitor lock/console/JSONL, read-only lease, CPU observations and two export archives/manifest. Docker maintained only the new sandbox's metadata/logs/private runtime files. Normal SSH/system audit records are outside test evidence and were not erased. The parent `/home/dronehacks/codex_stationary` was not deleted because it is outside the approved removal target.
- Initial export-preparation checks stopped safely on a dict-versus-list metadata parse and the false/null Docker OOM-field representation. No deletion happened on either failure; metadata handling was corrected, a complete export was produced and fully verified before cleanup. No reviewed control or autonomy code was changed to bypass a failure.
- Cleanup confirmed at2026-11-08T02:26:37.555925Z (2026-11-07 local client date). Exact confirmation and command exchanges are retained in phase2-command-evidence.json. Existing organizer processes/workspaces/image were preserved.

The Phase2 attempt and requested cleanup are complete, but **acceptance failed**. Review/fix test discovery and repeat synthetic qualification before advancing to Phase3.
