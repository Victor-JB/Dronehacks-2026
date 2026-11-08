# Corrected Phase 2 retest — frozen baseline, separate approval required

This is a local-only correction of the demonstrated zero-test runner failure. Do not SSH or run a retest until the user separately authorizes this candidate. The source commit is supplied as the exact40-hex `source_commit.txt` in the generated package; use that file, never the moving saketh/autonomy head. Later CAD-free navigation work is excluded from this frozen candidate.

## Root cause and correction

Target command evidence proved dh_autonomy executed `/usr/bin/python3 -m unittest -v` and zero tests, whereas dh_core used pytest. dh_autonomy lacked `tests_require=["pytest"]`; dh_core had it. Actual Colcon0.20.1 `PytestPythonTestingStep.match` locally returns false on the former metadata and true on corrected metadata. [Colcon documents this default-selection rule](https://colcon.readthedocs.io/en/released/reference/verb/test.html). Added that registration only to package setup; existing launch-test semantics are unchanged. Existing test_depend pytest/launch_testing/launch_testing_ros entries and installed target plugins were already sufficient imports. No additional driver, estimator or flight-interface launch is added.

Local real-pytest --collect-only with **import-only ROS stubs** finds both required methods and the shutdown case. This is not Humble launch/DDS execution. Real ROS is unavailable on this Windows environment. The future container must prove collection and actual execution using real installed plugins before qualifying the correction.

## Fail-closed retest stages

1. Exact reviewed source/control/course hashes checked before extraction.
2. Same nonroot, no-network/device/GPU/socket, private IPC/PID/cgroup, read-only-root, two-bind-mount isolation. Same1CPU/1536MiB/192PIDs,600s+5s PID1 GNU timeout, mandatory timeout probes, host resource/organizer monitoring and12s read-only lease. No weakening or changes to organizer code/services/image.
3. Build only dh_core/dh_autonomy; source/install imports as before.
4. Real pytest --collect-only of both ROS launch-test files; `check_test_evidence.py --stage collection` rejects zero/missing names.
5. Colcon explicitly uses `--python-testing pytest --return-code-on-test-failure`. The original real tests and isolation checks execute unchanged, sequentially inside domain77/network-none.
6. Gate reads actual dh_autonomy `log/latest_test/.../command.log` and rejects unittest or absent `python3 -m pytest`; requires nonempty dh_autonomy JUnit with both expected cases and no skip/error/failure. It does not infer execution from collection or aggregate core-test counts.
7. Dedicated stationary executable constructor/graph test remains after that gate. Successful checker produces a return-code marker and copied positive log. `--stage complete` rejects absent/nonexecuted/failed checker or missing positive dry-publisher evidence. No earlier stage can print final success.
8. Automatically export all evidence to Windows, verify gzip/TAR membership and all per-file hashes, then remove only the verified stopped container and canonical owned directory. Any export/hash/readability/identity error prevents deletion; preserve remote evidence and report. No prune or outside-directory cleanup.

## Payload generation and freeze

`tools/hil/prepare_phase2_retest.py --commit <full-source-sha> --output <new-local-directory>` reads committed files with git-show/git-archive. It does not copy the moving checkout. Archive is new, named by the frozen commit, and has fresh SHA256. The old eab3636 TAR/hash is explicitly rejected. The old course remains **only a synthetic regression fixture**, not the final CAD-free navigation model.

Generated package includes this plan, original isolation plan for the unchanged Docker flags, all control/export/checker code, new source TAR, course.obj, source_commit.txt and complete/payload checksum manifests. Exact commit/archive/package hashes are in the generated receipt. Preserve historical failed-run evidence and previous ZIPs.

## Exact proposed procedure after separate approval

Use the previous reviewed Docker-create flags from STATIONARY_DEPLOYMENT_PLAN.md, unchanged. The legacy directory/name are retained solely to keep the reviewed verifier stable: `/home/dronehacks/codex_stationary/eab3636-phase2` and `codex-autonomy-eab3636-phase2`. They must not exist; the prior run's resources were verified removed. Their labels do not select source; new source_commit.txt and hashes do.

1. Reconfirm approved stationary/props/battery/power conditions, target key/hostname, organizer permission/healthy existing stack, RAM>=2GiB/disk>=4GiB, Docker/cgroup compatibility and no collisions. Do not assume earlier preflight remains current. No automatic source/permission/resource workaround.
2. Create the exact dedicated directory with payload/workspace as previously reviewed. Transfer **this candidate's** source TAR, course.obj, source_commit.txt, SHA256SUMS.stationary-phase2 and all delivered `*.py`/`*.sh` controls to it. No old ZIP/source/manifest.
3. On target, with `root` set to the exact approved path:

```bash
cd "$root"
sha256sum -c SHA256SUMS.stationary-phase2
pin=$(cat source_commit.txt)
tar --no-same-owner --no-same-permissions -xf "stationary-reviewed-${pin:0:12}.tar" -C payload
cp course.obj container_build.sh container_guard.py verify_container.py check_test_evidence.py payload/
find payload -type l -print  # must be empty before build
find payload -type f | sort > transferred-files.txt
chmod -R a+rX payload
```

Every command must succeed. Then run the exact stopped-container creation and verify_container.py invocation from the reviewed isolation plan, and start only through monitor_phase2.py with the full verified ID. No direct docker-start bypass. The new build script handles collection/execution/final isolation gates automatically. Capture CPU from before startup if a peak is required; the existing monitor logs memory/OOM but does not provide CPU high-water evidence. Missing CPU measurement must be disclosed, never reported as a measured cap.

4. After the monitor/container finish (including failed tests), run the following **on Windows**, from this frozen package's control directory:

```powershell
python finish_phase2.py --target 192.168.68.77 --container-id <verified-full64hex-id> --source-commit <source_commit.txt-full40hex> --output <new-local-evidence-directory> --approved-cleanup
```

Normal SSH authentication is used without password storage; alias skybeta-02.local plus StrictHostKeyChecking=yes pins the known host key. The helper sends reviewed export_cleanup.py over SSH stdin. It refuses a running/unrecognized container, changed organizer identity or wrong source pin/root/ownership. It downloads complete-export.tar.gz, verifies its remote digest, gzip CRC, complete member set, every file hash and symlink record, and writes local verification evidence. Only then can it send the hash-bound cleanup request. Export failures leave the sandbox/files intact; no unverified cleanup call exists. Container failure is preserved, not turned into a successful test result by cleanup.

5. Cleanup matches exact full ID/name/run token/image/config/mounts and stopped state. It rechecks the entire directory against exported hashes, canonical UID1000 ownership, no nested mounts and secure no-follow removal support. It removes only that stopped ID and exact directory. It records a local cleanup receipt and confirms organizer identity/liveness unchanged. Parent directory and OS logs are not erased. No other container/image/workspace is removed.

6. Report actual command logs, test-case counts, all failures/metrics and cleanup receipt; update status/context on saketh/autonomy only. Stop before Phase3/4. Neither collected test names nor a passed build constitutes a successful retest.

## Acceptance and stop criteria

Require both actual real ROS methods `test_commands_only_on_dry_run_topic` and `test_real_images_clearance_and_isolation` to pass without skips; installed stationary checker must actually execute and pass; all runtime isolation/resource/organizer checks must hold. No direct FMU publisher/duplicate flight_interface is launched. The immutable source pipeline remains dry-run by default. No hardware command publishing, sensor-provider startup, package installation or live-camera autonomy is authorized by this plan.

This candidate is ready for independent review and **separate retest authorization**, not a claim of Jetson success. The user has explicitly prohibited Jetson access during the current local-development task.
