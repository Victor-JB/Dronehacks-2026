"""Proposed host-side supervisor. Run only after explicit Phase 2 authorization.

No organizer process is changed. All Docker mutations use the one verified full
sandbox ID. Missing telemetry revokes the read-only sandbox lease and stops it.
"""
import argparse
from contextlib import ExitStack
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import time

from verify_container import LABEL, MEMORY, ROOT, ValidationError, require, verify

GIB = 1024 ** 3


def docker(*args, timeout=4):
    p = subprocess.run(['docker', *args], capture_output=True, text=True, timeout=timeout)
    require(p.returncode == 0, 'Docker failed: ' + p.stderr[:500])
    return p.stdout


def stack_identity(d):
    require(d['State']['Running'] and not d['State'].get('Paused') and not d['State'].get('OOMKilled'), 'Organizer stack not healthy/running')
    return (d['Id'], d['Image'], d['RestartCount'], d['State']['StartedAt'], d['State']['Pid'])


def check_sample(s, baseline):
    require(s['available_ram'] >= GIB, 'Host available RAM below 1 GiB')
    require(s['disk_free'] >= 2 * GIB, 'Host disk free below 2 GiB')
    require(s['elapsed'] < 600, 'Whole-phase deadline')
    require(s['stack'] == baseline, 'Organizer container changed/restarted')
    require(not s['oom'] and s['oom_events'] == 0, 'Sandbox OOM/allocation failure')
    require(s['memory_max'] == MEMORY, 'Unverified memory enforcement')
    require(0 <= s['memory_current'] < MEMORY * 0.90, 'Sandbox memory reaches 90% cap')


def memory_metrics(pid, proc=Path('/proc'), cgroups=Path('/sys/fs/cgroup')):
    lines = (proc / str(pid) / 'cgroup').read_text().splitlines()
    v2 = [line.split(':', 2)[2] for line in lines if line.startswith('0::')]
    if v2:
        path = (cgroups / v2[0].lstrip('/')).resolve()
        require(path.is_relative_to(cgroups.resolve()), 'Unexpected cgroup path')
        events = dict(line.split() for line in (path / 'memory.events').read_text().splitlines())
        return (int((path / 'memory.current').read_text()), int((path / 'memory.max').read_text()),
                int(events['oom']) + int(events['oom_kill']))
    v1 = [line.split(':', 2)[2] for line in lines if 'memory' in line.split(':', 2)[1].split(',')]
    require(len(v1) == 1, 'Memory/OOM cgroup telemetry unavailable')
    path = (cgroups / 'memory' / v1[0].lstrip('/')).resolve()
    require(path.is_relative_to((cgroups / 'memory').resolve()), 'Unexpected cgroup path')
    return (int((path / 'memory.usage_in_bytes').read_text()),
            int((path / 'memory.limit_in_bytes').read_text()), int((path / 'memory.failcnt').read_text()))


def available_ram():
    entries = dict(line.split(':', 1) for line in Path('/proc/meminfo').read_text().splitlines())
    return int(entries['MemAvailable'].split()[0]) * 1024


def health(path, now):
    if path is None:
        return 'NOT MONITORED: no approved camera/SLAM health source'
    d = json.loads(path.read_text())
    require(-0.1 <= now - float(d['stamp_unix']) <= 5, 'Stale/invalid camera/SLAM health source')
    require(d.get('camera_ok') is True and d.get('slam_ok') is True, 'Camera/SLAM health degraded')
    return 'approved read-only camera/SLAM health source OK'


def write_lease(root, token, permit):
    path = root / 'payload/monitor_lease.json'
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps({'token': token, 'monotonic': time.monotonic(), 'permit_build': permit}))
    os.chmod(tmp, 0o644)
    os.replace(tmp, path)


def stop_sandbox(cid, call=docker):
    require(re.fullmatch(r'[0-9a-f]{64}', cid) is not None, 'Refuse unchecked cleanup target')
    try:
        call('stop', '--time', '5', cid, timeout=8)
    except (ValidationError, subprocess.SubprocessError, OSError):
        call('kill', '--signal', 'KILL', cid, timeout=4)


def run(a):
    with ExitStack() as resources:
        return _run(a, resources)


def _run(a, resources):
    root = Path(a.root)
    require(str(root.resolve()) == ROOT, 'Unapproved or symlinked root')
    for directory in ('payload', 'workspace'):
        p = root / directory
        require(p.is_dir() and p.resolve() == p, 'Missing/symlinked bind directory')
    require(re.fullmatch(r'[0-9a-f]{64}', a.container_id) is not None, 'Require full sandbox ID')
    # Lock prevents two host monitors issuing leases for the same workspace.
    import fcntl
    lock = resources.enter_context((root / 'monitor.lock').open('x'))
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    image_info = json.loads(docker('image', 'inspect', a.image))[0]
    require(image_info['Id'] == a.image, 'Image ID drift')
    d = json.loads(docker('inspect', a.container_id))[0]
    verify(d, a.root, a.image, a.owner, a.token, image_info['Config'])
    require(d['Id'] == a.container_id, 'Container ID mismatch')
    # No cleanup authorization until this specific full ID passes verification.
    cid = d['Id']
    baseline_info = json.loads(docker('inspect', 'skybeta-stack'))[0]
    baseline = stack_identity(baseline_info)
    require(available_ram() >= 2 * GIB and shutil.disk_usage(root).free >= 4 * GIB, 'Insufficient starting headroom')
    health_path = Path(a.health_file) if a.health_file else None
    health(health_path, time.time())
    (root / 'container-before-start.json').write_text(json.dumps(d, indent=2))
    (root / 'stack-before.json').write_text(json.dumps(baseline_info, indent=2))
    log = resources.enter_context((root / 'monitor.jsonl').open('x', buffering=1))
    start = time.monotonic()
    started = False
    try:
        write_lease(root, a.token, False)  # Guard may self-test but cannot build yet.
        started = True  # A start RPC can succeed even if its response times out.
        docker('start', cid)
        while True:
            sandbox, stack = json.loads(docker('inspect', cid, 'skybeta-stack'))
            require(sandbox['Id'] == cid and sandbox['Config']['Labels'].get(LABEL) == a.token, 'Sandbox identity drift')
            current_stack = stack_identity(stack)
            require(current_stack == baseline, 'Organizer container changed/restarted')
            sensor = health(health_path, time.time())
            elapsed = time.monotonic() - start
            require(available_ram() >= GIB and shutil.disk_usage(root).free >= 2 * GIB, 'Host resource threshold')
            if not sandbox['State']['Running']:
                (root / 'container-after-test.json').write_text(json.dumps(sandbox, indent=2))
                (root / 'stack-after.json').write_text(json.dumps(stack, indent=2))
                require(not sandbox['State']['OOMKilled'] and sandbox['State']['ExitCode'] == 0, 'Sandbox failed/timeout/OOM')
                require(elapsed < 600, 'Deadline exceeded')
                log.write(json.dumps({'result': 'sandbox exit0; inspect test evidence separately', 'elapsed': elapsed, 'sensors': sensor}) + '\n')
                return
            used, limit, events = memory_metrics(sandbox['State']['Pid'])
            sample = {'available_ram': available_ram(), 'disk_free': shutil.disk_usage(root).free,
                      'elapsed': elapsed, 'stack': current_stack, 'oom': sandbox['State']['OOMKilled'],
                      'memory_current': used, 'memory_max': limit, 'oom_events': events, 'sensors': sensor}
            log.write(json.dumps(sample) + '\n')
            check_sample(sample, baseline)
            write_lease(root, a.token, True)  # Only complete, healthy samples renew permission.
            time.sleep(2)
    except BaseException as e:
        log.write(json.dumps({'abort': str(e), 'elapsed': time.monotonic() - start}) + '\n')
        raise
    finally:
        try:
            write_lease(root, a.token, False)
        finally:
            if started:
                try:
                    stop_sandbox(cid)
                finally:
                    # Bounded RPCs; never remove resources or evidence automatically.
                    for filename, args in (('container-final.json', ('inspect', cid)),
                                           ('container.log', ('logs', cid)),
                                           ('stack-final.json', ('inspect', 'skybeta-stack'))):
                        try:
                            (root / filename).write_text(docker(*args))
                        except (OSError, ValidationError, subprocess.SubprocessError):
                            pass


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    for arg in ('root', 'container-id', 'image', 'owner', 'token'):
        p.add_argument('--' + arg, required=True)
    p.add_argument('--health-file', help='Only an explicitly approved existing read-only JSON source')
    p.add_argument('--approved-isolated-build', action='store_true', required=True)
    a = p.parse_args()
    def terminate(*_):
        raise ValidationError('Host monitor termination')
    signal.signal(signal.SIGTERM, terminate)
    try:
        run(a)
    except (ValidationError, OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as e:
        raise SystemExit('MONITOR ABORT: ' + str(e))
