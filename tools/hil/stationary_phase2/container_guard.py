"""Inside sandbox only: verify GNU timeout PID1; fail closed on expired host lease.

The outer GNU timeout is independent of this Python process and its cleanup.
Exiting PID1 ends the private PID namespace, including detached ROS descendants.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from verify_container import ValidationError, command, require

TTL = 12.0


def lease_valid(data, token, now):
    require(data.get('token') == token, 'Lease token mismatch')
    age = now - float(data['monotonic'])
    require(-0.05 <= age <= TTL, 'Host monitor lease missing/stale')
    require(type(data.get('permit_build')) is bool, 'Invalid lease permission')
    return data['permit_build']


def verify_timeout(token, run=subprocess.run):
    require(Path('/proc/1/exe').resolve() == Path('/usr/bin/timeout').resolve(), 'Timeout is not PID1')
    actual = Path('/proc/1/cmdline').read_bytes().split(b'\0')
    require([x.decode() for x in actual if x][1:] == command(token), 'Unexpected deadline arguments')
    v = run(['/usr/bin/timeout', '--version'], capture_output=True, text=True, timeout=3)
    require(v.returncode == 0 and v.stdout.startswith('timeout (GNU coreutils)'), 'Unverified timeout implementation')
    t = time.monotonic()
    probe = run(['/usr/bin/timeout', '--signal=TERM', '--kill-after=1s', '1s',
                 '/usr/bin/python3', '-c', 'import time; time.sleep(30)'], timeout=4)
    require(probe.returncode == 124 and time.monotonic() - t < 4, 'Timeout self-test failed')
    t = time.monotonic()
    forced = run(['/usr/bin/timeout', '--signal=TERM', '--kill-after=1s', '0.5s',
                  '/usr/bin/python3', '-c',
                  'import signal,time; signal.signal(signal.SIGTERM,signal.SIG_IGN); time.sleep(30)'], timeout=4)
    require(forced.returncode in (-9, 137) and time.monotonic() - t < 4, 'Forced-kill timeout self-test failed')


def stop_group(proc):
    if proc is not None:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=2)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=1)
            except subprocess.TimeoutExpired:
                pass


def supervise(read_lease, token, spawn, clock=time.monotonic, sleep=time.sleep, stop=stop_group):
    proc = None
    try:
        while True:
            permit = lease_valid(read_lease(), token, clock())
            require(permit or proc is None, 'Host monitor revoked build permission')
            if permit and proc is None:
                proc = spawn()
            if proc is not None and proc.poll() is not None:
                return proc.returncode
            sleep(0.25)
    finally:
        stop(proc)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--token', required=True)
    args = ap.parse_args()
    def terminated(*_):
        raise ValidationError('Deadline/termination')
    signal.signal(signal.SIGTERM, terminated)
    try:
        verify_timeout(args.token)
        def read():
            return json.loads(Path('/reviewed/monitor_lease.json').read_text())
        rc = supervise(read, args.token, lambda: subprocess.Popen(
            ['/bin/bash', '--noprofile', '--norc', '/reviewed/container_build.sh', '--approved-isolated-build'],
            start_new_session=True))
        raise SystemExit(rc)
    except (ValidationError, OSError, ValueError, KeyError, subprocess.SubprocessError) as e:
        print('GUARD ABORT:', str(e), flush=True)
        raise SystemExit(1)
