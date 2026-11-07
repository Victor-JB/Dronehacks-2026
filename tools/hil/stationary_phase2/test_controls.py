"""Local tests only: fake Docker/proc/clock/processes, no hardware or Docker calls."""
import copy
import json
import signal
import subprocess
import sys
import tempfile
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

import container_guard as guard
import monitor_phase2 as monitor
import verify_container as v

TOKEN, CID, IMAGE = 'b' * 32, 'a' * 64, 'sha256:' + 'c' * 64
OWNER = '1000:1000'
IMAGE_CONFIG = {'Env': ['PATH=/usr/bin:/bin'], 'Labels': {}}


def fixture():
    return {'Id': CID, 'Name': '/' + v.NAME, 'State': {'Status': 'created'}, 'Image': IMAGE,
            'Config': {'User': OWNER, 'Entrypoint': ['/usr/bin/timeout'], 'Cmd': v.command(TOKEN),
                       'WorkingDir': '/audit_ws', 'Tty': False, 'OpenStdin': False,
                       'Healthcheck': {'Test': ['NONE']}, 'StopSignal': 'SIGTERM', 'StopTimeout': 5,
                       'Env': IMAGE_CONFIG['Env'], 'Labels': {v.LABEL: TOKEN}},
            'HostConfig': {'NetworkMode': 'none', 'IpcMode': 'private', 'PidMode': '', 'UTSMode': '',
                           'CgroupnsMode': 'private', 'UsernsMode': '',
                           'Privileged': False, 'ReadonlyRootfs': True, 'Runtime': 'runc',
                           'Devices': [], 'DeviceRequests': [], 'CapAdd': [], 'CapDrop': ['ALL'],
                           'SecurityOpt': ['no-new-privileges:true'], 'NanoCpus': 1000000000,
                           'Memory': v.MEMORY, 'MemorySwap': v.MEMORY, 'PidsLimit': 192,
                           'RestartPolicy': {'Name': 'no', 'MaximumRetryCount': 0},
                           'LogConfig': {'Type': 'json-file', 'Config': {'max-size': '5m', 'max-file': '2'}},
                           'Tmpfs': {'/tmp': 'rw,nosuid,nodev,noexec,size=268435456'}},
            'NetworkSettings': {'Ports': {}},
            'Mounts': [{'Destination': dst, 'Source': v.ROOT + suffix, 'Type': 'bind', 'RW': rw, 'Propagation': 'rprivate'}
                       for dst, suffix, rw in (('/reviewed', '/payload', False), ('/audit_ws', '/workspace', True))]}


def verify(d):
    return v.verify(d, v.ROOT, IMAGE, OWNER, TOKEN, IMAGE_CONFIG)


class VerifierTests(unittest.TestCase):
    def test_accept_reviewed(self):
        verify(fixture())

    def test_reject_unsafe_variants(self):
        # Includes every original unsafe fixture, plus deadline/identity/config drift.
        variants = [('HostConfig', 'NetworkMode', 'host'), ('HostConfig', 'IpcMode', 'host'),
                    ('HostConfig', 'PidMode', 'host'), ('HostConfig', 'Privileged', True),
                    ('HostConfig', 'ReadonlyRootfs', False), ('HostConfig', 'Devices', ['serial']),
                    ('HostConfig', 'DeviceRequests', ['gpu']), ('HostConfig', 'CapAdd', ['SYS_ADMIN']),
                    ('HostConfig', 'SecurityOpt', []), ('HostConfig', 'ExtraHosts', ['host']),
                    ('HostConfig', 'PortBindings', {'42': []}), ('HostConfig', 'Memory', 0),
                    ('HostConfig', 'Runtime', 'nvidia'), ('Config', 'User', '0:0'),
                    ('Config', 'Cmd', ['bash']), ('State', 'Status', 'running'),
                    ('HostConfig', 'MemorySwap', -1), ('HostConfig', 'NanoCpus', 0),
                    ('HostConfig', 'PidsLimit', -1), ('HostConfig', 'RestartPolicy', {'Name': 'always'}),
                    ('HostConfig', 'Init', True), ('HostConfig', 'AutoRemove', True),
                    ('HostConfig', 'OomKillDisable', True), ('HostConfig', 'DeviceCgroupRules', ['a']),
                    ('HostConfig', 'VolumesFrom', ['skybeta-stack']), ('HostConfig', 'GroupAdd', ['0']),
                    ('HostConfig', 'Tmpfs', {'/tmp': 'rw,exec'}), ('HostConfig', 'CapDrop', []),
                    ('Config', 'Entrypoint', ['/bin/bash']), ('Config', 'Env', ['LD_PRELOAD=evil']),
                    ('Config', 'Healthcheck', {'Test': ['CMD', 'evil']}), ('Config', 'StopTimeout', 60),
                    ('Config', 'Labels', {}), ('Config', 'WorkingDir', '/root'),
                    ('HostConfig', 'CgroupnsMode', 'host'), ('HostConfig', 'UsernsMode', 'host')]
        for section, key, value in variants:
            with self.subTest(section=section, key=key):
                d = fixture()
                d[section][key] = value
                with self.assertRaises(v.ValidationError):
                    verify(d)
        for change in ('socket', 'rw_source', 'wrong_path', 'shared_propagation', 'duplicate'):
            with self.subTest(mount=change):
                d = fixture()
                if change == 'socket':
                    d['Mounts'].append({'Destination': '/var/run/docker.sock'})
                elif change == 'duplicate':
                    d['Mounts'][1] = copy.deepcopy(d['Mounts'][0])
                elif change == 'rw_source':
                    d['Mounts'][0]['RW'] = True
                elif change == 'wrong_path':
                    d['Mounts'][1]['Source'] = '/home/dronehacks/team_ws'
                else:
                    d['Mounts'][0]['Propagation'] = 'rshared'
                with self.assertRaises(v.ValidationError):
                    verify(d)

    def test_deadline_drift(self):
        for before, after in (('600s', '601s'), ('--kill-after=5s', '--kill-after=50s')):
            d = fixture()
            d['Config']['Cmd'] = [after if x == before else x for x in d['Config']['Cmd']]
            with self.assertRaises(v.ValidationError):
                verify(d)

    def test_wrong_container_identity(self):
        for key, value in (('Name', '/skybeta-stack'), ('Id', CID[:12]), ('Image', 'sha256:' + 'e' * 64)):
            with self.subTest(key=key):
                d = fixture(); d[key] = value
                with self.assertRaises(v.ValidationError): verify(d)


class PolicyTests(unittest.TestCase):
    def sample(self):
        return {'available_ram': 2 * monitor.GIB, 'disk_free': 4 * monitor.GIB, 'elapsed': 10,
                'stack': ('baseline',), 'oom': False, 'oom_events': 0,
                'memory_max': v.MEMORY, 'memory_current': v.MEMORY // 2}

    def test_good_sample(self):
        monitor.check_sample(self.sample(), ('baseline',))

    def test_threshold_failures(self):
        for key, bad in (('available_ram', monitor.GIB - 1), ('disk_free', 2 * monitor.GIB - 1),
                         ('elapsed', 600), ('oom', True), ('oom_events', 1),
                         ('memory_max', 0), ('memory_current', int(v.MEMORY * .90) + 1),
                         ('stack', ('restarted',))):
            with self.subTest(key=key):
                s = self.sample(); s[key] = bad
                with self.assertRaises(v.ValidationError):
                    monitor.check_sample(s, ('baseline',))

    def test_stack_liveness_is_not_sensor_health(self):
        self.assertIn('NOT MONITORED', monitor.health(None, 10))
        with tempfile.TemporaryDirectory() as t:
            p = Path(t) / 'status.json'
            for data in ({'stamp_unix': 10, 'camera_ok': True, 'slam_ok': False},
                         {'stamp_unix': 1, 'camera_ok': True, 'slam_ok': True}):
                p.write_text(json.dumps(data))
                with self.assertRaises(v.ValidationError): monitor.health(p, 10)
            p.write_text(json.dumps({'stamp_unix': 10, 'camera_ok': True, 'slam_ok': True}))
            self.assertIn('OK', monitor.health(p, 10))

    def test_stack_restart_paused_stopped(self):
        good = {'Id': CID, 'Image': IMAGE, 'RestartCount': 0,
                'State': {'Running': True, 'StartedAt': 'start', 'Pid': 42}}
        baseline = monitor.stack_identity(good)
        for change in ('Running', 'Paused', 'OOMKilled'):
            b = copy.deepcopy(good)
            b['State'][change] = change != 'Running'
            with self.assertRaises(v.ValidationError): monitor.stack_identity(b)
        b = copy.deepcopy(good); b['RestartCount'] = 1
        self.assertNotEqual(monitor.stack_identity(b), baseline)

    def test_cleanup_only_full_id_and_forced_fallback(self):
        call = Mock(side_effect=[subprocess.TimeoutExpired('docker', 8), 'killed'])
        monitor.stop_sandbox(CID, call)
        self.assertEqual(call.call_args_list[0].args, ('stop', '--time', '5', CID))
        self.assertEqual(call.call_args_list[1].args, ('kill', '--signal', 'KILL', CID))
        for target in ('skybeta-stack', v.NAME, CID[:12], ''):
            with self.assertRaises(v.ValidationError): monitor.stop_sandbox(target, call)
        self.assertEqual(call.call_count, 2)

    def test_cgroup_v2_read_and_unavailable(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t); proc = root / 'proc'; cg = root / 'cg'
            (proc / '42').mkdir(parents=True); (cg / 'sandbox').mkdir(parents=True)
            (proc / '42/cgroup').write_text('0::/sandbox\n')
            for name, data in (('memory.current', '100'), ('memory.max', str(v.MEMORY)),
                               ('memory.events', 'oom 1\noom_kill 1\n')):
                (cg / 'sandbox' / name).write_text(data)
            self.assertEqual(monitor.memory_metrics(42, proc, cg), (100, v.MEMORY, 2))
            (cg / 'sandbox/memory.events').unlink()
            with self.assertRaises(OSError): monitor.memory_metrics(42, proc, cg)


class GuardTests(unittest.TestCase):
    def test_initial_bad_lease_prevents_spawn(self):
        spawn = Mock()
        for data in ({'token': TOKEN, 'monotonic': 0, 'permit_build': True},
                     {'token': 'wrong', 'monotonic': 20, 'permit_build': True}):
            with self.assertRaises(v.ValidationError):
                guard.supervise(lambda: data, TOKEN, spawn, clock=lambda: 20, stop=Mock())
        spawn.assert_not_called()

    def test_monitor_loss_stops_child_after_ttl(self):
        now = [0.0]
        proc, stop = Mock(), Mock()
        proc.poll.return_value = None
        def sleep(dt): now[0] += dt
        with self.assertRaises(v.ValidationError):
            guard.supervise(lambda: {'token': TOKEN, 'monotonic': 0, 'permit_build': True},
                            TOKEN, lambda: proc, clock=lambda: now[0], sleep=sleep, stop=stop)
        self.assertLessEqual(now[0], 12.25)
        stop.assert_called_once_with(proc)

    def test_revocation_and_child_failure(self):
        proc, stop = Mock(), Mock(); proc.poll.return_value = None
        leases = iter([{'token': TOKEN, 'monotonic': 1, 'permit_build': True},
                       {'token': TOKEN, 'monotonic': 1, 'permit_build': False}])
        with self.assertRaises(v.ValidationError):
            guard.supervise(lambda: next(leases), TOKEN, lambda: proc, clock=lambda: 1, sleep=lambda _: None, stop=stop)
        stop.assert_called_once_with(proc)
        proc.poll.return_value = 17; proc.returncode = 17
        self.assertEqual(guard.supervise(lambda: {'token': TOKEN, 'monotonic': 1, 'permit_build': True},
                                        TOKEN, lambda: proc, clock=lambda: 1, stop=Mock()), 17)

    def test_cleanup_term_then_kill_is_bounded(self):
        proc = Mock(); proc.pid = 12345
        proc.wait.side_effect = [subprocess.TimeoutExpired('mock', 2), None]
        with patch.object(guard.os, 'killpg', create=True) as kill, patch.object(signal, 'SIGKILL', 9, create=True):
            guard.stop_group(proc)
            self.assertEqual(kill.call_args_list[0].args, (12345, signal.SIGTERM))
            self.assertEqual(kill.call_args_list[1].args, (12345, 9))
        self.assertEqual([c.kwargs['timeout'] for c in proc.wait.call_args_list], [2, 1])

    def test_timeout_probe_failure_or_missing_prevents_work(self):
        with patch.object(guard.Path, 'resolve', return_value=Path('/usr/bin/timeout')), \
             patch.object(guard.Path, 'read_bytes', return_value=('timeout\0' + '\0'.join(v.command(TOKEN)) + '\0').encode()):
            for run in (Mock(side_effect=FileNotFoundError('timeout')),
                        Mock(return_value=Mock(returncode=0, stdout='not GNU')),
                        Mock(side_effect=[Mock(returncode=0, stdout='timeout (GNU coreutils) 9'), Mock(returncode=0)]),
                        Mock(side_effect=[Mock(returncode=0, stdout='timeout (GNU coreutils) 9'), Mock(returncode=124), Mock(returncode=124)])):
                with self.assertRaises((v.ValidationError, OSError)):
                    guard.verify_timeout(TOKEN, run=run)
            guard.verify_timeout(TOKEN, run=Mock(side_effect=[Mock(returncode=0, stdout='timeout (GNU coreutils) 9'), Mock(returncode=124), Mock(returncode=-9)]))


class HostLifecycleTests(unittest.TestCase):
    def exercise(self, missing_metrics=False):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t).resolve()
            (root / 'payload').mkdir()
            (root / 'workspace').mkdir()
            stack = {'Id': 'd' * 64, 'Image': IMAGE, 'RestartCount': 0,
                     'State': {'Running': True, 'StartedAt': 'baseline', 'Pid': 42}}
            initial = fixture()
            running = copy.deepcopy(initial)
            running['State'] = {'Running': True, 'Pid': 99, 'OOMKilled': False}
            exited = copy.deepcopy(running)
            exited['State'].update(Running=False, ExitCode=0)
            observations = iter([running, exited])
            def fake_docker(*args, **kwargs):
                if args == ('image', 'inspect', IMAGE):
                    return json.dumps([{'Id': IMAGE, 'Config': IMAGE_CONFIG}])
                if args == ('inspect', CID, 'skybeta-stack'):
                    return json.dumps([next(observations), stack])
                if args == ('inspect', 'skybeta-stack'):
                    return json.dumps([stack])
                if args == ('inspect', CID):
                    return json.dumps([initial])
                return ''
            call = Mock(side_effect=fake_docker)
            lease, stop = Mock(), Mock()
            metrics = Mock(return_value=(100, v.MEMORY, 0), side_effect=OSError('no cgroup') if missing_metrics else None)
            a = Mock(root=str(root), container_id=CID, image=IMAGE, owner=OWNER, token=TOKEN, health_file=None)
            with patch.dict(sys.modules, {'fcntl': Mock(LOCK_EX=2, LOCK_NB=4)}), patch.object(monitor, 'ROOT', str(root)), \
                 patch.object(monitor, 'docker', call), patch.object(monitor, 'verify') as checked, \
                 patch.object(monitor, 'available_ram', return_value=2 * monitor.GIB), \
                 patch.object(monitor.shutil, 'disk_usage', return_value=Mock(free=4 * monitor.GIB)), \
                 patch.object(monitor, 'memory_metrics', metrics), patch.object(monitor, 'write_lease', lease), \
                 patch.object(monitor, 'stop_sandbox', stop), patch.object(monitor.time, 'sleep'):
                if missing_metrics:
                    with self.assertRaises(OSError): monitor.run(a)
                else:
                    monitor.run(a)
                checked.assert_called_once()
                stop.assert_called_once_with(CID)
                self.assertIn(('start', CID), [c.args for c in call.call_args_list])
                for c in call.call_args_list:
                    if c.args[0] in ('start', 'stop', 'kill'):
                        self.assertEqual(c.args[-1], CID)
                permits = [c.args[-1] for c in lease.call_args_list]
                self.assertEqual(permits, [False, False] if missing_metrics else [False, True, False])

    def test_monitor_lifecycle_uses_verified_id(self):
        self.exercise()

    def test_missing_telemetry_never_permits_build(self):
        self.exercise(missing_metrics=True)


if __name__ == '__main__':
    unittest.main(verbosity=2)
