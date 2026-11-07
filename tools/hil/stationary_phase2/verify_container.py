"""Fail-closed validation of the stopped Phase 2 sandbox; safe under python -O."""
import argparse
import json
import re
import sys

MEMORY = 1610612736
LABEL = 'org.dronehacks.phase2.run'
ROOT = '/home/dronehacks/codex_stationary/eab3636-phase2'
NAME = 'codex-autonomy-eab3636-phase2'


class ValidationError(RuntimeError):
    pass


def require(ok, message):
    if not ok:
        raise ValidationError(message)


def command(token):
    return ['--signal=TERM', '--kill-after=5s', '600s', '/usr/bin/env', '-i',
            'PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin',
            'HOME=/audit_ws/home', 'ROS_DOMAIN_ID=77', 'ROS_LOCALHOST_ONLY=1',
            'RMW_IMPLEMENTATION=rmw_fastrtps_cpp', 'ROS_LOG_DIR=/audit_ws/log/ros',
            'PYTHONDONTWRITEBYTECODE=1', 'OPENBLAS_NUM_THREADS=1', 'OMP_NUM_THREADS=1',
            '/usr/bin/python3', '-u', '/reviewed/container_guard.py', '--token', token]


def verify(d, root, image, uid, token, image_config):
    require(root == ROOT, 'Unapproved workspace')
    require(re.fullmatch(r'[0-9a-f]{32}', token) is not None, 'Invalid run token')
    require(re.fullmatch(r'[1-9][0-9]*:[0-9]+', uid) is not None, 'Require numeric nonroot UID:GID')
    require(re.fullmatch(r'sha256:[0-9a-f]{64}', image) is not None, 'Require immutable image ID')
    require(re.fullmatch(r'[0-9a-f]{64}', d['Id']) is not None, 'Require full container ID')
    h, c = d['HostConfig'], d['Config']
    require(d['Name'] == '/' + NAME, 'Unexpected name')
    require(d['State']['Status'] == 'created', 'Must verify before first start')
    require(d['Image'] == image, 'Unexpected image ID')
    require(c['User'] == uid, 'Unexpected user')
    require(h['NetworkMode'] == 'none' and not d['NetworkSettings']['Ports'], 'Network isolation')
    require(h['IpcMode'] == 'private' and h['PidMode'] == '' and h['UTSMode'] == ''
            and h['CgroupnsMode'] == 'private' and h['UsernsMode'] == '', 'Namespace isolation')
    require(not h['Privileged'] and h['ReadonlyRootfs'], 'Privilege/rootfs isolation')
    require(h['Runtime'] == 'runc' and not h.get('DeviceRequests'), 'No GPU/runtime injection')
    for key in ('Devices', 'DeviceCgroupRules', 'CapAdd', 'VolumesFrom', 'Links',
                'ExtraHosts', 'PortBindings', 'GroupAdd', 'Sysctls', 'CgroupParent',
                'Binds', 'PublishAllPorts', 'Dns', 'DnsOptions', 'DnsSearch',
                'CpuQuota', 'CpuPeriod', 'CpuShares', 'CpusetMems', 'MemoryReservation'):
        require(not h.get(key), 'Unexpected ' + key)
    require(h['CapDrop'] == ['ALL'], 'All capabilities must be dropped')
    require(h['SecurityOpt'] in (['no-new-privileges'], ['no-new-privileges:true']), 'Security options')
    require(h['NanoCpus'] == 1000000000 and not h.get('CpusetCpus'), 'CPU quota')
    require(h['Memory'] == MEMORY and h['MemorySwap'] == MEMORY, 'Memory/swap limit')
    require(not h.get('OomKillDisable') and not h.get('OomScoreAdj'), 'OOM settings')
    require(h['PidsLimit'] == 192, 'Process limit')
    require(h['RestartPolicy'] == {'Name': 'no', 'MaximumRetryCount': 0}, 'Restart forbidden')
    require(not h.get('AutoRemove') and not h.get('Init'), 'Keep evidence / timeout must be PID1')
    require(h['LogConfig'] == {'Type': 'json-file', 'Config': {'max-size': '5m', 'max-file': '2'}}, 'Bounded Docker logs')
    mounts = d['Mounts']
    require(len(mounts) == 2, 'Unexpected mounts')
    for dst, suffix, rw in (('/reviewed', '/payload', False), ('/audit_ws', '/workspace', True)):
        found = [m for m in mounts if m['Destination'] == dst]
        require(len(found) == 1, 'Missing/duplicate mount ' + dst)
        m = found[0]
        require(m['Type'] == 'bind' and m['Source'] == root + suffix and m['RW'] is rw, 'Mount path/mode')
        require(m.get('Propagation') == 'rprivate', 'Mount propagation')
    require(h['Tmpfs'] == {'/tmp': 'rw,nosuid,nodev,noexec,size=268435456'}, 'Exact tmpfs')
    require(c['WorkingDir'] == '/audit_ws' and not c['Tty'] and not c['OpenStdin'], 'Working directory/stdio')
    require(c['Entrypoint'] == ['/usr/bin/timeout'] and c['Cmd'] == command(token), 'Deadline/entrypoint/arguments')
    require(c['Healthcheck'] == {'Test': ['NONE']}, 'No inherited image health command')
    require(c['StopSignal'] == 'SIGTERM' and c['StopTimeout'] == 5, 'Stop fallback')
    require(c['Env'] == image_config.get('Env'), 'Unexpected image environment override')
    labels = dict(image_config.get('Labels') or {})
    labels[LABEL] = token
    require(c['Labels'] == labels, 'Run identity/labels')


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    for key in ('root', 'image', 'owner', 'token', 'image-json'):
        p.add_argument('--' + key, required=True)
    a = p.parse_args()
    try:
        with open(a.image_json) as f:
            config = json.load(f)[0]['Config']
        verify(json.load(sys.stdin)[0], a.root, a.image, a.owner, a.token, config)
    except (ValidationError, KeyError, ValueError, OSError, TypeError) as e:
        sys.exit('FAIL: ' + str(e))
    print('PASS: stopped sandbox isolation, identity, limits and 600s+5s deadline configuration')
