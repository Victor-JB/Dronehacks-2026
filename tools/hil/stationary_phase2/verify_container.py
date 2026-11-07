"""Validate a STOPPED Docker container before the proposed Phase 2 start.

Read docker-inspect JSON from stdin. This tool does not contact Docker or ROS.
"""
import json
import sys


def verify(d, root, image, uid):
    h, c = d['HostConfig'], d['Config']
    assert d['State']['Status'] == 'created', 'Must inspect before first start'
    assert d['Image'] == image, 'Unexpected image ID'
    assert c['User'] == uid and uid.split(':')[0] != '0', 'Require approved nonroot UID:GID'
    assert h['NetworkMode'] == 'none' and not d['NetworkSettings']['Ports']
    assert h['IpcMode'] == 'private' and h['PidMode'] == ''
    assert not h['Privileged'] and h['ReadonlyRootfs']
    assert h['Runtime'] == 'runc' and not h.get('DeviceRequests')
    assert not h.get('Devices') and not h.get('CapAdd')
    assert h['CapDrop'] == ['ALL']
    assert 'no-new-privileges' in h['SecurityOpt'] or 'no-new-privileges:true' in h['SecurityOpt']
    assert not h.get('VolumesFrom') and not h.get('Links')
    assert not h.get('ExtraHosts') and not h.get('PortBindings')
    assert h['NanoCpus'] == 1000000000
    assert h['Memory'] == 1610612736 and h['MemorySwap'] == h['Memory']
    assert h['PidsLimit'] == 192
    assert h['RestartPolicy']['Name'] == 'no'
    mounts = {m['Destination']: m for m in d['Mounts']}
    assert set(mounts) == {'/reviewed', '/audit_ws'}, mounts
    for dst, suffix, rw in (('/reviewed', '/payload', False), ('/audit_ws', '/workspace', True)):
        m = mounts[dst]
        assert m['Type'] == 'bind' and m['Source'] == root + suffix and m['RW'] is rw
    assert set(h['Tmpfs']) == {'/tmp'}
    assert c['Entrypoint'] == ['/usr/bin/env']
    expected = ['-i', 'PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin',
                'HOME=/audit_ws/home', 'ROS_DOMAIN_ID=77', 'ROS_LOCALHOST_ONLY=1',
                'RMW_IMPLEMENTATION=rmw_fastrtps_cpp', 'ROS_LOG_DIR=/audit_ws/log/ros',
                'PYTHONDONTWRITEBYTECODE=1', 'OPENBLAS_NUM_THREADS=1', 'OMP_NUM_THREADS=1',
                '/bin/bash', '--noprofile', '--norc',
                '/reviewed/container_build.sh', '--approved-isolated-build']
    # No inherited image startup, peer/profile variables or user remapping arguments.
    assert c['Cmd'] == expected, c['Cmd']


if __name__ == '__main__':
    verify(json.load(sys.stdin)[0], *sys.argv[1:])
    print('PASS: stopped container configuration matches proposed isolation boundary')
