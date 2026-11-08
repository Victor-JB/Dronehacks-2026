"""Reviewed remote export/cleanup helper, sent over stdin by finish_phase2.py.
No ROS, no new privileges, no cleanup until the Windows verifier supplies both hashes.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import tarfile

ROOT = Path('/home/dronehacks/codex_stationary/eab3636-phase2')
NAME = '/codex-autonomy-eab3636-phase2'


def need(ok, why):
    if not ok: raise RuntimeError(why)


def docker(*args):
    return subprocess.check_output(['docker', *args], text=True, timeout=10, stderr=subprocess.STDOUT)


def identity(cid, pin):
    need(re.fullmatch('[0-9a-f]{64}', cid) is not None, 'Full container ID required')
    need(re.fullmatch('[0-9a-f]{40}', pin) is not None, 'Frozen source commit required')
    need(not ROOT.is_symlink() and ROOT.resolve() == ROOT and ROOT.stat().st_uid == os.getuid() == 1000, 'Canonical root/owner')
    need((ROOT / 'source_commit.txt').read_text().strip() == pin, 'Wrong source candidate')
    need((ROOT / 'sandbox-id.txt').read_text().strip() == cid, 'Wrong recorded ID')
    d = json.loads(docker('inspect', cid))[0]
    b = json.loads((ROOT / 'container-before-start.json').read_text())
    if isinstance(b, list): b = b[0]
    need(d['Id'] == b['Id'] == cid and d['Name'] == NAME and not d['State']['Running'], 'Stopped sandbox identity')
    for obj in (d, b):
        need(obj['HostConfig'].get('OomKillDisable') in (False, None), 'OOM configuration drift')
        obj['HostConfig']['OomKillDisable'] = False
    for key in ('Image', 'Config', 'HostConfig', 'Mounts'):
        need(d[key] == b[key], 'Sandbox drift: ' + key)
    need(subprocess.run(['pgrep', '-af', '[m]onitor_phase2.py'], capture_output=True).returncode == 1, 'Monitor still active')
    sb = json.loads((ROOT / 'stack-before.json').read_text())
    s = json.loads(docker('inspect', 'skybeta-stack'))[0]
    for key in ('Id', 'Image', 'RestartCount'): need(s[key] == sb[key], 'Organizer drift')
    for key in ('Pid', 'StartedAt'): need(s['State'][key] == sb['State'][key], 'Organizer restart')
    need(s['State']['Running'] and not s['State']['Paused'] and not s['State']['OOMKilled'], 'Organizer state')
    return d


def entries(root):
    result = []
    for directory, dirs, files in os.walk(root, followlinks=False):
        for name in dirs + files:
            p = Path(directory) / name
            rel = p.relative_to(root).as_posix()
            if rel in ('complete-export.tar.gz', 'export-manifest.json'): continue
            st = p.lstat(); need(st.st_uid == 1000, 'Unexpected owner: ' + rel)
            item = dict(path=rel, uid=st.st_uid, gid=st.st_gid, mode=stat.S_IMODE(st.st_mode))
            if p.is_symlink(): item.update(type='symlink', target=os.readlink(p))
            elif p.is_dir(): item.update(type='directory')
            elif p.is_file(): item.update(type='file', size=st.st_size, sha256=hashlib.sha256(p.read_bytes()).hexdigest())
            else: raise RuntimeError('Special file: ' + rel)
            result.append(item)
    return sorted(result, key=lambda x: x['path'])


def execute(mode, cid, pin, archive_hash=None, manifest_hash=None):
    d = identity(cid, pin)
    archive, manifest = ROOT / 'complete-export.tar.gz', ROOT / 'export-manifest.json'
    if mode == 'export':
        need(not archive.exists() and not manifest.exists(), 'Do not overwrite evidence')
        (ROOT / 'final-container.json').write_text(json.dumps(d, indent=2))
        (ROOT / 'final-container.log').write_text(docker('logs', cid))
        data = dict(canonical_root=str(ROOT), container_id=cid, source_commit=pin, entries=entries(ROOT))
        manifest.write_text(json.dumps(data, indent=2))
        with tarfile.open(archive, 'w:gz', dereference=False) as t:
            for item in data['entries']: t.add(ROOT / item['path'], arcname=item['path'], recursive=False)
            t.add(manifest, arcname=manifest.name, recursive=False)
        return dict(archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
                    manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest(), entries=len(data['entries']))
    need(hashlib.sha256(archive.read_bytes()).hexdigest() == archive_hash, 'Archive changed; retain everything')
    need(hashlib.sha256(manifest.read_bytes()).hexdigest() == manifest_hash, 'Manifest changed; retain everything')
    need(entries(ROOT) == json.loads(manifest.read_text())['entries'], 'Contents changed; retain everything')
    for line in Path('/proc/self/mountinfo').read_text().splitlines():
        mp = line.split()[4]
        need(mp != str(ROOT) and not mp.startswith(str(ROOT) + '/'), 'Nested mount')
    need(shutil.rmtree.avoids_symlink_attacks, 'Secure removal unavailable')
    organizer_before = json.loads(docker('inspect', 'skybeta-stack'))[0]
    inode = (ROOT.stat().st_dev, ROOT.stat().st_ino)
    docker('rm', cid)  # Full verified stopped ID only; no force or prune.
    need((ROOT.stat().st_dev, ROOT.stat().st_ino) == inode and ROOT.resolve() == ROOT, 'Root changed')
    shutil.rmtree(ROOT)
    need(not os.path.lexists(ROOT), 'Root remains')
    need(not docker('ps', '-aq', '--filter', 'id=' + cid).strip(), 'Container remains')
    need(not docker('ps', '-aq', '--filter', 'name=^' + NAME + '$').strip(), 'Sandbox name remains')
    need(subprocess.run(['pgrep', '-af', '[m]onitor_phase2.py'], capture_output=True).returncode == 1, 'Monitor remains')
    s = json.loads(docker('inspect', 'skybeta-stack'))[0]
    need(s['State']['Running'] and not s['State']['Paused'] and not s['State']['OOMKilled'], 'Organizer not healthy after cleanup')
    for key in ('Id', 'Image', 'RestartCount'): need(s[key] == organizer_before[key], 'Organizer drift after cleanup')
    for key in ('Pid', 'StartedAt'): need(s['State'][key] == organizer_before['State'][key], 'Organizer restart after cleanup')
    return dict(container_removed=True, directory_removed=True, monitor_inactive=True, source_commit=pin)


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('mode', choices=('export', 'cleanup'))
    p.add_argument('container_id'); p.add_argument('source_commit')
    p.add_argument('--archive-hash'); p.add_argument('--manifest-hash')
    a = p.parse_args()
    print(json.dumps(execute(a.mode, a.container_id, a.source_commit, a.archive_hash, a.manifest_hash)))
