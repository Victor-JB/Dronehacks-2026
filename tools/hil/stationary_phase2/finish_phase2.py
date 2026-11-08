"""WINDOWS/LOCAL only: automatic export -> byte verification -> narrow cleanup.

Run only after separate retest/cleanup authorization. Existing SSH authentication
is used; no password handling or storage. Any exception BEFORE verified export
prevents cleanup. Failed test evidence is exported too, never discarded.
"""
import argparse
import gzip
import hashlib
import io
import json
from pathlib import Path, PurePosixPath
import re
import shlex
import subprocess
import tarfile

ROOT = '/home/dronehacks/codex_stationary/eab3636-phase2'


def verify_export(data, proof, cid, pin):
    if hashlib.sha256(data).hexdigest() != proof['archive_sha256']:
        raise RuntimeError('Export hash mismatch; NO CLEANUP')
    with tarfile.open(fileobj=io.BytesIO(gzip.decompress(data))) as t:
        members = t.getmembers()
        by_name = {m.name: m for m in members}
        if len(by_name) != len(members): raise RuntimeError('Duplicate archive names')
        mbytes = t.extractfile('export-manifest.json').read()
        if hashlib.sha256(mbytes).hexdigest() != proof['manifest_sha256']:
            raise RuntimeError('Manifest mismatch; NO CLEANUP')
        m = json.loads(mbytes)
        if m['canonical_root'] != ROOT or m['container_id'] != cid or m['source_commit'] != pin:
            raise RuntimeError('Export identity mismatch; NO CLEANUP')
        if set(by_name) != {i['path'] for i in m['entries']} | {'export-manifest.json'}:
            raise RuntimeError('Incomplete export; NO CLEANUP')
        for item in m['entries']:
            path = PurePosixPath(item['path']); node = by_name[item['path']]
            if path.is_absolute() or '..' in path.parts or item['uid'] != 1000 or node.uid != 1000:
                raise RuntimeError('Unsafe ownership/path; NO CLEANUP')
            if item['type'] == 'file':
                blob = t.extractfile(node).read()
                if not node.isfile() or len(blob) != item['size'] or hashlib.sha256(blob).hexdigest() != item['sha256']:
                    raise RuntimeError('File hash mismatch; NO CLEANUP')
                if str(path).endswith(('.log', '.json', '.jsonl', '.xml')):
                    blob.decode('utf-8')  # Verify human-readable evidence before deletion.
            elif item['type'] == 'directory':
                if not node.isdir(): raise RuntimeError('Directory mismatch')
            elif item['type'] != 'symlink' or not node.issym() or node.linkname != item['target']:
                raise RuntimeError('Unexpected archive member')
        return m


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--target', choices=('skybeta-02.local', '192.168.68.77'), default='skybeta-02.local')
    p.add_argument('--container-id', required=True); p.add_argument('--source-commit', required=True)
    p.add_argument('--output', required=True); p.add_argument('--approved-cleanup', action='store_true', required=True)
    a = p.parse_args()
    if not re.fullmatch('[0-9a-f]{64}', a.container_id) or not re.fullmatch('[0-9a-f]{40}', a.source_commit):
        raise RuntimeError('Exact full ID/commit required')
    output = Path(a.output).resolve(); output.mkdir(parents=True, exist_ok=False)
    helper = Path(__file__).with_name('export_cleanup.py').read_text(encoding='utf-8')
    opts = ['-o', 'HostKeyAlias=skybeta-02.local', '-o', 'StrictHostKeyChecking=yes', '-o', 'ConnectTimeout=8']
    target = 'dronehacks@' + a.target
    def remote(mode, hashes=()):
        args = ['env', 'HISTFILE=/dev/null', 'PYTHONDONTWRITEBYTECODE=1', '/usr/bin/python3', '-',
                mode, a.container_id, a.source_commit, *hashes]
        result = subprocess.run(['ssh', '-T', *opts, target, shlex.join(args)], input=helper,
                                text=True, stdout=subprocess.PIPE, timeout=180, check=True)
        return json.loads(result.stdout)
    proof = remote('export')
    (output / 'remote-export-proof.json').write_text(json.dumps(proof, indent=2))
    archive = output / 'complete-export.tar.gz'
    subprocess.run(['scp', *opts, target + ':' + ROOT + '/complete-export.tar.gz', str(archive)], check=True, timeout=180)
    manifest = verify_export(archive.read_bytes(), proof, a.container_id, a.source_commit)
    (output / 'verified-manifest.json').write_text(json.dumps(manifest, indent=2))
    (output / 'LOCAL_EXPORT_VERIFIED.json').write_text(json.dumps(dict(verified=True, **proof), indent=2))
    # This is the ONLY cleanup call; unreachable on export/verification failure.
    receipt = remote('cleanup', ['--archive-hash', proof['archive_sha256'], '--manifest-hash', proof['manifest_sha256']])
    (output / 'CLEANUP_RECEIPT.json').write_text(json.dumps(receipt, indent=2))
    print('Verified local export and narrow cleanup complete:', output)


if __name__ == '__main__':
    main()
