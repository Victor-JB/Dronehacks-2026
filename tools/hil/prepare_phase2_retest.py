"""LOCAL ONLY. Freeze source/control artifacts from a specific committed baseline."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import zipfile


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--commit',required=True); p.add_argument('--output',required=True)
    p.add_argument('--course',required=True)
    a=p.parse_args()
    if not re.fullmatch('[0-9a-f]{40}',a.commit): raise RuntimeError('Full frozen commit required')
    repo=Path(__file__).resolve().parents[2]
    out=Path(a.output).resolve(); out.mkdir(parents=True,exist_ok=False)
    def git(*args): return subprocess.check_output(['git',*args],cwd=repo)
    if git('rev-parse',a.commit).decode().strip()!=a.commit: raise RuntimeError('Unknown commit')
    files={}
    paths=git('ls-tree','-r','--name-only',a.commit,'tools/hil/stationary_phase2').decode().splitlines()
    for path in paths:
        if path.endswith(('.py','.sh')): files[Path(path).name]=git('show',a.commit+':'+path)
    for path in ('PHASE2_RETEST_PLAN.md','STATIONARY_DEPLOYMENT_PLAN.md'):
        files[path]=git('show',a.commit+':'+path)
    name='stationary-reviewed-'+a.commit[:12]+'.tar'
    files[name]=git('archive','--format=tar',a.commit,'ros2_ws/src/dh_core','ros2_ws/src/dh_autonomy','tools/hil/check_command_isolation.py')
    if hashlib.sha256(files[name]).hexdigest()=='da31d4b63ed05099d2be592d292c05cc4be142a699a96dbe15387f68db3d4f70':
        raise RuntimeError('Old source archive is forbidden')
    files['source_commit.txt']=(a.commit+'\n').encode()
    files['course.obj']=Path(a.course).read_bytes()
    payload={k:v for k,v in files.items() if k.endswith(('.py','.sh','.tar','.obj')) or k=='source_commit.txt'}
    files['SHA256SUMS.stationary-phase2']=''.join(hashlib.sha256(v).hexdigest()+'  '+k+'\n' for k,v in sorted(payload.items())).encode()
    files['REVIEW_PACKAGE_SHA256SUMS.txt']=''.join(hashlib.sha256(v).hexdigest()+'  '+k+'\n' for k,v in sorted(files.items())).encode()
    for k,v in files.items(): (out/k).write_bytes(v)
    archive=out.parent/'phase2_retest_candidate.zip'
    if archive.exists(): raise RuntimeError('Preserve existing retest ZIP')
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
        for k,v in sorted(files.items()): z.writestr(k,v)
    with zipfile.ZipFile(archive) as z:
        if z.testzip() is not None: raise RuntimeError('ZIP CRC failure')
        for k,v in files.items():
            if z.read(k)!=v: raise RuntimeError('ZIP byte mismatch')
    receipt=dict(source_commit=a.commit,source_archive=name,source_archive_sha256=hashlib.sha256(files[name]).hexdigest(),
                 zip=str(archive),zip_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),frozen=True,
                 ROS_Humble_executed=False,Jetson_accessed=False)
    (out/'FREEZE_RECEIPT.json').write_text(json.dumps(receipt,indent=2))
    print(json.dumps(receipt,indent=2))


if __name__=='__main__': main()
