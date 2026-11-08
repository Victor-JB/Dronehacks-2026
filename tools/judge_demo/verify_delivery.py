"""Decode all delivered media and reconcile reported metrics with actual traces."""
import argparse,csv,hashlib,json,subprocess
from pathlib import Path
import cv2
import numpy as np
import imageio_ffmpeg


def verify(demo):
    results=json.loads((demo/'evidence/results.json').read_text()); report={'videos':{},'screenshots':{},'metrics':{}}
    manifest=json.loads((demo/'MEDIA_MANIFEST.json').read_text())
    for name,expected in [('main_demo.mp4',manifest['main_frames']),('highlight.mp4',manifest['highlight_frames'])]:
        path=demo/name
        result=subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-v','error','-i',str(path),'-f','null','-'],capture_output=True,text=True)
        if result.returncode or result.stderr.strip():raise RuntimeError('Video decode failed: '+result.stderr)
        cap=cv2.VideoCapture(str(path));fps=cap.get(cv2.CAP_PROP_FPS);count=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        if count!=expected or abs(fps-20)>.01:raise RuntimeError('Wrong media duration')
        for index in (0,count//2,count-1):
            cap.set(cv2.CAP_PROP_POS_FRAMES,index);ok,frame=cap.read()
            if not ok or frame.shape[:2]!=(1080,1920) or frame.mean()<5:raise RuntimeError('Unreadable video frame')
        cap.release();report['videos'][name]=dict(frames=count,fps=fps,duration_s=count/fps,full_decode_pass=True,sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    for path in sorted((demo/'screenshots').glob('*.png')):
        frame=cv2.imread(str(path))
        if frame is None or frame.shape[:2]!=(1080,1920):raise RuntimeError('Bad screenshot '+str(path))
        report['screenshots'][path.name]=dict(width=1920,height=1080,sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    with (demo/'evidence/path_and_commands.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['case','time','x','y','z','yaw','vx','vy','vz','state','replans'])
        for case,result in zip(('course','diagnostic'),results):
            rows=json.loads((demo/'evidence'/case/'trace.json').read_text())
            positions=np.array([r['position'] for r in rows]);commands=np.array([r['command'] for r in rows])
            for row in rows:writer.writerow([case,row['time'],*row['position'],row['yaw'],*row['command'],row['state'],row['replans']])
            distance=float(np.linalg.norm(np.diff(positions,axis=0),axis=1).sum())
            if case=='diagnostic':distance+=float(np.linalg.norm(np.array(result['final_position'])-positions[-1]))
            if abs(distance-result['distance_m'])>1e-8:raise RuntimeError('Distance mismatch')
            if max(r['replans'] for r in rows)!=result['replans']:raise RuntimeError('Replan mismatch')
            if np.linalg.norm(commands,axis=1).max()>.30000001:raise RuntimeError('Speed cap failed')
            if result['collision_count']!=0 or result['mission_complete']:raise RuntimeError('Collision/completion mismatch')
            if case=='course' and (distance!=0 or any(r['startup_ready'] for r in rows)):raise RuntimeError('Startup guard bypass')
            if case=='diagnostic' and not any(r['index']>=40 and r['state'].startswith('HOLD') and np.linalg.norm(r['command'])<1e-9 for r in rows):raise RuntimeError('No obstacle stop')
            report['metrics'][case]=dict(distance_reconciled_m=distance,replans=result['replans'],speed_cap_pass=True,mission_complete=False)
    (demo/'VERIFICATION.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--demo',type=Path,required=True);a=parser.parse_args();verify(a.demo)
