"""One-line-per-bag survey: topics, duration, armed/offboard time, path length, extent."""
import sys, os, json, math, collections
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import mcapr, cdr
import numpy as np
for P in sys.argv[1:]:
    cnt = collections.Counter(); od = []; sky = []; t0 = None
    for t, s, lt, b in mcapr.read(P):
        cnt[t] += 1
        if t0 is None: t0 = lt
        T = (lt - t0) / 1e9
        if t == '/visual_slam/tracking/odometry':
            r = cdr.R(b); r.header(); r.string(); od.append((T, *r.f64(3)))
        elif t == '/skybeta/state':
            j = json.loads(cdr.R(b).string()); sky.append((T, bool(j.get('armed')), bool(j.get('offboard_active'))))
    od = np.array(od); sky = np.array(sky) if sky else np.zeros((0, 3))
    print('==', os.path.basename(P))
    if len(od):
        d = np.linalg.norm(np.diff(od[:, 1:3], axis=0), axis=1)
        print(f'  {od[-1,0]:.0f} s, path {d.sum():.1f} m, x {od[:,1].min():.2f}..{od[:,1].max():.2f}, '
              f'y {od[:,2].min():.2f}..{od[:,2].max():.2f}, z {od[:,3].min():.2f}..{od[:,3].max():.2f}')
    if len(sky):
        dt = np.diff(sky[:, 0], append=sky[-1, 0])
        print(f'  armed {dt[sky[:,1]>0].sum():.0f} s, offboard+armed {dt[(sky[:,1]>0)&(sky[:,2]>0)].sum():.0f} s')
    print('  ', ', '.join(f'{k.split("/")[-1] if len(k)>40 else k}:{v}' for k, v in sorted(cnt.items())))
