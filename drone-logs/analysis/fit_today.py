"""Scan (already on the 11-07 walk frame) -> today's route frame: small rigid fit so today's tracks sit in the
scan's free space. Then saves scan_route.npy in today's frame."""
import sys, os, math, numpy as np
H = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, H)
from model import TR
from scipy.spatial import cKDTree
def rot(a): c, s = math.cos(a), math.sin(a); return np.array([[c, -s], [s, c]])
p1 = np.load(os.path.join(H, 'fit_1107.npy'))
S = np.load(os.path.join(H, 'scan_route.npy')).astype(np.float64)
S[:, :2] = (S[:, :2] - [p1[0], p1[1]]) @ rot(-p1[2]).T                   # undo step 1: back in the 11-07 frame
np.save(os.path.join(H, 'scan_1107.npy'), S.astype(np.float32))
def cells(P, z0, z1, res=0.05, nmin=3):
    b = P[(P[:, 2] > z0) & (P[:, 2] < z1)][:, :2]
    k, n = np.unique(np.floor(b / res).astype(int), axis=0, return_counts=True)
    return (k[n >= nmin] + 0.5) * res
SW = cells(S, 0.5, 1.5)
# per-track points at their own heights (the carried / flown drone at 0.6-1.5 m)
TP = np.vstack([o[(o[:, 3] > 0.6) & (o[:, 3] < 1.5), 1:3][::3] for k, o in TR.items()])
def score(p, m=0.25):
    W = SW @ rot(p[2]).T + [p[0], p[1]]; d, _ = cKDTree(W).query(TP)
    return (np.clip(m - d, 0, None) ** 2).sum(), (d < 0.15).mean(), np.percentile(d, 5)
best = None
for da in np.radians(np.arange(-6, 6.01, 0.5)):
    for dx in np.arange(-0.8, 0.51, 0.05):
        for dy in np.arange(-0.5, 0.51, 0.05):
            c = score((dx, dy, da))
            if best is None or c[0] < best[0][0]: best = (c, (dx, dy, da))
c, p = best
print(f'11-07 -> today (via the scan): dx {p[0]:+.2f} dy {p[1]:+.2f} rot {math.degrees(p[2]):+.1f} deg | track points < 0.15 m from a scan wall {c[1]*100:.1f}%, 5th pct clearance {c[2]:.2f} m')
print(f'   at the v1 rule (-0.305, 0, 0): {score((-0.3048, 0, 0))[1]*100:.1f}%;  identity: {score((0, 0, 0))[1]*100:.1f}%')
S2 = S.copy(); S2[:, :2] = S[:, :2] @ rot(p[2]).T + [p[0], p[1]]
np.save(os.path.join(H, 'scan_route.npy'), S2.astype(np.float32)); np.save(os.path.join(H, 'fit_today.npy'), np.array(p))
W = SW @ rot(p[2]).T + [p[0], p[1]]; d, _ = cKDTree(W).query(TP); bad = TP[d < 0.15]
k, n = np.unique(np.round(bad / 0.5) * 0.5, axis=0, return_counts=True)
print('where tracks still touch an 11-06 wall (course changed there?):', sorted([(tuple(kk), int(nn)) for kk, nn in zip(k, n)], key=lambda t: -t[1])[:8])
