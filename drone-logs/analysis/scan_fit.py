"""Scan (2026-11-06 Scaniverse .ply) -> route frame. 1) team's level + register to CAD (sim/scan/fit_route_to_scan.py),
2) CAD -> route by the launch mark (fwd = CAD +Y, left = CAD -X), 3) refine (dx, dy, dtheta) so today's tracks
(where the drone was, without hitting anything) sit in free space. Saves scan_route.npy (x, y, z) in the route frame."""
import sys, os, math, numpy as np
H = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, H); sys.path.insert(0, os.path.join(H, '..', '..', 'sim', 'scan'))
import fit_route_to_scan as F
from model import TR
from scipy.spatial import cKDTree

Q = F.level(F.read_ply(F.HERE / 'dronehacks-course-scan-2026-11-06.ply'))
to_cad = F.register(Q, F.cad_walls())
C = np.hstack([to_cad(Q[:, :2]), Q[:, 2:3]])
R0 = np.c_[C[:, 1] - F.LAUNCH[1], -(C[:, 0] - F.LAUNCH[0]), C[:, 2]]        # route frame, take-off = launch mark

def rot(a): c, s = math.cos(a), math.sin(a); return np.array([[c, -s], [s, c]])
# walls: scan points 0.5-1.6 m up, 5 cm cells with >= 3 points
band = R0[(R0[:, 2] > 0.5) & (R0[:, 2] < 1.6)][:, :2]
k, n = np.unique(np.floor(band / 0.05).astype(int), axis=0, return_counts=True)
WALL = (k[n >= 3] + 0.5) * 0.05
# track points at flight height (carried / flown between 0.6 and 1.5 m), every 5 cm
pts = []
for name, o in TR.items():
    m = (o[:, 3] > 0.6) & (o[:, 3] < 1.5)
    P = o[m, 1:3]
    keep = [0]
    for i in range(1, len(P)):
        if np.linalg.norm(P[i] - P[keep[-1]]) > 0.05: keep.append(i)
    pts.append(P[keep])
TP = np.vstack(pts)
print('track points', len(TP), 'wall cells', len(WALL))

def cost(p):
    dx, dy, da = p
    W = WALL @ rot(da).T + [dx, dy]
    d, _ = cKDTree(W).query(TP)
    pen = np.clip(0.22 - d, 0, None)          # a track point closer than 0.22 m to a wall is a contradiction
    return (pen ** 2).sum(), (d < 0.15).mean(), np.median(d)

best = None
for da in np.radians(np.arange(-15, 15.1, 1.5)):
    for dx in np.arange(-2.5, 2.51, 0.1):
        for dy in np.arange(-2.5, 2.51, 0.1):
            c = cost((dx, dy, da))
            if best is None or c[0] < best[0][0]: best = (c, (dx, dy, da))
(c, p) = best
for step, astep in ((0.03, 0.4), (0.01, 0.1)):
    for _ in range(30):
        improved = False
        for q in [(p[0] + sx * step, p[1] + sy * step, p[2] + sa * math.radians(astep)) for sx in (-1, 0, 1) for sy in (-1, 0, 1) for sa in (-1, 0, 1)]:
            cq = cost(q)
            if cq[0] < c[0] - 1e-9: c, p, improved = cq, q, True
        if not improved: break
print(f'refine: dx {p[0]:+.2f} dy {p[1]:+.2f} rot {math.degrees(p[2]):+.1f} deg  | track points within 0.15 m of a wall {c[1]*100:.1f}%, median clearance {c[2]:.2f} m, penalty {c[0]:.3f}')
print('before refine:', cost((0, 0, 0)))
XY = R0[:, :2] @ rot(p[2]).T + [p[0], p[1]]
S = np.c_[XY, R0[:, 2]]
np.save(os.path.join(H, 'scan_route.npy'), S.astype(np.float32))
np.save(os.path.join(H, 'scan_fit.npy'), np.array(p))
# where do tracks still touch walls? (course changes since 11-06)
W = WALL @ rot(p[2]).T + [p[0], p[1]]
d, _ = cKDTree(W).query(TP)
bad = TP[d < 0.15]
if len(bad):
    print('track points within 0.15 m of a 11-06 wall (changed since?), by area:')
    k, n = np.unique(np.round(bad / 0.5) * 0.5, axis=0, return_counts=True)
    for kk, nn in sorted(zip(map(tuple, k), n), key=lambda t: -t[1])[:10]: print('   near', kk, nn, 'points')
