import sys, os, math, numpy as np
H = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, H); sys.path.insert(0, os.path.join(H, '..', '..', 'sim', 'scan'))
from model import TR
from scipy.spatial import cKDTree
from scipy.signal import fftconvolve
def rot(a): c, s = math.cos(a), math.sin(a); return np.array([[c, -s], [s, c]])
def cells(P, z0=0.5, z1=1.5, res=0.05, nmin=3):
    b = P[(P[:, 2] > z0) & (P[:, 2] < z1)][:, :2]
    k, n = np.unique(np.floor(b / res).astype(int), axis=0, return_counts=True)
    return (k[n >= nmin] + 0.5) * res
D = np.load(os.path.join(H, 'depth1107_odom.npy'))[:, :3].astype(np.float64)
Dw = cells(D, nmin=4)
# today's track points at flight height
TP = np.vstack([o[(o[:, 3] > 0.6) & (o[:, 3] < 1.5), 1:3][::3] for o in TR.values()])
def pen(W, P=TP, m=0.22):
    d, _ = cKDTree(W).query(P); return (np.clip(m - d, 0, None) ** 2).sum(), (d < 0.15).mean()
# 1. 11-07 -> today: start at (-0.3048, 0, 0), local search
best = None
for da in np.radians(np.arange(-4, 4.01, 0.5)):
    for dx in np.arange(-0.7, 0.11, 0.05):
        for dy in np.arange(-0.3, 0.31, 0.05):
            c = pen(Dw @ rot(da).T + [dx, dy])
            if best is None or c[0] < best[0][0]: best = (c, (dx, dy, da))
(c, p1) = best
print(f'11-07 -> today: dx {p1[0]:+.2f} dy {p1[1]:+.2f} rot {math.degrees(p1[2]):+.1f} deg; today\'s track points within 0.15 m of an 11-07 wall: {c[1]*100:.1f}%  (at the v1 guess -0.305,0,0: {pen(Dw + [-0.3048, 0])[1]*100:.1f}%)')
Dr = np.c_[D[:, :2] @ rot(p1[2]).T + [p1[0], p1[1]], D[:, 2]]
Dwr = Dw @ rot(p1[2]).T + [p1[0], p1[1]]
np.save(os.path.join(H, 'depth1107_route.npy'), Dr.astype(np.float32)); np.save(os.path.join(H, 'fit_1107.npy'), np.array(p1))
# 2. scan -> 11-07 depth walls (route frame): global search
import fit_route_to_scan as F
Q = F.level(F.read_ply(F.HERE / 'dronehacks-course-scan-2026-11-06.ply'))
Sw = cells(Q, nmin=3)
res = 0.05
lo = Dwr.min(0) - 4; hi = Dwr.max(0) + 4
G = np.zeros(np.ceil((hi - lo) / res).astype(int) + 1)
ij = np.floor((Dwr - lo) / res).astype(int); G[ij[:, 0], ij[:, 1]] = 1
from scipy.ndimage import gaussian_filter
Gs = gaussian_filter(G, 1.5)
cand = []
for deg in np.arange(0, 360, 2.0):
    S = Sw @ rot(math.radians(deg)).T
    S0 = S.min(0); ij = np.floor((S - S0) / res).astype(int)
    K = np.zeros(ij.max(0) + 1); K[ij[:, 0], ij[:, 1]] = 1
    corr = fftconvolve(Gs, K[::-1, ::-1], mode='full')
    k = np.unravel_index(corr.argmax(), corr.shape)
    # offset: grid index of S0
    off = (np.array(k) - (np.array(K.shape) - 1)) * res + lo - S0
    cand.append((corr.max() / len(Sw), deg, off))
cand.sort(key=lambda c: -c[0])
print('scan -> route, best rotations (score, deg):', [(round(c[0], 3), c[1]) for c in cand[:5]])
# ICP refine the top 3, pick the one with the best inlier fraction
def icp(S, a, t, iters=40, maxd=0.25):
    tree = cKDTree(Dwr)
    for _ in range(iters):
        P = S @ rot(a).T + t; d, j = tree.query(P); m = d < maxd
        A, B = S[m], Dwr[j[m]]; ca, cb = A.mean(0), B.mean(0)
        U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb)); R = (U @ Vt).T
        a = math.atan2(R[1, 0], R[0, 0]); t = cb - R @ ca
    P = S @ rot(a).T + t; d, _ = tree.query(P)
    return a, t, (d < 0.1).mean(), np.median(d)
res_ = [icp(Sw, math.radians(c[1]), c[2]) for c in cand[:4]]
for r in res_: print(f'   icp: rot {math.degrees(r[0]):7.2f} deg t ({r[1][0]:+.2f},{r[1][1]:+.2f})  scan wall cells within 0.10 m of an 11-07 wall {r[2]*100:.0f}%  median {r[3]:.3f} m')
a, t, f, _ = max(res_, key=lambda r: r[2])
S = np.c_[Q[:, :2] @ rot(a).T + t, Q[:, 2]]
np.save(os.path.join(H, 'scan_route.npy'), S.astype(np.float32)); np.save(os.path.join(H, 'scan_fit.npy'), np.array([t[0], t[1], a]))
print('scan in route frame: today\'s track points within 0.15 m of a scan wall:', f'{pen(cells(S) )[1]*100:.1f}%')
