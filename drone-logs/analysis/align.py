"""Rigid 2-D fit (rotation + translation) of each track onto run2 (the reference flight, heading 0 at
Offboard start, flown as commanded) over the part of the course they share, by ICP on the tracks."""
import pickle, math, os, numpy as np
H = os.path.dirname(__file__)
d = pickle.load(open(os.path.join(H, 'tracks.pkl'), 'rb'))

def xy(o, moving=True):
    p = o[:, 1:3]
    if moving:                     # drop the parts where it sits still (they would dominate)
        keep = np.r_[True, np.linalg.norm(np.diff(p, axis=0), axis=1) > 0.002]
        p = p[keep]
    # resample every 2 cm of path
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
    q = np.arange(0, s[-1], 0.02)
    return np.c_[np.interp(q, s, p[:, 0]), np.interp(q, s, p[:, 1])]

def rot(a): c, s = math.cos(a), math.sin(a); return np.array([[c, -s], [s, c]])

ref = xy(d['run2']['od'])
ref = ref[ref[:, 0] > -0.05] if False else ref      # run2 covers start -> window

def fit(src, ref, a0=0.0, iters=60, maxd=0.35):
    """move src onto ref: returns (angle, t) with p' = R p + t; uses src points near ref only"""
    a, t = a0, np.zeros(2)
    for _ in range(iters):
        P = src @ rot(a).T + t
        dd = np.linalg.norm(P[:, None, :] - ref[None, ::3, :], axis=2)
        j = dd.argmin(1); m = dd[np.arange(len(P)), j] < maxd
        A, B = src[m], ref[::3][j[m]]
        ca, cb = A.mean(0), B.mean(0)
        U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
        R = (U @ Vt).T
        if np.linalg.det(R) < 0: Vt[1] *= -1; R = (U @ Vt).T
        a, t = math.atan2(R[1, 0], R[0, 0]), cb - R @ ca
    P = src @ rot(a).T + t
    dd = np.linalg.norm(P[:, None, :] - ref[None, :, :], axis=2).min(1)
    return a, t, np.median(dd[dd < maxd]), (dd < maxd).mean()

out = {'run2': (0.0, np.zeros(2))}
for k in ('run1', 'walk_fwd', 'walk_rev'):
    src = xy(d[k]['od'])
    # only the part of the track that lies in run2's region (start .. window): x > -0.4 and y < 0.1
    src_shared = src[(src[:, 0] > -0.1) & (src[:, 1] > -5.3)]
    a0 = -d[k]['od'][0, 4]
    best = min((fit(src_shared, ref, a0=g) for g in (0.0, a0)), key=lambda r: r[2])
    out[k] = best[:2]
    print(f'{k:9s} rot {math.degrees(best[0]):+5.2f} deg  t ({best[1][0]:+.3f},{best[1][1]:+.3f})  '
          f'median dist {best[2]*100:.1f} cm  inlier {best[3]*100:.0f}%   (start yaw {math.degrees(-a0):+.1f})')
pickle.dump(out, open(os.path.join(H, 'align.pkl'), 'wb'))
