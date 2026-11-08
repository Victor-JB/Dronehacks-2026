"""Draft v2 routes from the 2026-11-08 data. Route frame = run2 = take-off spot, heading 0, x forward, y left."""
import sys, os, math, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from model import *

def section(k, x_lo=None, t_lo=None, t_hi=None):
    o = TR[k]; m = np.ones(len(o), bool)
    if t_lo is not None: m &= o[:, 0] >= t_lo
    if t_hi is not None: m &= o[:, 0] <= t_hi
    return o[m]

def centreline(a, b, step=0.05):
    """mean of two tracks of the same stretch (b already in the same direction as a), by arc length"""
    def rs(P):
        s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]; q = np.linspace(0, s[-1], 400)
        return np.c_[np.interp(q, s, P[:, 0]), np.interp(q, s, P[:, 1])]
    A, B = rs(a), rs(b)
    # match each point of A to the nearest of B (robust to different speeds)
    j = np.sqrt(((A[:, None] - B[None]) ** 2).sum(-1)).argmin(1)
    return 0.5 * (A + B[j]), np.linalg.norm(A - B[j], axis=1)

def dp(P, tol):
    """Douglas-Peucker"""
    if len(P) < 3: return P
    a, b = P[0], P[-1]; ab = b - a; n = np.linalg.norm(ab)
    d = np.abs(ab[0] * (P - a)[:, 1] - ab[1] * (P - a)[:, 0]) / n if n > 1e-9 else np.linalg.norm(P - a, axis=1)
    i = d.argmax()
    if d[i] > tol: return np.vstack([dp(P[:i + 1], tol)[:-1], dp(P[i:], tol)])
    return np.vstack([a, b])

if __name__ == '__main__':
    # times (s) of each stretch in the walkthroughs, from the 4 s prints
    wf = TR['walk_fwd']; wr = TR['walk_rev']
    # back half, forward direction: from just past the window (x < -0.3, far side) to pad 1
    f_sec = wf[(wf[:, 0] > 56) & (wf[:, 0] < 112)][:, 1:3]
    r_sec = wr[(wr[:, 0] > 60) & (wr[:, 0] < 84)][:, 1:3][::-1]          # reverse walk, flipped
    f_sec = f_sec[f_sec[:, 0] < -0.3]; r_sec = r_sec[r_sec[:, 0] < -0.3]
    C, gap = centreline(f_sec, r_sec)
    print('back half: the two walks are %.2f m apart on median, %.2f max (at %s)' % (np.median(gap), gap.max(), np.round(C[gap.argmax()], 2)))
    W = dp(C, 0.06)
    print('centreline waypoints (fwd, left):')
    for p in W: print('   (%.3f, %.3f)  free_dist %.2f' % (p[0], p[1], free_dist(p)[0]))
    np.save(os.path.join(H, 'back_half.npy'), W)
    np.save(os.path.join(H, 'back_half_gap.npy'), np.c_[C, gap])

def dp3(P, tol):
    """Douglas-Peucker in 3-D"""
    if len(P) < 3: return P
    a, b = P[0], P[-1]; ab = b - a; n = np.linalg.norm(ab)
    d = (np.linalg.norm(np.cross(ab, P - a), axis=1) / n) if n > 1e-9 else np.linalg.norm(P - a, axis=1)
    i = d.argmax()
    if d[i] > tol: return np.vstack([dp3(P[:i + 1], tol)[:-1], dp3(P[i:], tol)])
    return np.vstack([a, b])

def track_dist(P, k):
    T = TR[k][:, 1:3]; P = np.atleast_2d(P)
    return np.sqrt(((P[:, None] - T[None, ::2]) ** 2).sum(-1).min(1))

def back_half_from_walk_fwd(tol=0.05):
    wf = TR['walk_fwd']
    sec = wf[(wf[:, 0] > 56) & (wf[:, 0] < 111)]
    sec = sec[np.r_[True, np.linalg.norm(np.diff(sec[:, 1:3], axis=0), axis=1) > 0.003]]
    return dp(sec[:, 1:3], tol)

def wall_from_walk_fwd(tol=0.06):
    wf = TR['walk_fwd']
    sec = wf[(wf[:, 0] > 120) & (wf[:, 0] < 170)]
    sec = sec[np.r_[True, np.linalg.norm(np.diff(sec[:, 1:4], axis=0), axis=1) > 0.003]]
    return dp3(sec[:, 1:4], tol)
