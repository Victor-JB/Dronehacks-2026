"""Course model from the 2026-11-08 morning data, all in run2's frame (= the route frame: x forward, y left
from the take-off spot, heading 0). Walkthroughs rotated by -(their start heading) about their start
(their two independent tracks then agree on pad 1 to 4 cm and on the far side to 2 cm)."""
import pickle, math, os, numpy as np
H = os.path.dirname(os.path.abspath(__file__))
d = pickle.load(open(os.path.join(H, 'tracks.pkl'), 'rb'))
def rot(a): c, s = math.cos(a), math.sin(a); return np.array([[c, -s], [s, c]])
def aligned(k):
    o = d[k]['od'].copy()
    if k.startswith('walk'):
        p0, a = o[0, 1:3].copy(), -o[0, 4]
        o[:, 1:3] = (o[:, 1:3] - p0) @ rot(a).T
        o[:, 4] = o[:, 4] + a
    return o
TR = {k: aligned(k) for k in d}
FREE = np.vstack([TR[k][:, 1:3] for k in ('run2', 'walk_fwd', 'walk_rev')] + [TR['run1'][TR['run1'][:, 0] < TR['run1'][-1, 0] - 1.5, 1:3]])
PROP_R = 0.29          # props reach 0.29 m from the centre
CONTACT = {'run1 (tunnel exit)': (TR['run1'][-1, 1:3], TR['run1'][-1, 4]), 'run2 (window)': (TR['run2'][-1, 1:3], TR['run2'][-1, 4])}
def free_dist(P):
    """distance from each point to the nearest place the drone has been without hitting anything"""
    P = np.atleast_2d(P)
    return np.sqrt(((P[:, None, :] - FREE[None, ::2, :]) ** 2).sum(-1).min(1))
def leg_points(a, b, step=0.03):
    n = max(2, int(math.dist(a, b) / step)); t = np.linspace(0, 1, n)[:, None]
    return np.asarray(a) * (1 - t) + np.asarray(b) * t
