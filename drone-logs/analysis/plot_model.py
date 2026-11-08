import sys, os, math, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from model import *
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
V1 = [(0, 0), (0.875, -0.26), (1.145, -4.1), (0.885, -4.81), (0.555, -5.04), (-0.215, -4.82), (-1.645, -5.09),
      (-3.035, -4.68), (-3.5, -3.52), (-3.465, -3.29), (-3.545, -1.98), (-3.175, -1.53), (-3.335, -0.93), (-3.355, -0.45)]
col = dict(run1='tab:red', run2='tab:orange', walk_fwd='tab:blue', walk_rev='tab:green')
def draw(ax, lim=None, route=V1, label='v1 route'):
    r = np.array(route); ax.plot(r[:, 0], r[:, 1], 'k--o', ms=4, lw=1, label=label, zorder=5)
    for i, (x, y) in enumerate(route): ax.annotate(str(i), (x, y), fontsize=9, xytext=(4, 4), textcoords='offset points', zorder=6)
    for k, o in TR.items():
        ax.plot(o[:, 1], o[:, 2], color=col[k], lw=1.1, label=k)
        ax.plot(o[-1, 1], o[-1, 2], 'x', color=col[k], ms=11, mew=2)
    for k, (p, yaw) in CONTACT.items():
        ax.add_patch(plt.Circle(p, PROP_R, fill=False, color='r', lw=1.5, ls=':'))
    ax.set_aspect('equal'); ax.grid(alpha=.3)
    if lim: ax.set_xlim(*lim[0]); ax.set_ylim(*lim[1])
fig, ax = plt.subplots(1, 2, figsize=(20, 10), gridspec_kw=dict(width_ratios=[1.3, 1]))
draw(ax[0]); ax[0].legend(loc='center'); ax[0].set_title('all tracks in the route frame (walks rotated by their start heading)\nx = where each bag STOPS (recording cut off; both flights went further)')
draw(ax[1], lim=((-0.8, 1.7), (-5.5, -3.6))); ax[1].set_title('tunnel exit and window (x forward, y left)')
for a in ax: a.set_xlabel('x forward [m]'); a.set_ylabel('y left [m]')
plt.tight_layout(); plt.savefig(os.path.join(H, 'model_top.png'), dpi=70)
# window crossing table
for k in ('run2', 'walk_fwd', 'walk_rev'):
    o = TR[k]; P = o[:, 1:3]; m = P[:, 1] < -4.4
    for xc in (0.6, 0.4, 0.3, 0.2, 0.0, -0.2):
        Q = P[m]; i = np.where(np.diff(np.sign(Q[:, 0] - xc)) != 0)[0]
        print(f'{k:9s} x={xc:+.1f}: y {Q[i, 1].round(2).tolist()}', end='   ')
    print()
