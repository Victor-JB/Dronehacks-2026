import sys, os, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from model import TR
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
S = np.load('scan_route.npy')
V2 = [(0,0),(0.875,-0.26),(1.145,-4.10),(0.885,-4.81),(-0.275,-4.90),(-2.056,-4.897),(-3.089,-5.104),(-3.532,-5.144),
      (-3.755,-4.931),(-3.932,-3.859),(-3.808,-2.553),(-3.639,-1.921),(-3.807,-1.430),(-3.668,-0.940),(-3.347,-0.386)]
fig, ax = plt.subplots(figsize=(13, 13))
b = S[(S[:, 2] > 0.5) & (S[:, 2] < 1.6)]
ax.scatter(b[::3, 0], b[::3, 1], s=0.3, c=b[::3, 2], cmap='viridis', vmin=0.5, vmax=2.2)
t = S[S[:, 2] > 1.6]; ax.scatter(t[::2, 0], t[::2, 1], s=0.5, c='r', label='scan above 1.6 m')
col = dict(run1='tab:red', run2='tab:orange', walk_fwd='tab:blue', walk_rev='tab:green')
for k, o in TR.items(): ax.plot(o[:, 1], o[:, 2], color=col[k], lw=1, label=k)
r = np.array(V2); ax.plot(r[:, 0], r[:, 1], 'k-o', ms=4, lw=1.5, label='v2 route')
for i, (x, y) in enumerate(V2): ax.annotate(str(i), (x, y), fontsize=10, xytext=(4, 4), textcoords='offset points')
ax.set_aspect('equal'); ax.set_xlim(-5, 2.5); ax.set_ylim(-6.2, 1.3); ax.grid(alpha=.3); ax.legend(loc='upper right', markerscale=8)
ax.set_title('2026-11-06 scan (walls 0.5-1.6 m, coloured by height; red = above 1.6 m) fitted to today\'s tracks, v2 route')
ax.set_xlabel('x forward [m]'); ax.set_ylabel('y left [m]')
plt.tight_layout(); plt.savefig('scan_top.png', dpi=75)
