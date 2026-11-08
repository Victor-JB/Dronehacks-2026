"""Visual check of the v2 mission: top-down over the 2026-11-06 scan + the 2026-11-07 depth map + today's tracks,
and side views of the wall and both windows. python3 plot_check.py <route.yaml> <out.png>"""
import sys, os, math, numpy as np
H = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, H)
sys.path.insert(0, os.path.join(H, '..', '..', 'drone-pkgs', 'box_route_v2', 'box_route_v2'))
import yaml, mission_plan as MP
from model import TR
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
cfgp, out = sys.argv[1], sys.argv[2]
raw = yaml.safe_load(open(cfgp))
cfg = {'height_m': raw['height_m'], 'scale': raw.get('scale', 1.0), 'waypoints_to_fly': 0,
       'waypoints': [(w['fwd'], w['left'], w.get('wall', False), w.get('height'), w.get('straight', False)) for w in raw['waypoints']],
       'sections': MP.parse_sections(raw)}
st = MP.build_steps(cfg)
S = np.load(os.path.join(H, 'scan_route.npy')).astype(float); S = S[S[:, 2] < 3.0]
fig = plt.figure(figsize=(22, 13)); gs = fig.add_gridspec(3, 2, width_ratios=[1.35, 1])
ax = fig.add_subplot(gs[:, 0])
b = S[(S[:, 2] > 0.5) & (S[:, 2] < 1.6)]
ax.scatter(b[::3, 0], b[::3, 1], s=0.25, c='0.55', label='scan 11-06, 0.5-1.6 m')
t = S[(S[:, 2] > 1.6)]; ax.scatter(t[::2, 0], t[::2, 1], s=0.4, c='tab:red', alpha=.5, label='scan above 1.6 m (wall, tall parts)')
for k, c in dict(run2='tab:orange', walk_fwd='tab:blue', walk_rev='tab:green').items():
    ax.plot(TR[k][:, 1], TR[k][:, 2], color=c, lw=0.8, alpha=.7, label=k)
ph_col = dict(course='k', pad1='m', wall='tab:red', pad2='m', reverse='tab:purple')
P = np.array([[s['x'], s['y']] for s in st])
for i in range(1, len(st)):
    s = st[i]; ax.plot(P[i - 1:i + 1, 0], P[i - 1:i + 1, 1], color=ph_col[s['phase']], lw=2.6 if s['phase'] == 'course' else 1.6,
                       ls='-' if s['phase'] != 'reverse' else (0, (4, 2)), zorder=6)
for i, s in enumerate(st):
    if s['phase'] == 'course':
        ax.plot(s['x'], s['y'], 'o', color='k', ms=5, zorder=7)
        ax.annotate(f"{i}" + (f" {s['z']:.1f}m" if abs(s['z'] - cfg['height_m']) > 1e-6 else '') + (' S' if s['straight'] else ''),
                    (s['x'], s['y']), fontsize=9, xytext=(4, 4), textcoords='offset points', zorder=8)
for name, (x, y) in {'PAD 1': st[-1 if False else len(raw['waypoints'])]['x':'y'] if False else (P[len(raw['waypoints']), 0], P[len(raw['waypoints']), 1]),
                     'PAD 2 / take-off': (-0.128, 0.03)}.items():
    ax.add_patch(plt.Circle((x, y), 0.3, fill=False, color='m', lw=2)); ax.annotate(name, (x, y), xytext=(8, -14), textcoords='offset points', color='m')
ax.plot([], [], 'k', lw=2.6, label='course (S = straight / lined-up leg)'); ax.plot([], [], color='tab:red', label='wall (2.2 m)')
ax.plot([], [], color='tab:purple', ls='--', label='reverse')
ax.set_aspect('equal'); ax.set_xlim(-4.8, 2.0); ax.set_ylim(-6.0, 0.9); ax.grid(alpha=.3); ax.legend(loc='upper right', fontsize=8, markerscale=6)
ax.set_xlabel('x forward [m]'); ax.set_ylabel('y left [m]'); ax.set_title(f'v2 mission ({len(st)} steps) over the scan and today\'s tracks')
def side(axs, a, b, half, title, zpath=None, marks=()):
    a, b = np.array(a, float), np.array(b, float); L = np.linalg.norm(b - a); u = (b - a) / L; n = np.array([-u[1], u[0]])
    rel = S[:, :2] - a; s = rel @ u; d = rel @ n; m = (np.abs(d) < half) & (s > -0.3) & (s < L + 0.3)
    axs.scatter(s[m], S[m, 2], s=0.5, c='0.4')
    for k, c in dict(walk_fwd='tab:blue', walk_rev='tab:green', run2='tab:orange').items():
        o = TR[k]; r = o[:, 1:3] - a; ss = r @ u; dd = r @ n; mm = (np.abs(dd) < 0.4) & (ss > -0.3) & (ss < L + 0.3)
        axs.plot(ss[mm], o[mm, 3], '.', ms=1.5, color=c)
    if zpath is not None:
        axs.plot(zpath[:, 0], zpath[:, 1], 'r-', lw=2.5, label='v2 plan (drone centre)')
        axs.fill_between(zpath[:, 0], zpath[:, 1] - 0.08, zpath[:, 1] + 0.08, color='r', alpha=.15)
    for x, lab in marks: axs.axvline(x, color='m', ls=':'); axs.annotate(lab, (x, 0.1), color='m', fontsize=8)
    axs.set_title(title, fontsize=10); axs.set_ylim(0, 2.8); axs.grid(alpha=.3); axs.set_ylabel('up [m]')
# 1. wall: along y = -0.6 from x -3.4 to 0
wsteps = [s for s in st if s['phase'] == 'wall'][:len(cfg['sections']['wall']['waypoints'])]
zp = np.array([[s['x'] + 3.4, s['z']] for s in wsteps]) if wsteps else None
side(fig.add_subplot(gs[0, 1]), (-3.4, -0.6), (0.0, -0.6), 0.2, 'WALL: side view along y = -0.60 (scan +-0.2 m; walks in blue/green). x axis = fwd + 3.4', zp,
     marks=((-1.75 + 3.4, 'earliest face'), (-1.15 + 3.4, 'latest face')))
# 2. window 2: the wall line, crossing point
wa, wb = (-4.6, -2.55), (-2.9, -2.92)
c10 = [s for s in st if s['label'] == 'course 10'][0]
side(fig.add_subplot(gs[1, 1]), wa, wb, 0.12, 'WINDOW 2 (run 2 hit it, v1 at 1.11 m along): along its wall (scan +-0.12 m); red = v2 crossing (0.79 m along, 1.2 m)',
     np.array([[0.79 - 0.29, c10['z']], [0.79 + 0.29, c10['z']]]), marks=((1.11, 'v1 crossing (run 2 hit)'),))
# 3. window 1: across the far side at x -0.1
side(fig.add_subplot(gs[2, 1]), (-0.1, -5.8), (-0.1, -4.2), 0.12, 'BRIDGE: across the far side at x -0.1 (scan +-0.12 m), from y -5.8; red = route (y -4.86)',
     np.array([[5.8 - 4.86 - 0.29, cfg['height_m']], [5.8 - 4.86 + 0.29, cfg['height_m']]]), marks=((5.8 - 5.00, 'run 2 (passed)'),))
plt.tight_layout(); plt.savefig(out, dpi=72); print('wrote', out, len(st), 'steps:', MP.describe(st))
