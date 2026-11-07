"""Fit the course route and the sim's walls to a point scan of the real course.

    python3 sim/scan/fit_route_to_scan.py [--ply sim/scan/dronehacks-course-scan-2026-11-06.ply]
                                          [--route-out route.json] [--mesh-out sim/models/course_v3/meshes/scan_walls.obj]

Needs numpy and opencv (cv2) only. Steps, each printed as it runs:
 1. Level the scan: RANSAC the floor plane, rotate it to z = 0.
 2. Register the course CAD (course_v3 wall meshes) to the scan: every yaw at 1 deg with an FFT
    translation search, then a chamfer refine at 2 cm. Prints the runner-up yaws so an ambiguous
    fit is visible. Everything after this is in the CAD frame (metres, course centre, Z up).
 3. Measure the window opening in the scan (the CAD one is wrong: higher and smaller in reality).
 4. Obstacles at flight height: scan cells at HEIGHT_M +- BAND_M with >= 3 points, plus the nets
    (the scan does not see them) and walls listed in KEEP_FROM_CAD (in the CAD, not in the scan),
    minus EXCLUDE boxes (things that were in the course only during the scan).
 5. Move each CAD waypoint (within its limit) to maximise the clearance of its two legs, sweeping
    until nothing moves. The window waypoints are put on the measured opening's axis instead.
 6. Write route.json (forward/left from the launch mark, refnav's frame) and the scan wall mesh
    (5 cm voxels merged into boxes) for the sim, and print the per-leg clearances.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
COURSE = HERE.parent / 'models' / 'course_v3'

LAUNCH = np.array([-2.5146, 0.9144])            # launch mark, CAD frame; takeoff faces +Y
HEIGHT_M, BAND_M = 1.15, 0.12                   # flight height; drone half-height 0.07 + altitude error 0.05
REACH_M = 0.29                                  # Sky Beta: rotor centre 0.20 m out + 7 in prop (sim/skybeta/AIRFRAME.md)
NET = 3.048
KEEP_FROM_CAD = ('c2b_chicane_wall',)           # not in the scan: kept, to be safe
EXCLUDE = [(-2.0, -1.2, 2.1, 2.7)]              # x0, x1, y0, y1: a person-shaped column in the maze (0.37 m wide,
                                                # 0.6-1.8 m tall on sparse "legs"); check on site
# The sim's surveyed CAD route (was sim/mission/course-window-x500.json): (x, y, phase, move limit m)
CAD_ROUTE = [(-2.5146, 0.9144, 'TAKEOFF', 0), (-2.5146, 2.58445, 'FIRST_CORRIDOR', .8), (-1.3716, 2.58445, 'MAZE', .8),
             (-1.3716, 0.97155, 'MAZE', .8), (-0.254, 0.97155, 'MAZE', .8), (-0.254, 2.58445, 'MAZE', .8),
             (0.95, 2.58445, 'GATE_A', .8), (2.3114, 2.2, 'TUNNEL_ALIGN', .8), (2.3114, 1.6, 'TUNNEL_ALIGN', .3),
             (2.3114, -0.72, 'TUNNEL', .3), (2.13, -1.7, 'CHICANE', .3), (0.3, -1.9, 'WINDOW_APPROACH', .3),
             (0.045893, -2.460006, 'WINDOW_ALIGN', 0), (-0.987769, -2.083784, 'WINDOW_CROSS', 0),
             (-2.0422, -0.7925, 'LANDING_APPROACH', .15)]
WINDOW = dict(center=(-0.470938, -2.271895), axis=(0.3420201433256687, 0.9396926207859084),
              normal=(0.9396926207859084, -0.3420201433256687))      # CAD aperture: 0.914 x 0.914 at z 0.305-1.219
rng = np.random.default_rng(0)


def read_ply(path):
    data = Path(path).read_bytes()
    head = data[:data.index(b'end_header\n')].decode()
    if 'binary_little_endian' not in head:
        raise SystemExit('expected a binary little-endian PLY')
    props = [l.split()[-1] for l in head.splitlines() if l.startswith('property')]
    types = {'x': '<f4', 'y': '<f4', 'z': '<f4', 'red': 'u1', 'green': 'u1', 'blue': 'u1'}
    a = np.frombuffer(data[data.index(b'end_header\n') + 11:], np.dtype([(p, types[p]) for p in props]))
    return np.stack([a['x'], a['y'], a['z']], 1).astype(np.float64)


def level(P):
    low = P[P[:, 2] < np.percentile(P[:, 2], 30)]
    best = None
    for _ in range(300):
        s = low[rng.choice(len(low), 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        k = (np.abs((low - s[0]) @ n) < 0.02).sum()
        if best is None or k > best[0]:
            best = (k, n if n[2] > 0 else -n, s[0])
    _, n, p0 = best
    v, c = np.cross(n, [0, 0, 1.0]), n[2]
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    R = np.eye(3) + vx + vx @ vx / (1 + c)
    Q = P @ R.T
    Q[:, 2] -= (p0 @ R.T)[2]
    print(f'1. floor: {best[0]} inlier points, tilt {np.degrees(np.arccos(c)):.2f} deg removed')
    return Q


def load_obj(path):
    V, F = [], []
    for line in open(path):
        if line.startswith('v '):
            V.append([float(x) for x in line.split()[1:4]])
        elif line.startswith('f '):
            F.append([int(t.split('/')[0]) - 1 for t in line.split()[1:4]])
    return np.array(V), np.array(F)


def cad_walls(z0=0.6, z1=1.5):
    """Surface samples (x, y) of each collidable CAD wall between z0 and z1."""
    parts = {}
    for g in json.load(open(COURSE / 'groups.json'))['groups']:
        n = g['name']
        if not g['collidable'] or n.startswith('net') or 'cradle' in n or 'boulder' in n:
            continue
        V, F = load_obj(COURSE / g['mesh'])
        pts = []
        for f in F:
            a, b, c = V[f]
            k = max(1, int(np.linalg.norm(np.cross(b - a, c - a)) / 2 * 400))
            u, v = rng.random((2, k))
            m = u + v > 1
            u[m], v[m] = 1 - u[m], 1 - v[m]
            pts.append(a + np.outer(u, b - a) + np.outer(v, c - a))
        X = np.vstack(pts)
        parts[n] = X[(X[:, 2] > z0) & (X[:, 2] < z1)][:, :2]
    return parts


def register(Q, parts):
    """Scan (levelled) -> CAD frame, as a function of 2D points."""
    T = np.vstack(list(parts.values()))
    M = T.mean(0)
    S = Q[(Q[:, 2] > 0.6) & (Q[:, 2] < 1.5)][:, :2]
    res = 0.05
    lo = S.min(0) - 1
    shape = np.ceil((S.max(0) + 1 - lo) / res).astype(int)[::-1]
    occ = np.zeros(shape)
    ij = ((S - lo) / res).astype(int)
    np.add.at(occ, (ij[:, 1], ij[:, 0]), 1)
    occ = cv2.blur(np.minimum(occ, 5) / 5, (3, 3))
    Fo = np.fft.rfft2(occ)
    scores = []
    for yaw in np.radians(np.arange(0, 360, 1.0)):
        c, s = np.cos(yaw), np.sin(yaw)
        X = (T - M) @ np.array([[c, -s], [s, c]]).T
        tij = ((X - X.min(0)) / res).astype(int)
        tmp = np.zeros_like(occ)
        np.add.at(tmp, (tij[:, 1], tij[:, 0]), 1)
        corr = np.fft.irfft2(Fo * np.conj(np.fft.rfft2(tmp)), s=occ.shape)
        iy, ix = np.unravel_index(np.argmax(corr), corr.shape)
        scores.append((corr[iy, ix] / len(T), yaw, np.array([ix, iy]) * res + lo - X.min(0)))
    scores.sort(key=lambda t: -t[0])
    _, yaw, t = scores[0]
    # chamfer refine at 2 cm
    res = 0.02
    lo = S.min(0) - 1
    shape = np.ceil((S.max(0) + 1 - lo) / res).astype(int)[::-1]
    cnt = np.zeros(shape, np.uint16)
    ij = ((S - lo) / res).astype(int)
    np.add.at(cnt, (ij[:, 1], ij[:, 0]), 1)
    dist = cv2.distanceTransform(np.where(cnt >= 2, 0, 255).astype(np.uint8), cv2.DIST_L2, 5) * res

    def xf(P, p):
        c, s = np.cos(p[0]), np.sin(p[0])
        return (P - M) @ np.array([[c, -s], [s, c]]).T + p[1:]

    def cost(p):
        ij = ((xf(T, p) - lo) / res).astype(int)
        ok = (ij >= 0).all(1) & (ij[:, 0] < dist.shape[1]) & (ij[:, 1] < dist.shape[0])
        d = np.full(len(T), 0.15)
        d[ok] = np.minimum(dist[ij[ok, 1], ij[ok, 0]], 0.15)
        return d.mean()

    p, step = np.array([yaw, *t]), np.array([np.radians(2), 0.1, 0.1])
    c = cost(p)
    while step[1] > 0.001:
        better = False
        for k in range(3):
            for sg in (1, -1):
                q = p.copy()
                q[k] += sg * step[k]
                if cost(q) < c:
                    p, c, better = q, cost(q), True
        if not better:
            step /= 2
    others = [cost(p + [np.radians(a), 0, 0]) for a in (90, 180, 270)]
    print(f'2. CAD -> scan: yaw {np.degrees(p[0]):.2f} deg, mean wall distance {c:.3f} m '
          f'(the same fit turned 90/180/270 deg: {", ".join(f"{o:.3f}" for o in others)})')
    for name, X in parts.items():
        ij = ((xf(X, p) - lo) / res).astype(int)
        med = np.median(dist[ij[:, 1], ij[:, 0]])
        if med > 0.1:
            print(f'   {name}: {med:.2f} m from the scan, the real one is elsewhere (or absent)')
    c_, s_ = np.cos(p[0]), np.sin(p[0])
    R = np.array([[c_, -s_], [s_, c_]])
    return lambda XY: (XY - p[1:]) @ R + M


def window_axis(P):
    """Centre (u along the wall, z) of the real window opening, from points within 12 cm of its plane."""
    c, h, n = (np.array(WINDOW[k]) for k in ('center', 'axis', 'normal'))
    rel = P[:, :2] - c
    u, d = rel @ h, rel @ n
    s = (np.abs(d) < 0.12) & (np.abs(u) < 1.0) & (P[:, 2] > 0.05) & (P[:, 2] < 1.9)
    res = 0.04
    G = np.zeros((int(1.9 / res) + 1, int(2.0 / res) + 1), int)
    np.add.at(G, ((P[s, 2] / res).astype(int), ((u[s] + 1.0) / res).astype(int)), 1)
    solid = G >= 4
    # per row, the widest run of empty cells walled on BOTH sides (an open row above the wall is not
    # the opening); the opening is the longest block of consecutive rows that have one
    rows = []
    for r in range(G.shape[0]):
        best, run = None, 0
        for i in range(1, G.shape[1]):
            if not solid[r, i]:
                run += 1
            else:
                if run and solid[r, i - run - 1] and (best is None or run > best[0]):
                    best = (run, i - run, i - 1)
                run = 0
        rows.append(best if best and 0.3 < best[0] * res < 1.5 else None)
    blocks, cur = [], []
    for r, b in enumerate(rows):
        if b:
            cur.append((r * res, b[1] * res - 1.0, (b[2] + 1) * res - 1.0))
        elif cur:
            blocks.append(cur)
            cur = []
    blocks.append(cur)
    block = max(blocks, key=len)
    z0, z1 = block[0][0], block[-1][0] + res
    u0, u1 = np.median([r[1] for r in block]), np.median([r[2] for r in block])
    print(f'3. window opening: z {z0:.2f}-{z1:.2f} m, {u1 - u0:.2f} m wide, centre {(u0 + u1) / 2:+.2f} m along the wall '
          f'from the CAD centre (CAD: 0.30-1.22 m, 0.91 m wide)')
    if not z0 + REACH_M / 4 < HEIGHT_M < z1 - REACH_M / 4:
        print(f'   WARNING: HEIGHT_M {HEIGHT_M} is not inside the opening with margin')
    inside = s & (u > u0) & (u < u1) & (P[:, 2] > z0) & (P[:, 2] < z1)
    if inside.sum() > 10:
        print(f'   {inside.sum()} points inside the opening at z ~{np.median(P[inside, 2]):.2f} m: a crossbar, tape, '
              f'or scan noise? check on site')
    return c + (u0 + u1) / 2 * h, n


def clearance(O, a, b):
    d = b - a
    t = np.clip(((O - a) @ d) / (d @ d), 0, 1)
    return np.sqrt((((a + np.outer(t, d)) - O) ** 2).sum(1)).min()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ply', default=HERE / 'dronehacks-course-scan-2026-11-06.ply')
    ap.add_argument('--route-out', default='route.json')
    ap.add_argument('--mesh-out', default=COURSE / 'meshes' / 'scan_walls.obj')
    a = ap.parse_args()

    Q = level(read_ply(a.ply))
    parts = cad_walls()
    to_cad = register(Q, parts)
    P = np.hstack([to_cad(Q[:, :2]), Q[:, 2:3]])
    centre, normal = window_axis(P)

    band = P[(np.abs(P[:, 2] - HEIGHT_M) < BAND_M) & (np.abs(P[:, :2]) < NET + 0.05).all(1)][:, :2]
    k, n = np.unique(np.floor(band / 0.05).astype(int), axis=0, return_counts=True)
    O = (k[n >= 3] + 0.5) * 0.05
    for x0, x1, y0, y1 in EXCLUDE:
        O = O[~((O[:, 0] > x0) & (O[:, 0] < x1) & (O[:, 1] > y0) & (O[:, 1] < y1))]
    g = np.arange(-NET, NET + 1e-9, 0.05)
    O = np.vstack([O, np.c_[g, g * 0 + NET], np.c_[g, g * 0 - NET], np.c_[g * 0 + NET, g], np.c_[g * 0 - NET, g]]
                  + [parts[k][::5] for k in KEEP_FROM_CAD])
    print(f'4. obstacles at {HEIGHT_M:.2f} +- {BAND_M:.2f} m: {len(O)} cells (scan, nets, {", ".join(KEEP_FROM_CAD)})')

    W0 = np.array([r[:2] for r in CAD_ROUTE], float)
    W = W0.copy()
    for i, r in enumerate(CAD_ROUTE):
        if r[2] in ('WINDOW_ALIGN', 'WINDOW_CROSS'):
            W[i] = centre + ((W0[i] - np.array(WINDOW['center'])) @ normal) * normal
    for _ in range(8):
        moved = 0
        for i, (_, _, _, lim) in enumerate(CAD_ROUTE):
            if not lim:
                continue
            best = None
            for dx in np.arange(-lim, lim + 1e-9, 0.04):
                for dy in np.arange(-lim, lim + 1e-9, 0.04):
                    q = W0[i] + [dx, dy]
                    cs = [clearance(O, W[i - 1], q)] + ([clearance(O, q, W[i + 1])] if i + 1 < len(W) else [])
                    score = min(min(cs), 0.6) - 0.03 * np.hypot(dx, dy)
                    if best is None or score > best[0] + 1e-9:
                        best = (score, q)
            if np.linalg.norm(best[1] - W[i]) > 1e-6:
                W[i], moved = best[1], moved + 1
        if not moved:
            break
    print(f'5. leg clearances (m, centre to the nearest real surface; the Sky Beta reaches {REACH_M}):')
    for i in range(1, len(W)):
        c0, c1 = clearance(O, W0[i - 1], W0[i]), clearance(O, W[i - 1], W[i])
        flag = '' if c1 > REACH_M + 0.03 else '   <- check'
        print(f'   {i:2d} {CAD_ROUTE[i][2]:17s} CAD {c0:.2f} -> fitted {c1:.2f}{flag}')

    route = [[round(float(p[1] - LAUNCH[1]), 3), round(float(-(p[0] - LAUNCH[0])), 3)] for p in W[1:]]
    Path(a.route_out).write_text(json.dumps({'route': route}) + '\n')
    print(f'6. route (forward/left from the launch mark, facing +Y) -> {a.route_out}; height {HEIGHT_M} m')
    write_mesh(P, a.mesh_out)


def write_mesh(P, out, V=0.05):
    """Scan surfaces inside the nets (floor and EXCLUDE removed) as 5 cm voxels merged into boxes."""
    m = (np.abs(P[:, :2]) < NET - 0.07).all(1) & (P[:, 2] > 0.06) & (P[:, 2] < 2.0)
    for x0, x1, y0, y1 in EXCLUDE:
        m &= ~((P[:, 0] > x0) & (P[:, 0] < x1) & (P[:, 1] > y0) & (P[:, 1] < y1))
    k, n = np.unique(np.floor(P[m] / V).astype(int), axis=0, return_counts=True)
    cols = {}
    for ix, iy, iz in k[n >= 3]:
        cols.setdefault((ix, iy), []).append(iz)
    segs = []
    for (ix, iy), zs in cols.items():
        zs = sorted(zs)
        s = e = zs[0]
        for z in zs[1:]:
            if z <= e + 2:
                e = z
            else:
                segs.append((iy, s, e, ix))
                s = e = z
        segs.append((iy, s, e, ix))
    segs.sort()
    boxes, cur = [], None
    for iy, s, e, ix in segs:
        if cur and cur[:3] == [iy, s, e] and cur[4] == ix - 1:
            cur[4] = ix
        else:
            if cur:
                boxes.append(cur)
            cur = [iy, s, e, ix, ix]
    boxes.append(cur)
    with open(out, 'w') as f:
        f.write('# Course walls from the point scan (sim/scan/fit_route_to_scan.py): 5 cm voxels merged into boxes\n')
        quads = ((0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3))
        for b, (iy, s, e, x0, x1) in enumerate(boxes):
            lo = np.array([x0, iy, s]) * V
            hi = np.array([x1 + 1, iy + 1, e + 1]) * V
            for c in range(8):
                f.write('v %.3f %.3f %.3f\n' % tuple(hi[j] if c >> (2 - j) & 1 else lo[j] for j in range(3)))
            for q in quads:
                i = [8 * b + v + 1 for v in q]
                f.write(f'f {i[0]} {i[1]} {i[2]}\nf {i[0]} {i[2]} {i[3]}\n')
    print(f'   scan walls: {len(boxes)} boxes -> {out}')


if __name__ == '__main__':
    main()
