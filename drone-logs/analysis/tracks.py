"""Top-down + side plots of the 2026-11-08 runs and walkthroughs against the v1 route, in each bag's odom frame."""
import sys, os, json, struct, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import mcapr, cdr
import numpy as np
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
D = os.path.join(os.path.dirname(__file__), '..', '1108')
BAGS = {'run1': '20261108-055027', 'run2': '20261108-055648', 'walk_fwd': '20261108-065240', 'walk_rev': '20261108-065751'}
ROUTE = [(0, 0), (0.875, -0.26), (1.145, -4.1), (0.885, -4.81), (0.555, -5.04), (-0.215, -4.82), (-1.645, -5.09),
         (-3.035, -4.68), (-3.5, -3.52), (-3.465, -3.29), (-3.545, -1.98), (-3.175, -1.53), (-3.335, -0.93), (-3.355, -0.45)]

def load(name):
    P = os.path.join(D, BAGS[name], BAGS[name] + '_0.mcap')
    od, lp, att, sky = [], [], [], []
    t0 = None
    for t, s, lt, b in mcapr.read(P, {'/visual_slam/tracking/odometry', '/fmu/out/vehicle_local_position',
                                      '/fmu/out/vehicle_attitude', '/skybeta/state'}):
        if t0 is None: t0 = lt
        T = (lt - t0) / 1e9
        if t == '/visual_slam/tracking/odometry':
            r = cdr.R(b); r.header(); r.string(); p = r.f64(3); q = r.f64(4)
            yaw = math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))
            od.append((T, *p, yaw))
        elif t == '/fmu/out/vehicle_local_position':
            d = b[4:]; x, y, z = struct.unpack_from('<3f', d, 20); lp.append((T, y, x, -z))
        elif t == '/fmu/out/vehicle_attitude':
            d = b[4:]; w, x, y, z = struct.unpack_from('<4f', d, 16)
            roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)); pitch = math.asin(max(-1, min(1, 2 * (w * y - z * x))))
            att.append((T, math.degrees(roll), math.degrees(pitch)))
        elif t == '/skybeta/state':
            j = json.loads(cdr.R(b).string()); sky.append((T, bool(j.get('armed')), bool(j.get('offboard_active'))))
    return {k: np.array(v) for k, v in dict(od=od, lp=lp, att=att, sky=sky).items()}

if __name__ == '__main__':
    data = {k: load(k) for k in BAGS}
    import pickle; pickle.dump(data, open(os.path.join(os.path.dirname(__file__), 'tracks.pkl'), 'wb'))
    fig, ax = plt.subplots(1, 1, figsize=(13, 11))
    r = np.array(ROUTE)
    ax.plot(r[:, 0], r[:, 1], 'k--o', ms=4, lw=1, label='v1 route (fwd, left)')
    for i, (x, y) in enumerate(ROUTE): ax.annotate(str(i), (x, y), fontsize=9, xytext=(4, 4), textcoords='offset points')
    col = dict(run1='tab:red', run2='tab:orange', walk_fwd='tab:blue', walk_rev='tab:green')
    for k, d in data.items():
        o = d['od']; ax.plot(o[:, 1], o[:, 2], color=col[k], lw=1.2, label=f'{k} ({BAGS[k]})')
        ax.plot(o[0, 1], o[0, 2], 'o', color=col[k]); ax.plot(o[-1, 1], o[-1, 2], 'x', color=col[k], ms=10, mew=2)
        print(k, 'start yaw %.1f deg' % math.degrees(o[0, 4]), 'end', np.round(o[-1, 1:4], 2))
    ax.set_aspect('equal'); ax.grid(alpha=.3); ax.legend(); ax.set_xlabel('x = forward [m]'); ax.set_ylabel('y = left [m]')
    ax.set_title('2026-11-08 morning: VIO tracks (odom frame) vs v1 route. o = start, x = end')
    plt.tight_layout(); plt.savefig(os.path.join(os.path.dirname(__file__), 'tracks_top.png'), dpi=80)
