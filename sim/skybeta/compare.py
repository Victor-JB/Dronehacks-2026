#!/usr/bin/env python3
"""Compare a drone bag with a sim bag (or any two fly.sh / fly-sim-drone.sh bags).

    python3 -m pip install rosbags numpy matplotlib        # once, anywhere (no ROS needed)
    python3 sim/skybeta/compare.py DRONE_BAG SIM_BAG [-o OUT_DIR]

Both bags come from the same topic list (jetson scripts/fly.sh, fly-sim-drone.sh). Each run is
aligned at the moment PX4 entered Offboard (t = 0), and the report puts them side by side:

  tracking   |setpoint - odometry| over time: how closely the vehicle follows the mission
  motion     speed and tilt (how hard it flies), altitude: VIO vs PX4 vs the rangefinder
  pipeline   rate of every recorded topic, and odometry/depth latency (bag receive time - stamp)
  mission    refnav phases and how long each took, how the run ended
  estimator  share of time EKF2 fused vision position/yaw and rangefinder height

Writes OUT_DIR/report.md and OUT_DIR/*.png (default: next to the second bag).
"""
import argparse
import json
import math
from collections import defaultdict
from pathlib import Path

import numpy as np

OFFBOARD = 14


def read_bag(path):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    series = defaultdict(list)          # topic -> [(t_receive_s, msg)]
    with AnyReader([Path(path)], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        for conn, t, raw in reader.messages():
            if conn.topic in ('/oak/stereo/image_raw',):
                # keep the header only: images are big and only their timing is compared
                msg = reader.deserialize(raw, conn.msgtype)
                series[conn.topic].append((t * 1e-9, stamp(msg.header.stamp)))
                continue
            try:
                series[conn.topic].append((t * 1e-9, reader.deserialize(raw, conn.msgtype)))
            except Exception:
                pass
    return series


def stamp(s):
    return s.sec + s.nanosec * 1e-9


def offboard_start(series):
    for t, m in series.get('/fmu/out/vehicle_status_v1', []):
        if m.nav_state == OFFBOARD:
            return t
    for t, m in series.get('/skybeta/state', []):
        if json.loads(m.data).get('offboard_active'):
            return t
    return None


def xyz(points):
    return np.array([[p.x, p.y, p.z] for p in points])


def run_summary(name, s):
    t0 = offboard_start(s)
    if t0 is None:
        t0 = min(v[0][0] for v in s.values() if v)
        note = 'never entered Offboard: t = 0 is the start of the bag'
    else:
        note = ''
    out = {'name': name, 't0': t0, 'note': note, 'series': {}, 'numbers': {}}

    odom = s.get('/visual_slam/tracking/odometry', [])
    if odom:
        t = np.array([a for a, _ in odom]) - t0
        p = xyz([m.pose.pose.position for _, m in odom])
        out['series']['odom'] = (t, p)
        lat = np.array([a - stamp(m.header.stamp) for a, m in odom])
        out['numbers']['odometry latency p50 / p95 ms'] = f'{1e3*np.percentile(lat, 50):.0f} / {1e3*np.percentile(lat, 95):.0f}'
    sp = s.get('/skybeta/setpoint/pose', [])
    if sp and odom:
        ts = np.array([a for a, _ in sp]) - t0
        ps = xyz([m.pose.position for _, m in sp])
        to, po = out['series']['odom']
        idx = np.clip(np.searchsorted(to, ts), 0, len(to) - 1)
        err = np.linalg.norm(ps - po[idx], axis=1)
        out['series']['setpoint'] = (ts, ps)
        out['series']['tracking_error'] = (ts, err)
        flying = ts > 0
        if flying.any():
            out['numbers']['tracking error median / p95 m'] = f'{np.median(err[flying]):.2f} / {np.percentile(err[flying], 95):.2f}'
    lp = s.get('/fmu/out/vehicle_local_position', [])
    if lp:
        t = np.array([a for a, _ in lp]) - t0
        v = np.array([math.sqrt(m.vx**2 + m.vy**2 + m.vz**2) for _, m in lp])
        z = np.array([-m.z for _, m in lp])
        dist = np.array([m.dist_bottom if m.dist_bottom_valid else np.nan for _, m in lp])
        out['series']['speed'] = (t, v)
        out['series']['px4_height'] = (t, z)
        out['series']['rangefinder'] = (t, dist)
        f = t > 0
        if f.any():
            out['numbers']['speed max / mean in Offboard m/s'] = f'{v[f].max():.2f} / {v[f].mean():.2f}'
    att = s.get('/fmu/out/vehicle_attitude', [])
    if att:
        t = np.array([a for a, _ in att]) - t0
        tilt = []
        for _, m in att:
            w, x, y, z = m.q
            tilt.append(math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y))))))
        tilt = np.array(tilt)
        out['series']['tilt'] = (t, tilt)
        f = t > 0
        if f.any():
            out['numbers']['tilt max / p95 deg'] = f'{tilt[f].max():.1f} / {np.percentile(tilt[f], 95):.1f}'
    depth = s.get('/oak/stereo/image_raw', [])
    if depth:
        lat = np.array([a - st for a, st in depth])
        out['numbers']['depth latency p50 / p95 ms'] = f'{1e3*np.percentile(lat, 50):.0f} / {1e3*np.percentile(lat, 95):.0f}'
    flags = s.get('/fmu/out/estimator_status_flags', [])
    if flags:
        for field in ('cs_ev_pos', 'cs_ev_yaw', 'cs_rng_hgt', 'cs_baro_hgt'):
            vals = [bool(getattr(m, field)) for a, m in flags if a - t0 > 0]
            if vals:
                out['numbers'][f'EKF2 {field} (share of Offboard)'] = f'{100*np.mean(vals):.0f} %'
    phases, last = [], None
    for t, m in s.get('/refnav/status', []):
        st = json.loads(m.data)
        if st.get('phase') != last:
            phases.append((t - t0, st.get('phase'), st.get('why')))
            last = st.get('phase')
        out['result'] = st.get('result')
    out['phases'] = phases
    out['rates'] = {topic: (len(v) / (v[-1][0] - v[0][0]) if len(v) > 1 and v[-1][0] > v[0][0] else 0.0)
                    for topic, v in s.items()}
    return out


def plot(runs, out_dir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    panels = [('tracking_error', '|setpoint - odometry| (m)'), ('speed', 'speed (m/s)'),
              ('tilt', 'tilt (deg)'), ('px4_height', 'PX4 height (m)'), ('rangefinder', 'rangefinder (m)')]
    fig, axes = plt.subplots(len(panels), 1, figsize=(10, 2.4 * len(panels)), sharex=True)
    for ax, (key, label) in zip(axes, panels):
        for r in runs:
            if key in r['series']:
                t, y = r['series'][key]
                ax.plot(t, y, label=r['name'], lw=1)
        ax.set_ylabel(label)
        ax.axvline(0, color='k', lw=0.5)
        ax.grid(alpha=0.3)
    axes[0].legend()
    axes[-1].set_xlabel('time from Offboard (s)')
    fig.tight_layout()
    fig.savefig(out_dir / 'timeseries.png', dpi=120)

    fig, ax = plt.subplots(figsize=(6, 6))
    for r in runs:
        if 'odom' in r['series']:
            ax.plot(*r['series']['odom'][1][:, :2].T, label=f"{r['name']} odometry", lw=1)
        if 'setpoint' in r['series']:
            ax.plot(*r['series']['setpoint'][1][:, :2].T, '--', label=f"{r['name']} setpoint", lw=0.8)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m, odom)')
    ax.set_ylabel('y (m, odom)')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out_dir / 'path.png', dpi=120)


def report(runs, out_dir):
    names = [r['name'] for r in runs]
    lines = ['# Run comparison', '', '| | ' + ' | '.join(names) + ' |', '|---' * (len(runs) + 1) + '|']
    keys = []
    for r in runs:
        keys += [k for k in r['numbers'] if k not in keys]
    for k in keys:
        lines.append(f'| {k} | ' + ' | '.join(r['numbers'].get(k, '-') for r in runs) + ' |')
    lines.append('| result | ' + ' | '.join(str(r.get('result', '-')) for r in runs) + ' |')
    for r in runs:
        if r['note']:
            lines += ['', f"**{r['name']}**: {r['note']}"]
    lines += ['', '![time series](timeseries.png)', '', '![path](path.png)', '', '## Mission phases', '']
    for r in runs:
        lines += [f"**{r['name']}**", '', '| t from Offboard (s) | phase | why |', '|---|---|---|']
        lines += [f'| {t:.1f} | {p} | {w or ""} |' for t, p, w in r['phases']]
        lines.append('')
    lines += ['## Topic rates (Hz)', '', '| topic | ' + ' | '.join(names) + ' |', '|---' * (len(runs) + 1) + '|']
    topics = sorted(set().union(*[r['rates'] for r in runs]))
    for tp in topics:
        lines.append(f'| {tp} | ' + ' | '.join(f"{r['rates'].get(tp, 0):.1f}" for r in runs) + ' |')
    (out_dir / 'report.md').write_text('\n'.join(lines) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('bags', nargs=2, type=Path, help='bag folders (mcap), e.g. drone first, sim second')
    parser.add_argument('--names', nargs=2, default=None, help='labels (default: drone sim)')
    parser.add_argument('-o', '--out', type=Path, default=None)
    args = parser.parse_args()
    names = args.names or ['drone', 'sim']
    out_dir = args.out or args.bags[1] / 'compare'
    out_dir.mkdir(parents=True, exist_ok=True)
    runs = [run_summary(n, read_bag(b)) for n, b in zip(names, args.bags)]
    plot(runs, out_dir)
    report(runs, out_dir)
    print(f'wrote {out_dir / "report.md"}')


if __name__ == '__main__':
    main()
