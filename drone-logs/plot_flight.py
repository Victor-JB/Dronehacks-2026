"""Plot a flight bag: estimated position vs time and setpoint error vs time.
python3 plot_flight.py <bag.mcap> <out.png>"""
import sys, struct, json, math, numpy as np, os
sys.path.insert(0, os.path.dirname(__file__))
import mcapr, cdr
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
P, OUT = sys.argv[1], sys.argv[2]
od, sp, lp, sky = [], [], [], []
t0 = None
for t, s, lt, b in mcapr.read(P, {'/visual_slam/tracking/odometry', '/skybeta/setpoint/pose',
                                 '/fmu/out/vehicle_local_position', '/skybeta/state'}):
    if t0 is None: t0 = lt
    T = (lt - t0) / 1e9; r = cdr.R(b)
    if t == '/visual_slam/tracking/odometry':
        r.header(); r.string(); p = r.f64(3); od.append((T, *p))
    elif t == '/skybeta/setpoint/pose':
        r.header(); p = r.f64(3); sp.append((T, *p))
    elif t == '/fmu/out/vehicle_local_position':
        d = b[4:]; x, y, z = struct.unpack_from('<3f', d, 20); lp.append((T, y, x, -z))   # NED -> ENU
    else:
        j = json.loads(r.string()); sky.append((T, bool(j.get('armed')), bool(j.get('offboard_active'))))
od, sp, lp, sky = (np.array(a) for a in (od, sp, lp, sky))
print(f'odom {len(od)}  setpoints {len(sp)}  px4 {len(lp)}  duration {od[-1,0]:.1f} s')
# error: setpoint (held between messages) minus VIO position, while Offboard
idx = np.clip(np.searchsorted(sp[:, 0], od[:, 0], side='right') - 1, 0, len(sp) - 1)
err = sp[idx, 1:4] - od[:, 1:4]
offb = (np.interp(od[:, 0], sky[:, 0], sky[:, 2].astype(float)) > 0.5) & (np.interp(od[:, 0], sky[:, 0], sky[:, 1].astype(float)) > 0.5)
fig, ax = plt.subplots(4, 1, figsize=(12, 13), sharex=True)
for k, (name, c) in enumerate((('x (forward)', 'tab:blue'), ('y (left)', 'tab:orange'), ('z (up)', 'tab:green'))):
    ax[k].plot(od[:, 0], od[:, k + 1], color=c, label='VIO estimate')
    if len(lp): ax[k].plot(lp[:, 0], lp[:, k + 1], color=c, ls=':', label='PX4 estimate (EKF2)')
    ax[k].step(sp[:, 0], sp[:, k + 1], where='post', color='k', lw=0.8, alpha=0.6, label='setpoint')
    ax[k].set_ylabel(name + ' [m]'); ax[k].legend(loc='upper left', fontsize=8); ax[k].grid(alpha=0.3)
e = np.where(offb[:, None], err, np.nan)
ax[3].plot(od[:, 0], e[:, 0], label='x error'); ax[3].plot(od[:, 0], e[:, 1], label='y error')
ax[3].plot(od[:, 0], e[:, 2], label='z error'); ax[3].plot(od[:, 0], np.linalg.norm(e, axis=1), 'k', label='|error|')
ax[3].set_ylabel('setpoint - VIO [m]\n(armed + Offboard)'); ax[3].set_xlabel('time [s]'); ax[3].legend(loc='upper left', fontsize=8); ax[3].grid(alpha=0.3)
if offb.any():                                   # zoom on the flight (a VIO blow-up on the ground would flatten it)
    fl = od[offb]
    for k in range(3):
        lo, hi = np.nanpercentile(fl[:, k + 1], 2), np.nanpercentile(fl[:, k + 1], 90); m = 0.3 + 0.1 * (hi - lo)
        ax[k].set_ylim(lo - m, hi + m)
    ax[3].set_xlim(max(0, fl[0, 0] - 5), fl[-1, 0] + 5)
    ax[3].set_ylim(-1.5, 1.5)
for a in ax:
    for T, arm, ob in sky[np.r_[True, np.any(np.diff(sky[:, 1:], axis=0) != 0, axis=1)]]:
        a.axvline(T, color='r' if ob else ('m' if arm else 'gray'), lw=0.6, alpha=0.5)
fig.suptitle(os.path.basename(P) + '   (red line = Offboard on, magenta = armed, gray = disarmed / off)')
plt.tight_layout(); plt.savefig(OUT, dpi=70)
n = np.linalg.norm(err[offb], axis=1) if offb.any() else np.array([np.nan])
print(f'Offboard {offb.sum()/len(offb)*100:.0f}% of samples; |error| median {np.nanmedian(n):.2f} m, max {np.nanmax(n):.2f} m')
