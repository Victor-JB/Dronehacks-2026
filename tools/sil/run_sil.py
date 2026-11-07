"""Software-in-the-loop run of the REAL dh_autonomy mission (and localizer) nodes, without ROS.

The simulated drone:
  * odometry is published in an `odom` frame that is rotated/offset from the
    course frame (like cuVSLAM's startup-anchored frame), with noise, random-walk
    drift, and a velocity error while inside the dark tube;
  * it obeys /skybeta/setpoint/twist through a PX4-like velocity loop
    (1.0 m/s horizontal cap, first-order lag; by default also a horizontal acceleration
    limit (MPC_ACC_HOR-like) and attitude lag, so curvature jumps and steps cost something);
  * optional synthetic forward scans (80 deg, blind in the tube) feed the localizer;
  * a ray-cast /dh/forward_clearance (+-15 deg, blind in the tube) drives the mission's
    stereo slowdown exactly as stereo_scan_node would;
  * altitude drifts too (random walk + tube error), and wall contact is checked in 3-D against
    course slices at the drone's TRUE height (window sill/lintel, tube oval).
  --legacy reproduces the old idealised simulator (no accel limit, no vertical drift,
  no forward clearance, 2-D contact check) for comparison with earlier results.

Usage: python run_sil.py [--route window|gate] [--localizer] [--seed N] [--plot out.png]
"""
import argparse, json, math, os, sys, types
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "dh_core"))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "dh_autonomy"))
import fake_ros  # noqa: E402
fake_ros.install()
from fake_ros import BUS, Node, Odometry, String, Empty, LaserScan, Float32  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--route", default="window")
ap.add_argument("--localizer", action="store_true")
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--plot", default="")
ap.add_argument("--drift", type=float, default=0.01)
ap.add_argument("--dark_err", type=float, default=0.08)
ap.add_argument("--quiet", action="store_true")
ap.add_argument("--v_dark", type=float, default=0.6)
ap.add_argument("--backend", default="skybeta", choices=["skybeta", "px4"])
ap.add_argument("--with_return", action="store_true")
ap.add_argument("--planner", default="fillet", choices=["fillet", "sfc", "mpl"])
ap.add_argument("--v_max", type=float, default=0.85, help="planned cruise speed (m/s)")
ap.add_argument("--v_cap", type=float, default=1.0, help="mission follower command cap (m/s)")
ap.add_argument("--px4_cap", type=float, default=1.0, help="simulated PX4 MPC_XY_VEL_MAX (m/s)")
ap.add_argument("--acc_max", type=float, default=3.0, help="horizontal acceleration limit (m/s^2), PX4 MPC_ACC_HOR")
ap.add_argument("--att_tau", type=float, default=0.12, help="attitude/thrust response lag (s)")
ap.add_argument("--z_drift", type=float, default=0.005, help="altitude random walk (m/sqrt(m))")
ap.add_argument("--legacy", action="store_true", help="old idealised simulator")
ap.add_argument("--corner", default="clothoid", choices=["clothoid", "arc"])
ap.add_argument("--a_total", type=float, default=1.5, help="combined accel limit (0 = separate along/lateral)")
ap.add_argument("--window_straight", type=float, default=0.6)
args = ap.parse_args()

COURSE = os.environ.get("DH_COURSE_OBJ", "C:/dhdl/obj/dronehacks-course-v3.obj")
Node.PARAM_OVERRIDES = {"course_obj": COURSE, "backend": args.backend, "route": args.route, "planner": args.planner,
                        "autostart_on_offboard": False, "align_mode": "start", "v_dark": args.v_dark,
                        "enable_return": args.with_return, "sensor_x": 0.10,
                        "v_max": args.v_max, "v_cap": args.v_cap,
                        "corner": args.corner, "a_total": args.a_total, "window_straight": args.window_straight}   # SIL scans come from a sensor 0.10 m ahead

import dh_autonomy.mission_node as mn  # noqa: E402
mn.time = types.SimpleNamespace(time=lambda: BUS.t)
from dh_core.route import dark_zones  # noqa: E402
from dh_core.localize import raycast_scan  # noqa: E402

rng = np.random.default_rng(args.seed)
import time as _walltime  # noqa: E402
_t_plan = _walltime.perf_counter()
node = mn.MissionNode()
plan_compute_s = _walltime.perf_counter() - _t_plan
loc = None
if args.localizer:
    import dh_autonomy.localizer_node as ln
    ln.time = types.SimpleNamespace(time=lambda: BUS.t)
    loc = ln.LocalizerNode()

course = node.course
grid = node.plan.grid
dist = grid.distance()
zones = [b for b in dark_zones(course, ("tube", "tunnel"))]
scan_grid = course.occupancy(0.025, 0.58, 1.12)
# 3-D contact check: occupancy slices (drone half-height 0.12 m) at 10 cm height steps
SLICE_Z = np.arange(0.30, 1.65, 0.10)
slices = [] if args.legacy else [(g_, g_.distance()) for g_ in (course.occupancy(0.05, z - 0.12, z + 0.12) for z in SLICE_Z)]

# --- simulated vehicle (course frame) and odom frame offset
th0, off = 0.7, np.array([1.2, -0.4, 0.0])            # odom = Rz(-th0) (map - off)
c0, s0 = math.cos(th0), math.sin(th0)
def map_to_odom(p):
    d = p - off
    return np.array([c0 * d[0] + s0 * d[1], -s0 * d[0] + c0 * d[1], d[2]])
def odom_vec_to_map(v):
    return np.array([c0 * v[0] - s0 * v[1], s0 * v[0] + c0 * v[1], v[2]])

p = np.array([course.launch[0], course.launch[1], 0.0])
yaw = math.pi / 2      # placed on the launch mark facing north
v = np.zeros(3)
acc = np.zeros(3)
v_cmd = np.zeros(3); yr_cmd = 0.0
bias = np.zeros(3); dark_v = None
cmd = {"v": np.zeros(3), "yr": 0.0}

def on_twist(m):
    cmd["v"] = odom_vec_to_map(np.array([m.twist.linear.x, m.twist.linear.y, m.twist.linear.z]))
    cmd["yr"] = m.twist.angular.z
BUS.subs.setdefault("/skybeta/setpoint/twist", []).append(on_twist)

# --- fake PX4 (only used with --backend px4): offboard / arm / NAV_LAND / auto-disarm
px4 = {"nav": "manual", "armed": False, "landed": True, "land_t": None, "ocm_t": -9.0, "v": np.zeros(3)}
def on_ocm(m):
    px4["ocm_t"] = BUS.t
def on_ts(m):
    vn = list(m.velocity)
    if len(vn) == 3 and all(np.isfinite(vn)):
        px4["v"] = odom_vec_to_map(np.array([vn[1], vn[0], -vn[2]]))   # NED -> odom ENU -> map
def on_cmd(m):
    if m.command == 176 and m.param2 == 6.0 and BUS.t - px4["ocm_t"] < 0.5:
        px4["nav"] = "offboard"
    elif m.command == 400:
        if m.param1 == 1.0 and px4["nav"] != "land":
            px4["armed"] = True
        elif m.param1 == 0.0 and px4["armed"]:
            px4["armed"] = False
            events.setdefault("disarms", []).append(round(BUS.t, 2))
    elif m.command == 21:
        px4["nav"] = "land"
for tpc, cb in (("/fmu/in/offboard_control_mode", on_ocm), ("/fmu/in/trajectory_setpoint", on_ts),
                ("/fmu/in/vehicle_command", on_cmd)):
    BUS.subs.setdefault(tpc, []).append(cb)
events = {}

dt = 0.01
traj_true, min_clear, touches, contact = [], np.inf, 0, False
est_err = []
max_speed = 0.0
land_t = None
t_start = None
while BUS.t < 150:
    t = BUS.t
    # --- sensors
    if abs((t * 100) % 3.333) < 1.0:     # ~30 Hz odom
        in_dark = any(lo[0] <= p[0] <= hi[0] and lo[1] <= p[1] <= hi[1] for lo, hi in zones)
        if in_dark:
            if dark_v is None:
                dark_v = np.append(rng.normal(0, args.dark_err, 2), 0.0 if args.legacy else rng.normal(0, 0.5 * args.dark_err))
        else:
            dark_v = None
        o = Odometry()
        po = map_to_odom(p + bias + np.append(rng.normal(0, 0.01, 2), 0.0 if args.legacy else rng.normal(0, 0.005)))
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = po
        yo = yaw - th0
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(yo / 2), math.cos(yo / 2)
        BUS.publish("/visual_slam/tracking/odometry", o)
    if abs((t * 100) % 10) < 1.0:
        BUS.publish("/skybeta/state", String(data=json.dumps({"offboard_active": True, "armed": True})))
        if not args.legacy:
            in_dark_fc = any(lo[0] <= p[0] <= hi[0] and lo[1] <= p[1] <= hi[1] for lo, hi in zones)
            fc = math.inf
            if not in_dark_fc and p[2] > 0.3:          # stereo is blind in the dark tube
                pts_fc = raycast_scan(scan_grid, np.array([p[0], p[1], yaw]), 30.0, 9, r_min=0.05, r_max=4.0,
                                      noise=0.02, rng=rng)
                if len(pts_fc):
                    fc = float(np.min(np.hypot(pts_fc[:, 0], pts_fc[:, 1])))
            BUS.publish("/dh/forward_clearance", Float32(data=fc))
        if loc is not None:
            in_dark = any(lo[0] <= p[0] <= hi[0] and lo[1] <= p[1] <= hi[1] for lo, hi in zones)
            if not in_dark and p[2] > 0.4:
                # sensor sits 0.10 m ahead of the body origin (localizer param sensor_x)
                sp = np.array([p[0] + 0.10 * math.cos(yaw), p[1] + 0.10 * math.sin(yaw), yaw])
                sc = raycast_scan(scan_grid, sp, 80.0, 64, r_max=4.0, noise=0.02, rng=rng)
                if len(sc):
                    nb = 64
                    amin, amax = -math.radians(40), math.radians(40)
                    inc = (amax - amin) / (nb - 1)
                    ranges = np.full(nb, np.inf)
                    idx = np.round((np.arctan2(sc[:, 1], sc[:, 0]) - amin) / inc).astype(int)
                    for i, r in zip(idx, np.hypot(sc[:, 0], sc[:, 1])):
                        if 0 <= i < nb:
                            ranges[i] = min(ranges[i], r)
                    ls = LaserScan()
                    ls.angle_min, ls.angle_increment = amin, inc
                    ls.ranges = list(ranges); ls.range_min, ls.range_max = 0.2, 4.0
                    BUS.publish("/dh/stereo_scan", ls)
    if t_start is None and t >= 1.0:
        BUS.publish("/dh/start", Empty()); t_start = t
    # --- node timers
    for tm in BUS.timers:
        tm[1] += dt
        if tm[1] >= tm[0] - 1e-9:
            tm[1] = 0.0
            tm[2]()
    # --- vehicle dynamics (PX4 velocity loop)
    if args.backend == "px4":
        if abs((t * 100) % 10) < 1.0:
            st = sys.modules["px4_msgs.msg"].VehicleStatus()
            st.nav_state = 14 if px4["nav"] == "offboard" else (18 if px4["nav"] == "land" else 0)
            st.arming_state = 2 if px4["armed"] else 1
            BUS.publish("/fmu/out/vehicle_status_v1", st)
            BUS.publish("/fmu/out/vehicle_land_detected", sys.modules["px4_msgs.msg"].VehicleLandDetected(landed=px4["landed"]))
        if not px4["armed"]:
            vc = np.zeros(3)
            if p[2] <= 0.01:
                v[:] = 0.0
        elif px4["nav"] == "land":
            vc = np.array([0.0, 0.0, -0.7 if p[2] > 1.0 else -0.3])
        elif px4["nav"] == "offboard":
            vc = px4["v"].copy()
        else:
            vc = np.zeros(3)
        if px4["armed"] and p[2] <= 0.01 and vc[2] <= 0:
            if not px4["landed"]:
                px4["landed"] = True; px4["land_t"] = t
            if px4["land_t"] is not None and t - px4["land_t"] > 2.0:     # COM_DISARM_LAND
                px4["armed"] = False; px4["land_t"] = None
                events.setdefault("disarms", []).append(round(t, 2))
        elif p[2] > 0.05:
            px4["landed"] = False; px4["land_t"] = None
    else:
        vc = cmd["v"].copy()
    h = math.hypot(vc[0], vc[1])
    if h > args.px4_cap:
        vc[:2] *= args.px4_cap / h
    if args.legacy:
        v += (vc - v) / 0.3 * dt
    else:
        a_cmd = (vc - v) / 0.3
        ah = math.hypot(a_cmd[0], a_cmd[1])
        if ah > args.acc_max:
            a_cmd[:2] *= args.acc_max / ah
        a_cmd[2] = float(np.clip(a_cmd[2], -3.0, 3.0))
        acc += (a_cmd - acc) * min(1.0, dt / args.att_tau)
        v += acc * dt
    p += v * dt
    if p[2] <= 0.0:
        p[2] = 0.0; v[2] = max(v[2], 0.0)
    yaw += float(np.clip(cmd["yr"], -1.5, 1.5)) * dt
    step = np.linalg.norm(v[:2] * dt)
    bias[:2] += rng.normal(0, args.drift * math.sqrt(step + 1e-12), 2)
    if not args.legacy:
        bias[2] += rng.normal(0, args.z_drift * math.sqrt(step + 1e-12))
    if dark_v is not None:
        bias += dark_v * dt
    traj_true.append(p.copy())
    max_speed = max(max_speed, float(math.hypot(v[0], v[1])))
    if node.align.valid and node.odom is not None and int(round(t / dt)) % 10 == 0:
        pm_est, _ = node._map_pose()
        est_err.append((t, float(np.linalg.norm(pm_est[:2] - p[:2])), len(BUS.published.get("/dh/map_correction", []))))
    if 0.3 < p[2] < course.wall_height + 0.15:
        if slices:
            g_, d_ = slices[int(np.clip(round((p[2] - SLICE_Z[0]) / 0.10), 0, len(slices) - 1))]
            r_, c_ = g_.to_cell(p)
            clr = d_[r_, c_] - 0.24
        else:
            r_, c_ = grid.to_cell(p)
            clr = dist[r_, c_] - 0.24
        min_clear = min(min_clear, clr)
        if clr < 0 and not contact:
            touches += 1; contact = True; events.setdefault("contacts", []).append([round(t, 2), round(float(p[0]), 2), round(float(p[1]), 2)])
        elif clr >= 0:
            contact = False
    if args.backend == "px4":
        if "finish" not in events and events.get("disarms"):
            events["finish"] = events["disarms"][0]
            events["finish_pad_err"] = round(float(np.linalg.norm(p[:2] - course.pad_center[:2])), 3)
        if node.state == "DONE" or t > 180:
            if len(events.get("disarms", [])) >= (2 if args.with_return else 1):
                events["return_launch_err"] = round(float(np.linalg.norm(p[:2] - course.launch[:2])), 3)
            land_t = land_t or t
            break
        if node.state in ("LAND",) and land_t is None:
            land_t = t
    else:
        if node.state in ("LAND", "RETURN_LAND", "DONE") and land_t is None:
            land_t = t
        if land_t is not None and t - land_t > 4.0:
            break
    BUS.t += dt

pad_err = float(np.linalg.norm(p[:2] - course.pad_center[:2]))
if est_err:
    ee = np.array(est_err)
    k = int(np.argmax(ee[:, 1]))
    print("estimate error: mean %.3f max %.3f at t=%.1f (corrections so far %d of %d)"
          % (ee[:, 1].mean(), ee[k, 1], ee[k, 0], int(ee[k, 2]), int(ee[-1, 2])))
result = {"route": args.route, "localizer": args.localizer, "seed": args.seed,
          "course_time_s": None if land_t is None else round(land_t - t_start, 2),
          "touches": touches, "min_clearance_m": round(float(min_clear), 3), "pad_xy_error_m": round(pad_err, 3),
          "final_state": node.state, "planner": node.plan.info.get("planner"),
          "planned_time_s": round(node.plan.legs[0].traj.duration, 2), "planned_length_m": round(node.plan.legs[0].traj.length, 2),
          "plan_compute_s": round(plan_compute_s, 1), "max_speed_mps": round(max_speed, 2),
          "est_err_mean_m": round(float(np.mean([e for _, e, _ in est_err])), 3) if est_err else None,
          "est_err_max_m": round(float(np.max([e for _, e, _ in est_err])), 3) if est_err else None,
          "route_used": node.plan.route_used}
if args.backend == "px4":
    result.update({k: v_ for k, v_ in events.items() if k not in ("disarms",)})
    result["disarm_times"] = events.get("disarms", [])
    if "finish" in events:
        result["scored_finish_s"] = round(events["finish"] - 1.0, 2)   # clock starts at liftoff ~ start cmd
result["contacts"] = events.get("contacts", [])
print("RESULT", json.dumps(result))

if args.plot:
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    tt = np.array(traj_true)
    fig, ax = plt.subplots(figsize=(7, 7))
    ext = [grid.origin[0], grid.origin[0] + grid.shape[1] * grid.res, grid.origin[1], grid.origin[1] + grid.shape[0] * grid.res]
    ax.imshow(grid.occ, origin="lower", extent=ext, cmap="Greys")
    pl = node.plan.legs[0].traj.p
    ax.plot(pl[:, 0], pl[:, 1], "b--", lw=1, label="plan")
    ax.plot(tt[:, 0], tt[:, 1], "r-", lw=1, label="flown (true)")
    ax.add_patch(plt.Circle(course.pad_center[:2], course.pad_radius, fill=False))
    ax.set_aspect("equal"); ax.legend(); ax.set_title("SIL: %s, localizer=%s, touches=%d" % (args.route, args.localizer, touches))
    fig.savefig(args.plot, dpi=110, bbox_inches="tight")

if os.environ.get("DH_DEBUG_CORR"):
    lo, hi = map(float, os.environ["DH_DEBUG_CORR"].split(","))
    ee = {round(t, 1): e for t, e, _ in est_err}
    for t, m in BUS.published.get("/dh/map_correction", []):
        if lo <= t <= hi:
            print("t=%.2f corr=(%+.3f,%+.3f) |%.3f| est_err=%.3f" % (t, m.vector.x, m.vector.y, math.hypot(m.vector.x, m.vector.y), ee.get(round(t, 1), float('nan'))))
    for t, m in BUS.published.get("/dh/loc_health", []):
        if lo <= t <= hi and m.data[0] < 0.5:
            print("t=%.2f REJECTED inl=%.2f rms=%.3f eig=%.3f" % (t, m.data[1], m.data[2], m.data[3]))
    for t, m in BUS.published.get("/dh/stereo_scan", []):
        if lo <= t <= hi:
            print("t=%.2f scan finite=%d" % (t, int(np.isfinite(np.array(m.ranges)).sum())))
    st = [(t, m.data) for t, m in BUS.published.get("/dh/mission_state", [])]
    seen = None
    for t, s in st:
        if s != seen:
            print("t=%.2f state %s" % (t, s)); seen = s
