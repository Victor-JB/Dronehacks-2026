"""Closed loop through the REAL perception code: rendered stereo images -> stereo_scan_node (OpenCV SGBM,
rectification, pseudo-scan, forward clearance) + line_node -> localizer_node -> mission_node -> vehicle.

Unlike run_sil.py (which ray-casts scans straight from the map and skips OpenCV), here the localizer and
the forward-clearance governor only see what SGBM extracts from images. Mocked at the hardware boundary
only: the cameras (images rendered from the course mesh), cuVSLAM odometry (truth + drift + tube error),
the Sky Beta state topic and the vehicle (PX4-like velocity loop with acceleration limit).

SYNTHETIC IMAGES ARE NOT CAMERA VALIDATION: the walls carry a procedural solid texture and lighting is
uniform. Real wall finish, lighting, exposure, motion blur and calibration error are untested here.

  python tools/sil/run_perception_loop.py [--seed 0] [--route window] [--fps 5] [--t_max 40] [--plot out.png]
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "dh_core"))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "dh_autonomy"))
import cv2  # noqa: E402
import fake_ros  # noqa: E402

fake_ros.install()
from fake_ros import BUS, CameraInfo, Empty, Image, Node, Odometry, String, make_time  # noqa: E402
from dh_core.camera_geom import optical_to_body  # noqa: E402
from dh_core.route import dark_zones  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--route", default="window")
ap.add_argument("--fps", type=float, default=5.0)
ap.add_argument("--t_max", type=float, default=40.0)
ap.add_argument("--drift", type=float, default=0.01)
ap.add_argument("--dark_err", type=float, default=0.08)
ap.add_argument("--plot", default="")
ap.add_argument("--out", default="")
ap.add_argument("--trace", default="", help="write truth/estimate/clearance diagnostic samples (simulation only)")
ap.add_argument("--blackout_tube", action="store_true", help="optional sensor-fault injection; real tube is illuminated")
ap.add_argument("--initial_height", type=float, default=0.20, help="synthetic body centre above floor while resting on landing gear (m); unmeasured hardware assumption")
args = ap.parse_args()
COURSE = os.environ.get("DH_COURSE_OBJ", "C:/dhdl/obj/dronehacks-course-v3.obj")

W, H, F = 512, 320, 280.0
CX, CY = W / 2.0, H / 2.0
BASE = 0.075
PITCH, CAM_X, CAM_Z = math.radians(20.0), 0.10, -0.02
K = np.array([[F, 0, CX], [0, F, CY], [0, 0, 1.0]])

common = {"course_obj": COURSE}
Node.PARAM_OVERRIDES = dict(common, ground_z=args.initial_height, route=args.route, require_vo_status=False, mount_source="parameters", dry_run=False, align_mode="start", v_dark=0.85,
                            scale=1.0, rectify_source="camera_info", camera_pitch_deg=20.0, camera_x=CAM_X,
                            camera_z=CAM_Z, num_disparities=64)
import dh_autonomy.mission_node as mn  # noqa: E402
import dh_autonomy.localizer_node as lnode  # noqa: E402
import dh_autonomy.stereo_scan_node as ssn  # noqa: E402
import dh_autonomy.line_node as linen  # noqa: E402

node = mn.MissionNode()
loc = lnode.LocalizerNode()
stereo = ssn.StereoScanNode()
sync = fake_ros.ApproximateTimeSynchronizer.last
line = linen.LineNode()
course = node.course

# ---- camera_info (rectified pair: R = I, P_right has the baseline)
def info(frame, right):
    ci = CameraInfo(); ci.header.frame_id = frame; ci.width, ci.height = W, H
    ci.k = list(K.ravel()); ci.d = [0.0] * 5; ci.r = list(np.eye(3).ravel())
    P = np.c_[K, np.zeros(3)]
    if right:
        P[0, 3] = -F * BASE
    ci.p = list(P.ravel())
    return ci
stereo._on_info("l", info("oak_left_camera_optical_frame", False))
stereo._on_info("r", info("oak_right_camera_optical_frame", True))
line._on_info(info("oak_left_camera_optical_frame", False))

# ---- scene: distance field of the walls (full height) + solid texture + guideline + dark zones
from scene import VoxelScene
scene = VoxelScene.from_course(course)
zones = dark_zones(course, ("tube", "tunnel"))
rng_tex = np.random.default_rng(7)
TEX = rng_tex.uniform(0, 1, (48, 48, 48)).astype(np.float32)
TEX = cv2.GaussianBlur(TEX.reshape(48, -1), (0, 0), 0.6).reshape(48, 48, 48)
gl = course.guideline


def solid_texture(P, cell=0.035):
    q = P / cell
    i0 = np.floor(q).astype(int); f = q - i0
    out = np.zeros(len(P), np.float32)
    for dx in (0, 1):
        for dy in (0, 1):
            for dz in (0, 1):
                w = (f[:, 0] if dx else 1 - f[:, 0]) * (f[:, 1] if dy else 1 - f[:, 1]) * (f[:, 2] if dz else 1 - f[:, 2])
                out += w * TEX[(i0[:, 0] + dx) % 48, (i0[:, 1] + dy) % 48, (i0[:, 2] + dz) % 48]
    return out


def dist_to_polyline(xy, pts):
    d = np.full(len(xy), np.inf)
    for a, b in zip(pts[:-1], pts[1:]):
        ab = b - a; L2 = float(ab @ ab)
        if L2 < 1e-12:
            continue
        t = np.clip(((xy - a) @ ab) / L2, 0, 1)
        d = np.minimum(d, np.linalg.norm(xy - (a + t[:, None] * ab), axis=1))
    return d


u, v = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
RAYS = np.stack([(u - CX) / F, (v - CY) / F, np.ones_like(u)], -1).reshape(-1, 3)
RAYS /= np.linalg.norm(RAYS, axis=1, keepdims=True)
DB = np.stack(optical_to_body(RAYS[:, 0], RAYS[:, 1], RAYS[:, 2], PITCH, 0.0, 0.0), -1)
RIGHT_OFF = np.array(optical_to_body(np.array([BASE]), np.array([0.0]), np.array([0.0]), PITCH, 0.0, 0.0)).ravel()


def render(p_body, yaw, right=False, rng=None):
    c, s = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    o = p_body + Rz @ np.array([CAM_X, 0.0, CAM_Z]) + (Rz @ RIGHT_OFF if right else 0.0)
    d = DB @ Rz.T
    Pw, hit, use_floor = scene.raycast(o, d)
    img = 0.25 + 0.6 * solid_texture(Pw)
    sky = ~hit
    img[sky] = 0.35
    if gl is not None:
        fl = np.nonzero(use_floor)[0]
        on_line = dist_to_polyline(Pw[fl, :2], gl) < 0.0254
        img[fl[on_line]] *= 0.35
    img = img.reshape(H, W)
    if args.blackout_tube and any(lo[0] <= p_body[0] <= hi[0] and lo[1] <= p_body[1] <= hi[1] for lo, hi in zones):
        img *= 0.03                                       # the dark tube
    img = cv2.GaussianBlur(img, (0, 0), 0.6)
    if rng is not None:
        img = img + rng.normal(0, 0.012, img.shape)
    return np.clip(img * 255, 0, 255).astype(np.uint8)


def image_msg(arr, frame, t):
    m = Image(); m.header.frame_id = frame; m.header.stamp = make_time(t)
    m.height, m.width, m.step, m.encoding = H, W, W * 3, "bgr8"     # the organizers' driver ships bgr8
    m.data = np.repeat(arr[:, :, None], 3, axis=2).tobytes()
    return m


# ---- vehicle + odometry (same models as run_sil.py)
rng = np.random.default_rng(args.seed)
th0, off = 0.7, np.array([1.2, -0.4, 0.0])
c0, s0 = math.cos(th0), math.sin(th0)
map_to_odom = lambda q: np.array([c0 * (q - off)[0] + s0 * (q - off)[1], -s0 * (q - off)[0] + c0 * (q - off)[1], (q - off)[2]])
odom_vec_to_map = lambda w: np.array([c0 * w[0] - s0 * w[1], s0 * w[0] + c0 * w[1], w[2]])
p = np.array([course.launch[0], course.launch[1], args.initial_height]); yaw = math.pi / 2
v = np.zeros(3); acc = np.zeros(3); bias = np.zeros(3); dark_v = None
cmd = {"v": np.zeros(3), "yr": 0.0}


def on_twist(m):
    cmd["v"] = odom_vec_to_map(np.array([m.twist.linear.x, m.twist.linear.y, m.twist.linear.z]))
    cmd["yr"] = m.twist.angular.z
BUS.subs.setdefault("/skybeta/setpoint/twist", []).append(on_twist)

dt = 0.01
period = 1.0 / args.fps
next_frame = 0.5
grid = node.plan.grid; dist = grid.distance()
min_clear, touches, contact, est, t_start, land_t = np.inf, 0, False, [], None, None
traj = []
trace = []
truth_t = []
render_s = 0.0
wall0 = time.time()
while BUS.t < args.t_max:
    t = BUS.t
    if abs((t * 100) % 3.333) < 1.0:
        in_dark = args.blackout_tube and any(lo[0] <= p[0] <= hi[0] and lo[1] <= p[1] <= hi[1] for lo, hi in zones)
        dark_v = (dark_v if dark_v is not None else np.append(rng.normal(0, args.dark_err, 2), rng.normal(0, 0.04))) if in_dark else None
        o = Odometry(); o.header.stamp = make_time(t); o.header.frame_id = "odom"
        po = map_to_odom(p + bias + np.append(rng.normal(0, 0.01, 2), rng.normal(0, 0.005)))
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = po
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin((yaw - th0) / 2), math.cos((yaw - th0) / 2)
        BUS.publish("/visual_slam/tracking/odometry", o)
    if abs((t * 100) % 10) < 1.0:
        BUS.publish("/skybeta/state", String(data=json.dumps({"offboard_active": True, "armed": True})))
    if t >= next_frame - 1e-9:
        next_frame += period
        r0 = time.time()
        L_img = render(p.copy(), yaw, False, rng); R_img = render(p.copy(), yaw, True, rng)
        render_s += time.time() - r0
        ml, mr = image_msg(L_img, "oak_left_camera_optical_frame", t), image_msg(R_img, "oak_right_camera_optical_frame", t)
        sync.cb(ml, mr)                                   # real stereo_scan_node (SGBM) -> scan + clearance
        line._on_img(ml)                                  # real line_node
    if t_start is None and t >= 1.0:
        BUS.publish("/dh/start", Empty()); t_start = t
    for tm in BUS.timers:
        tm[1] += dt
        if tm[1] >= tm[0] - 1e-9:
            tm[1] = 0.0; tm[2]()
    vc = cmd["v"].copy()
    h = math.hypot(vc[0], vc[1])
    if h > 1.0:
        vc[:2] *= 1.0 / h
    a_cmd = (vc - v) / 0.3
    ah = math.hypot(a_cmd[0], a_cmd[1])
    if ah > 3.0:
        a_cmd[:2] *= 3.0 / ah
    acc += (a_cmd - acc) * min(1.0, dt / 0.12)
    v += acc * dt; p += v * dt
    if p[2] < args.initial_height:
        p[2] = args.initial_height; v[2] = max(v[2], 0.0)
    yaw += float(np.clip(cmd["yr"], -1.5, 1.5)) * dt
    bias[:2] += rng.normal(0, args.drift * math.sqrt(np.linalg.norm(v[:2] * dt) + 1e-12), 2)
    if dark_v is not None:
        bias += dark_v * dt
    traj.append(p.copy())
    truth_t.append(t)
    if node.align.valid and int(round(t / dt)) % 10 == 0:
        pm, ym = node._map_pose(); est.append((t, float(np.linalg.norm(pm[:2] - p[:2]))))
        trace.append(dict(t=t, truth=p.tolist(), estimate=pm.tolist(), yaw=yaw, estimate_yaw=ym,
                          clearance=node.fc.range, expected=node._map_range(pm, ym),
                          tau=None if node.follower is None else node.follower.tau,
                          state=node.state, health=node.health.evaluate(t)[0]))
    if 0.3 < p[2] < course.wall_height:
        r_, c_ = grid.to_cell(p); clr = dist[r_, c_] - 0.24
        min_clear = min(min_clear, clr)
        if clr < 0 and not contact:
            touches += 1; contact = True
        elif clr >= 0:
            contact = False
    if node.state in ("LAND", "ABORTED", "DONE") and land_t is None:
        land_t = t
    if land_t is not None and t - land_t > 3.0:
        break
    BUS.t = round(BUS.t + dt, 6)

fc = [m.range for _, m in BUS.published.get("/dh/forward_clearance", [])]
# accuracy of every map fix the localizer published, against the TRUE pose at the scan's capture time
TT, TP = np.array(truth_t), np.array(traj)
fix_err = []
for _, m in BUS.published.get("/dh/map_fix", []):
    ts = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
    k = int(np.clip(np.searchsorted(TT, ts), 0, len(TT) - 1))
    fix_err.append(float(np.hypot(m.pose.pose.position.x - TP[k, 0], m.pose.pose.position.y - TP[k, 1])))
from collections import Counter
reasons = Counter(" ".join(m.data.split(" ")[:2]) + (" | " + " ".join(m.data.split(" ")[2:5]) if m.data.startswith("HOVER") else "")
                  for _, m in BUS.published.get("/dh/health", []))
res = {
    "initial_height_m": args.initial_height, "route": args.route, "route_used": node.plan.route_used, "planned_length_m": node.plan.legs[0].traj.length, "blackout_tube": args.blackout_tube, "seed": args.seed, "final_state": node.state, "abort_reason": node.abort_reason,
    "course_time_s": None if land_t is None or node.state == "ABORTED" else round(land_t - t_start, 2),
    "touches": touches, "min_clearance_m": round(float(min_clear), 3),
    "pad_xy_error_m": round(float(np.linalg.norm(p[:2] - course.pad_center[:2])), 3),
    "est_err_mean_m": round(float(np.mean([e for _, e in est])), 3) if est else None,
    "est_err_max_m": round(float(np.max([e for _, e in est])), 3) if est else None,
    "stereo_pairs": stereo.stats["pairs"], "stereo_dark": stereo.stats["dark"],
    "fc_clear": int(np.sum(np.isinf(fc))), "fc_obstacle": int(np.sum(np.isfinite(fc))), "fc_unknown": int(np.sum(np.isnan(fc))),
    "map_fixes_applied": node.counters["fix_applied"], "localizer_rejected": loc.rejected,
    "line_fixes_applied": node.counters["line_applied"], "hover_ticks": node.counters["hover_ticks"],
    "fix_err_median_m": round(float(np.median(fix_err)), 3) if fix_err else None,
    "fix_err_p90_m": round(float(np.percentile(fix_err, 90)), 3) if fix_err else None,
    "health": dict(reasons.most_common(6)),
    "render_s": round(render_s, 1), "wall_s": round(time.time() - wall0, 1),
}
print("RESULT", json.dumps(res))
if args.trace:
    with open(args.trace, "w") as f:
        json.dump(trace, f)
if args.out:
    with open(args.out, "a", encoding="utf-8") as f:
        f.write(json.dumps(res) + "\n")
if args.plot:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    tt = np.array(traj)
    fig, axs = plt.subplots(1, 2, figsize=(13, 6))
    ext = [grid.origin[0], grid.origin[0] + grid.shape[1] * grid.res, grid.origin[1], grid.origin[1] + grid.shape[0] * grid.res]
    axs[0].imshow(grid.occ, origin="lower", extent=ext, cmap="Greys")
    pl = node.plan.legs[0].traj.p
    axs[0].plot(pl[:, 0], pl[:, 1], "--", color="#2a78d6", lw=1, label="plan")
    axs[0].plot(tt[:, 0], tt[:, 1], "-", color="#eb6834", lw=1.2, label="flown (true)")
    axs[0].set_aspect("equal"); axs[0].legend(); axs[0].set_title("Closed loop through real OpenCV perception")
    axs[1].imshow(render(np.array([course.launch[0], course.launch[1] + 0.6, 0.85]), math.pi / 2), cmap="gray")
    axs[1].set_title("rendered left image (first corridor, guideline on the floor)"); axs[1].axis("off")
    fig.savefig(args.plot, dpi=100, bbox_inches="tight")
