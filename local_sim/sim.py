"""Local, pure-Python stand-in for the FlyX Lab sim: no ROS, no PX4, no Gazebo, no GPU.
Runs the REAL refnav.core.Mission decision logic (unmodified) against:
  - a kinematic fake PX4 (same model as tests/test_refnav_core.py's FakePX4)
  - a ray-cast depth camera seeing the REAL course geometry (local_sim/course.py)
and independently checks the flown trajectory against ground-truth obstacle distance.

What this validates: whether the mission's flight-decision code + a given route.json fly a
geometrically sound path through the actual course layout, and whether the depth-based safety
stop would catch a bad leg. What it does NOT validate: cuVSLAM/VIO accuracy, PX4 attitude/rate
control, real camera noise, real network/DDS timing, or anything that needs the real hardware or
the FlyX Lab sim. cuVSLAM in particular is NVIDIA-only (CUDA) and cannot run on this machine at all.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from refnav import core as C  # noqa: E402

from course import HALF, LAUNCH_XY, course_to_ne, nearest_obstacle_dist, outside_net, segments, true_clearance  # noqa: E402,F401
import dynamics as D  # noqa: E402

# A rough stand-in for the OAK-D S2's forward stereo pair: proportionally scaled down from the
# real 640x400 sensor used in refnav/node.py so the raycaster stays fast; same convention as
# tests/test_refnav_core.py's small test camera.
W, H, FX = 160, 100, 106.0
K = (FX, FX, W / 2, H / 2)
R_BODY_CAM = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])  # FLU <- optical, forward camera
T_BODY_CAM = np.array([0.16, 0.0, 0.0])
FLOOR_D = 0.18  # body height above the floor, as in the real harness
DRONE_RADIUS_M = 0.25  # placeholder prop-guard half-span -- replace with the real airframe once measured


GEAR_BELOW_M = 0.17     # body centre to landing-gear bottom, for "is it above that boulder"

# stereo depth model (OAK-D S2-like, scaled to this 160x100 test camera) -- assumptions
DEPTH_HZ = 15.0          # the FlyX qualified preset runs stereo/depth at 15 Hz
DEPTH_LATENCY_S = 0.08   # capture -> mission
MIN_DEPTH_M = 0.20       # OAK-D S2 with extended disparity; ~0.35 m without
MAX_DEPTH_M = 8.0
DEPTH_DROPOUT = 0.03     # fraction of pixels with no stereo match
NET_H = 3.048            # the 10 ft safety net around/over the course
NET_RETURN = 0.5         # chance stereo matches on the net's thin strands at a given pixel (assumption)


def q_yaw(yaw):
    return (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


def render_depth(pos_ned, att, segs, rng=None, net_return=NET_RETURN):
    """Depth image (m, NaN = no return) from the forward camera. att: NED <- FRD body rotation
    (3x3), or a yaw angle for a level camera. The camera tilts with the body."""
    if np.ndim(att) == 0:
        c, s = math.cos(att), math.sin(att)
        att = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    Rb = np.asarray(att, float)
    vs, us = np.mgrid[0:H, 0:W].astype(np.float64)
    d_opt = np.stack([(us - K[2]) / K[0], (vs - K[3]) / K[1], np.ones_like(us)], -1)
    d_frd = (d_opt @ R_BODY_CAM.T) * np.array([1.0, -1.0, -1.0])
    d_ned = d_frd @ Rb.T
    o = np.asarray(pos_ned, float) + Rb @ (T_BODY_CAM * np.array([1.0, -1.0, -1.0]))
    rn, re, rd = d_ned[..., 0], d_ned[..., 1], d_ned[..., 2]
    best = np.full((H, W), np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        tf = (FLOOR_D - o[2]) / rd
        best = np.where(tf > 0, np.minimum(best, tf), best)
        for n0, e0, n1, e1, height in segs:
            sn, se = n1 - n0, e1 - e0
            denom = sn * re - rn * se
            denom = np.where(np.abs(denom) < 1e-12, np.nan, denom)
            qn, qe = n0 - o[0], e0 - o[1]
            t = (sn * qe - qn * se) / denom
            svals = (rn * qe - qn * re) / denom
            p_d = o[2] + t * rd
            hit = (t > 0) & (svals >= 0) & (svals <= 1) & (p_d <= FLOOR_D) & (p_d >= FLOOR_D - height)
            best = np.where(hit, np.minimum(best, t), best)
        # the net: walls at course x/y = +-HALF (NED e/n) up to NET_H, and its roof; partly seen
        tnet = np.full((H, W), np.inf)
        top = FLOOR_D - NET_H
        for comp, val in ((1, -HALF), (1, HALF), (0, -HALF), (0, HALF)):
            r_ = d_ned[..., comp]
            tp = (val - o[comp]) / r_
            other = d_ned[..., 1 - comp] * tp + o[1 - comp]
            pd = o[2] + tp * rd
            ok = (tp > 0) & (np.abs(other) <= HALF) & (pd <= FLOOR_D) & (pd >= top)
            tnet = np.where(ok, np.minimum(tnet, tp), tnet)
        tp = (top - o[2]) / rd
        ok = (tp > 0) & (np.abs(o[0] + tp * rn) <= HALF) & (np.abs(o[1] + tp * re) <= HALF)
        tnet = np.where(ok, np.minimum(tnet, tp), tnet)
        seen = (tnet < best) & ((rng.random((H, W)) < net_return) if rng is not None else True)
        best = np.where(seen, tnet, best)
    best[~np.isfinite(best)] = np.nan
    return best.astype(np.float32)


def stereo_noise(img, rng):
    """Range-dependent stereo error (sigma_z ~ z^2), min/max range cut-off, random dropouts."""
    z = img.astype(np.float64)
    z = z + rng.normal(0.0, 1.0, z.shape) * (0.003 + 0.004 * z * z)
    z[(z < MIN_DEPTH_M) | (z > MAX_DEPTH_M)] = np.nan
    z[rng.random(z.shape) < DEPTH_DROPOUT] = np.nan
    return z.astype(np.float32)


def load_route(path):
    doc = json.load(open(path))
    route = [(float(a), float(b)) for a, b in doc["route"]]
    params = doc.get("params", {})
    return route, params


def run(route_path, seconds=600.0, dt=0.05, depth_hz=DEPTH_HZ, trace=None, model="dynamics", seed=0,
        noise=True, stop_on_collision=True, est_params=None):
    """model: "dynamics" (PX4-like physics + estimator, see dynamics.py) or "kinematic" (the old
    idealised model). seed: estimator/turbulence/sensor-noise randomness. trace: optional list,
    one dict per tick (truth + estimate + attitude + events + the depth frame delivered)."""
    route, params_doc = load_route(route_path)
    prm = C.Params(route=route)
    for k, v in vars(C.Params()).items():
        if k == "route":
            continue
        if k in params_doc:
            setattr(prm, k, type(v)(params_doc[k]))
    prm.depth_border_px = 6  # 24px real-camera border, scaled for this 160-wide test camera

    rng = np.random.default_rng(seed)
    m = C.Mission(prm)
    if model == "dynamics":
        px4 = D.PX4Quad(rng, ep=D.EstimatorParams(**(est_params or {})), noise=noise)
    else:
        px4 = D.KinematicPX4()
        noise = False
    segs = segments()
    home_ne = np.array([*course_to_ne(*LAUNCH_XY), 0.0])   # world NED is relative to the launch mark
    log = []
    worst_clearance, worst_at, left_net, collision = math.inf, None, False, None
    pending = []
    t, next_depth = 0.0, 0.0
    while t < seconds and m.phase != "done":
        m.on_health(t, True)
        if t >= next_depth:
            img = render_depth(px4.p + home_ne, px4.R, segs, rng=rng if noise else None)
            if noise:
                img = stereo_noise(img, rng)
            pending.append((t + (DEPTH_LATENCY_S if noise else 0.0), t, img))
            next_depth = t + 1.0 / depth_hz
        veh = px4.vehicle()
        delivered = None
        while pending and pending[0][0] <= t + 1e-9:
            _, t_cap, img = pending.pop(0)
            # like refnav.node: the frame is levelled with the attitude current when it ARRIVES
            m.on_depth(t_cap, img, K, R_BODY_CAM, T_BODY_CAM, veh.q, now=t)
            delivered = img
        out = m.step(t, veh)

        cx, cy = LAUNCH_XY[0] + px4.p[1], LAUNCH_XY[1] + px4.p[0]
        cz = FLOOR_D - px4.p[2]
        clr = true_clearance(cx, cy, cz - GEAR_BELOW_M) - DRONE_RADIUS_M
        if clr < worst_clearance:
            worst_clearance, worst_at = clr, (t, cx, cy, m.phase)
        if outside_net(cx, cy):
            left_net = True
        if trace is not None:
            trace.append(dict(t=t, pos=tuple(px4.p), R=px4.R.copy(), yaw=px4.yaw, v=tuple(px4.v),
                              est=tuple(px4.est_pos()), est_yaw=px4.est_yaw(), armed=px4.armed, nav=px4.nav,
                              phase=m.phase, legs_done=m.legs_done, events=list(out.events), depth=delivered,
                              clr=clr))
        log.append((t, m.phase, cx, cy, clr))
        if clr < 0 and collision is None and px4.armed:
            collision = (t, cx, cy, m.phase, m.legs_done)
            if stop_on_collision:
                break
        px4.apply(t, out, dt)
        t += dt

    est_err = float(np.linalg.norm((D.Rz(px4.yaw0) @ (px4.est_pos()) + px4.place - px4.p)[:2]))
    return dict(
        result="COLLISION" if collision else m.result, why=m.why, legs=len(route), legs_done=m.legs_done,
        flight_s=t, worst_clearance_m=worst_clearance, worst_at=worst_at, collision=collision,
        left_net=left_net, counts=m.counts, log=log, model=model, seed=seed,
        place_err_m=float(np.hypot(*px4.place[:2])), place_yaw_deg=math.degrees(px4.yaw0),
        final_est_err_m=est_err,
    )


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("route", nargs="?", default=str(Path(__file__).resolve().parents[1] / "route.json"))
    ap.add_argument("--model", default="dynamics", choices=["dynamics", "kinematic"])
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--no-noise", action="store_true")
    a = ap.parse_args()
    r = run(a.route, model=a.model, seed=a.seed, noise=not a.no_noise)
    print(f"route file:       {a.route}  (model={r['model']}, seed={r['seed']})")
    print(f"result:           {r['result']}  ({r['why']})" if r["why"] else f"result:           {r['result']}")
    print(f"legs done:        {r['legs_done']} / {r['legs']}")
    print(f"flight time:      {r['flight_s']:.1f} s (sim time)")
    print(f"worst clearance:  {r['worst_clearance_m']:.3f} m (drone radius {DRONE_RADIUS_M} m already subtracted)")
    if r["worst_at"]:
        t, x, y, phase = r["worst_at"]
        print(f"                  at t={t:.1f}s, course xy=({x:.2f},{y:.2f}), phase={phase}")
    print(f"placement error:  {r['place_err_m'] * 100:.1f} cm, {r['place_yaw_deg']:+.2f} deg heading")
    print(f"VIO error at end: {r['final_est_err_m'] * 100:.1f} cm")
    print(f"counts:           {r['counts']}")
    if r["collision"]:
        t, x, y, phase, legs = r["collision"]
        print(f"\n*** GROUND-TRUTH COLLISION at t={t:.1f}s, course xy=({x:.2f},{y:.2f}), {phase}, after {legs} legs ***")
