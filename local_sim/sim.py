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

from course import LAUNCH_XY, course_to_ne, nearest_obstacle_dist, outside_net, segments  # noqa: E402

# A rough stand-in for the OAK-D S2's forward stereo pair: proportionally scaled down from the
# real 640x400 sensor used in refnav/node.py so the raycaster stays fast; same convention as
# tests/test_refnav_core.py's small test camera.
W, H, FX = 160, 100, 106.0
K = (FX, FX, W / 2, H / 2)
R_BODY_CAM = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])  # FLU <- optical, forward camera
T_BODY_CAM = np.array([0.16, 0.0, 0.0])
FLOOR_D = 0.18  # body height above the floor, as in the real harness
DRONE_RADIUS_M = 0.25  # placeholder prop-guard half-span -- replace with the real airframe once measured


def q_yaw(yaw):
    return (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


def render_depth(pos_ned, yaw, segs):
    """Depth image (m, NaN = no return) seen by a level, forward-facing camera at pos_ned/yaw,
    against an arbitrary set of (n0, e0, n1, e1, height) obstacle segments."""
    vs, us = np.mgrid[0:H, 0:W].astype(np.float64)
    d_opt = np.stack([(us - K[2]) / K[0], (vs - K[3]) / K[1], np.ones_like(us)], -1)
    d_flu = d_opt @ R_BODY_CAM.T
    d_frd = d_flu * np.array([1.0, -1.0, -1.0])
    c, s = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    d_ned = d_frd @ Rz.T
    o = np.asarray(pos_ned) + Rz @ (T_BODY_CAM * np.array([1.0, -1.0, -1.0]))
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
            hit = (t > 0) & (svals >= 0) & (svals <= 1) & (p_d <= 0) & (p_d >= -height)
            best = np.where(hit, np.minimum(best, t), best)
    best[~np.isfinite(best)] = np.nan
    return best.astype(np.float32)


class FakePX4:
    """Same fidelity as tests/test_refnav_core.py's FakePX4: obeys Offboard/arm/land, tracks the
    commanded setpoint at <= 1 m/s / 60 deg/s (well above the mission's own 0.6 m/s)."""

    def __init__(self, pos=(0.0, 0.0, 0.0), yaw=0.0):
        self.pos = np.array(pos, float)
        self.yaw = yaw
        self.armed, self.nav, self.landed = False, 4, True
        self.stream_since = None

    def apply(self, t, out, dt):
        if out.stream:
            self.stream_since = t if self.stream_since is None else self.stream_since
        else:
            self.stream_since = None
        for cmd in out.commands:
            if cmd == "offboard" and self.stream_since is not None and t - self.stream_since >= 0.5:
                self.nav = C.NAV_OFFBOARD
            elif cmd == "arm":
                self.armed = True
            elif cmd == "land":
                self.nav = C.NAV_AUTO_LAND
        if self.armed and self.nav == C.NAV_OFFBOARD and out.setpoint is not None:
            n, e, d, yaw = out.setpoint
            step = np.array([n, e, d]) - self.pos
            dist = np.linalg.norm(step)
            self.pos += step * min(1.0, 1.0 * dt / dist) if dist > 1e-9 else 0
            err = (yaw - self.yaw + math.pi) % (2 * math.pi) - math.pi
            self.yaw += max(-math.radians(60) * dt, min(math.radians(60) * dt, err))
            self.landed = self.pos[2] > -0.05
        elif self.nav == C.NAV_AUTO_LAND and self.armed:
            self.pos[2] = min(0.0, self.pos[2] + 0.7 * dt)
            if self.pos[2] >= 0.0:
                self.landed, self.armed = True, False

    def vehicle(self):
        return C.Vehicle(pos=tuple(self.pos), heading=self.yaw, pos_ok=True, q=q_yaw(self.yaw),
                         armed=self.armed, nav_state=self.nav, landed=self.landed)


def load_route(path):
    doc = json.load(open(path))
    route = [(float(a), float(b)) for a, b in doc["route"]]
    params = doc.get("params", {})
    return route, params


def run(route_path, seconds=180.0, dt=0.05, depth_hz=7.5):
    route, params_doc = load_route(route_path)
    prm = C.Params(route=route)
    for k, v in vars(C.Params()).items():
        if k == "route":
            continue
        if k in params_doc:
            setattr(prm, k, type(v)(params_doc[k]))
    prm.depth_border_px = 6  # 24px real-camera border, scaled for this 160-wide test camera

    m = C.Mission(prm)
    # yaw=0: PX4/NED heading 0 = facing north = facing +n, which is "forward" in this route's frame
    # (the launch mark faces into the first corridor, course +y -- confirmed, see local_sim/README.md)
    px4 = FakePX4(pos=(0.0, 0.0, 0.0), yaw=0.0)
    segs = segments()
    log = []
    worst_clearance = math.inf
    worst_at = None
    left_net = False
    t, next_depth = 0.0, 0.0
    while t < seconds and m.phase != "done":
        m.on_health(t, True)
        if t >= next_depth:
            img = render_depth(px4.pos, px4.yaw, segs)
            m.on_depth(t, img, K, R_BODY_CAM, T_BODY_CAM, q_yaw(px4.yaw), now=t)
            next_depth = t + 1.0 / depth_hz
        out = m.step(t, px4.vehicle())
        # ground truth: course-frame position = launch mark + (e, n) offset from home
        cx, cy = LAUNCH_XY[0] + px4.pos[1], LAUNCH_XY[1] + px4.pos[0]
        clr = nearest_obstacle_dist(cx, cy) - DRONE_RADIUS_M
        if clr < worst_clearance:
            worst_clearance, worst_at = clr, (t, cx, cy, m.phase)
        if outside_net(cx, cy):
            left_net = True
        log.append((t, m.phase, cx, cy, clr))
        px4.apply(t, out, dt)
        t += dt

    return dict(
        result=m.result, why=m.why, legs=len(route), legs_done=m.legs_done,
        flight_s=t, worst_clearance_m=worst_clearance, worst_at=worst_at,
        left_net=left_net, counts=m.counts, log=log,
    )


if __name__ == "__main__":
    route_file = sys.argv[1] if len(sys.argv) > 1 else str(Path(__file__).resolve().parents[1] / "route.json")
    r = run(route_file)
    print(f"route file:      {route_file}")
    print(f"result:           {r['result']}  ({r['why']})" if r["why"] else f"result:           {r['result']}")
    print(f"legs done:        {r['legs_done']} / {r['legs']}")
    print(f"flight time:      {r['flight_s']:.1f} s (sim decision clock, not wall time)")
    print(f"worst clearance:  {r['worst_clearance_m']:.3f} m (drone radius {DRONE_RADIUS_M} m already subtracted)")
    if r["worst_at"]:
        t, x, y, phase = r["worst_at"]
        print(f"                  at t={t:.1f}s, course xy=({x:.2f},{y:.2f}), phase={phase}")
    print(f"left net bounds:  {r['left_net']}")
    print(f"counts:           {r['counts']}")
    if r["worst_clearance_m"] < 0:
        print("\n*** GROUND-TRUTH COLLISION: this route clips an obstacle at the margin above. ***")
