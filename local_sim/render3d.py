"""3D flight video of a local_sim run: ray-traced chase cam over the real course geometry, a
modeled quad, and live sensor panels (FPV camera, the actual depth frames refnav was handed, a
depth point cloud / pseudo-LiDAR scan, minimap, telemetry, mission event log).

Pure numpy ray tracer + Pillow HUD + ffmpeg -- no GPU/OpenGL needed.

    cd local_sim && python3 render3d.py ../route.json ../diagrams/course-run-3d.mp4

What's real vs. dressing: the trajectory, yaw, phase, leg count, events and depth frames are the
actual local_sim run of refnav.core.Mission. Roll/pitch are derived from the trajectory's
acceleration (a multirotor has to tilt to accelerate); the FPV image, point cloud and rangefinder
are rendered from course geometry. The flight model is still local_sim's kinematic one.
"""
import math
import os
import subprocess
import sys
from multiprocessing import get_context
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sim as S  # noqa: E402
from course import (BOULDER_H, BOULDER_R, BOULDERS, HALF, LANDING_XY, LAUNCH_XY, WALL_BOXES,  # noqa: E402
                    WALL_H, nearest_obstacle_dist)

OUT_W, OUT_H = 1920, 1080
FPS = 30
SPEED = 2.0
NET_H = 3.048
ROOM = (7.5, 7.5, 6.0)
LIGHT = (np.array([0.30, 0.42, 0.86]) / np.linalg.norm([0.30, 0.42, 0.86])).astype(np.float32)
EPS = 1e-4

# materials
M_ROOMWALL, M_CEIL, M_FLOOR_OUT, M_FLOOR, M_WALL, M_ROCK, M_POLE = 1, 2, 3, 4, 5, 6, 7
M_CARBON, M_RED, M_MOTOR, M_CAM, M_BATT, M_TOP, M_GEAR, M_LED = 8, 9, 10, 11, 12, 13, 14, 15
DRONE_MATS = (M_CARBON, M_RED, M_MOTOR, M_CAM, M_BATT, M_TOP, M_GEAR)
BASE = {
    M_ROOMWALL: (0.19, 0.20, 0.23), M_CEIL: (0.11, 0.11, 0.12), M_FLOOR_OUT: (0.30, 0.30, 0.31),
    M_FLOOR: (0.53, 0.54, 0.55), M_WALL: (0.87, 0.86, 0.83), M_ROCK: (0.44, 0.41, 0.37),
    M_POLE: (0.07, 0.07, 0.08), M_CARBON: (0.06, 0.06, 0.07), M_RED: (0.78, 0.09, 0.07),
    M_MOTOR: (0.58, 0.59, 0.63), M_CAM: (0.17, 0.17, 0.19), M_BATT: (0.10, 0.21, 0.55),
    M_TOP: (0.22, 0.23, 0.25), M_GEAR: (0.09, 0.09, 0.10),
}
SPECULAR = {M_CARBON: 0.35, M_RED: 0.4, M_MOTOR: 0.7, M_CAM: 0.3, M_BATT: 0.3, M_TOP: 0.4}

GOOD, WARN, CRIT = (12, 163, 12), (250, 178, 25), (208, 59, 59)
ACCENT = (79, 209, 255)


def nrm(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


# ------------------------------------------------------------------------------------ camera
class Camera:
    def __init__(self, pos, fwd, up_hint, W, H, hfov_deg):
        self.C = np.asarray(pos, float)
        self.f = nrm(fwd)
        self.r = nrm(np.cross(self.f, up_hint))
        self.u = np.cross(self.r, self.f)
        self.W, self.H = W, H
        self.fx = (W / 2) / math.tan(math.radians(hfov_deg) / 2)

    @classmethod
    def look_at(cls, pos, target, W, H, hfov):
        return cls(pos, np.subtract(target, pos), (0, 0, 1), W, H, hfov)

    def rays(self):
        xs = (np.arange(self.W) + 0.5 - self.W / 2) / self.fx
        ys = (np.arange(self.H) + 0.5 - self.H / 2) / self.fx
        D = (self.f[None, None, :] + xs[None, :, None] * self.r[None, None, :]
             - ys[:, None, None] * self.u[None, None, :])
        D /= np.linalg.norm(D, axis=-1, keepdims=True)
        return D.astype(np.float32)

    def project(self, P):
        v = np.asarray(P, float) - self.C
        zc = v @ self.f
        with np.errstate(divide="ignore", invalid="ignore"):
            x = self.fx * (v @ self.r) / zc + self.W / 2
            y = -self.fx * (v @ self.u) / zc + self.H / 2
        return x, y, zc, np.linalg.norm(v, axis=-1)

    def rect(self, corners, pad=2):
        x, y, zc, _ = self.project(corners)
        if (zc < 0.05).any():
            return (slice(0, self.H), slice(0, self.W))
        x0, x1 = max(0, int(np.floor(x.min())) - pad), min(self.W, int(np.ceil(x.max())) + pad)
        y0, y1 = max(0, int(np.floor(y.min())) - pad), min(self.H, int(np.ceil(y.max())) + pad)
        if x0 >= x1 or y0 >= y1:
            return None
        return (slice(y0, y1), slice(x0, x1))


def box_corners(lo, hi):
    return np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])


# ------------------------------------------------------------------------- intersections
def box_hit(O, D, inv, lo, hi):
    t1 = (lo - O) * inv
    t2 = (hi - O) * inv
    tmn, tmx = np.minimum(t1, t2), np.maximum(t1, t2)
    tn, tf = tmn.max(-1), tmx.min(-1)
    hit = (tf >= tn) & (tn > EPS)
    t = np.where(hit, tn, np.inf)
    ax = tmn.argmax(-1)
    n = np.zeros(D.shape, np.float32)
    d_ax = np.take_along_axis(D, ax[..., None], -1)
    np.put_along_axis(n, ax[..., None], -np.sign(d_ax), -1)
    return t, n


def box_occ(O, inv, lo, hi):
    t1 = (lo - O) * inv
    t2 = (hi - O) * inv
    tn, tf = np.minimum(t1, t2).max(-1), np.maximum(t1, t2).min(-1)
    return (tf >= np.maximum(tn, EPS))


def cyl_hit(O, D, cx, cy, r, z0, z1):
    ox, oy, oz = O[..., 0] - cx, O[..., 1] - cy, O[..., 2]
    dx, dy, dz = D[..., 0], D[..., 1], D[..., 2]
    a = dx * dx + dy * dy + 1e-12
    b = 2 * (ox * dx + oy * dy)
    c = ox * ox + oy * oy - r * r
    disc = b * b - 4 * a * c
    ts = (-b - np.sqrt(np.maximum(disc, 0))) / (2 * a)
    zs = oz + ts * dz
    t = np.where((disc >= 0) & (ts > EPS) & (zs >= z0) & (zs <= z1), ts, np.inf)
    tc = (z1 - oz) / np.where(np.abs(dz) < 1e-9, 1e-9, dz)
    px, py = ox + tc * dx, oy + tc * dy
    cap = (tc > EPS) & (px * px + py * py <= r * r) & (tc < t)
    t = np.where(cap, tc, t)
    tt = np.where(np.isfinite(t), t, 0)
    n = np.stack([np.where(cap, 0, (ox + tt * dx) / r), np.where(cap, 0, (oy + tt * dy) / r),
                  np.where(cap, 1.0, 0.0)], -1).astype(np.float32)
    return t, n


def ell_hit(O, D, c, rad):
    o, d = (O - c) / rad, D / rad
    a = (d * d).sum(-1)
    b = 2 * (o * d).sum(-1)
    cc = (o * o).sum(-1) - 1
    disc = b * b - 4 * a * cc
    t = (-b - np.sqrt(np.maximum(disc, 0))) / (2 * a)
    t = np.where((disc >= 0) & (t > EPS), t, np.inf)
    tt = np.where(np.isfinite(t), t, 0)
    p = O + D * tt[..., None]
    n = (p - c) / (rad * rad)
    n /= np.linalg.norm(n, axis=-1, keepdims=True) + 1e-12
    return t, n.astype(np.float32)


# ------------------------------------------------------------------------------- scene
def build_scene():
    f32 = lambda v: np.array(v, np.float32)  # noqa: E731
    boxes = [(f32([x0, y0, 0.0]), f32([x1, y1, WALL_H]), M_WALL) for _, x0, x1, y0, y1 in WALL_BOXES]
    r = 0.02
    for (a, b) in (((-HALF, -HALF - r), (HALF, -HALF + r)), ((-HALF, HALF - r), (HALF, HALF + r)),
                   ((-HALF - r, -HALF), (-HALF + r, HALF)), ((HALF - r, -HALF), (HALF + r, HALF))):
        boxes.append((f32([a[0], a[1], NET_H - r]), f32([b[0], b[1], NET_H + r]), M_POLE))
    poles = [(sx, sy, 0.035, 0.0, NET_H + 0.02) for sx in (-HALF, HALF) for sy in (-HALF, HALF)]
    rocks = [(f32([cx, cy, BOULDER_H / 2]), f32([rr, rr, BOULDER_H / 2])) for cx, cy, rr in BOULDERS]
    return boxes, poles, rocks


BOXES, POLES, ROCKS = build_scene()

# drone model, body frame FLU (x fwd, y left, z up), metres; ~500 mm motor-to-motor class
ARM = 0.25
ROTOR_R = 0.127
C45 = math.cos(math.pi / 4)
E1, E2 = np.array([C45, C45, 0.0]), np.array([-C45, C45, 0.0])   # front-left / back-left arm axes
MOTORS = [ARM * E1, -ARM * E1, ARM * E2, -ARM * E2]               # FL, BR, BL, FR
BODY_BOXES = [
    ((-0.09, -0.07, -0.02), (0.09, 0.07, 0.02), M_CARBON),
    ((-0.07, -0.05, 0.02), (0.05, 0.05, 0.07), M_TOP),
    ((-0.08, -0.035, -0.06), (0.08, 0.035, -0.02), M_BATT),
    ((0.09, -0.05, -0.016), (0.125, 0.05, 0.016), M_CAM),
    ((-0.105, -0.012, 0.0), (-0.09, 0.012, 0.014), M_LED),
    ((-0.10, 0.075, -0.175), (0.10, 0.09, -0.16), M_GEAR),
    ((-0.10, -0.09, -0.175), (0.10, -0.075, -0.16), M_GEAR),
    ((-0.012, 0.075, -0.16), (0.0, 0.088, -0.02), M_GEAR),
    ((-0.012, -0.088, -0.16), (0.0, -0.075, -0.02), M_GEAR),
]
# arms in the 45-degree frame (x' along E1, y' along E2); front halves red
ARM_BOXES = [
    ((0.0, -0.012, -0.008), (ARM, 0.012, 0.012), M_RED),
    ((-ARM, -0.012, -0.008), (0.0, 0.012, 0.012), M_CARBON),
    ((-0.012, -ARM, -0.008), (0.012, 0.0, 0.012), M_RED),
    ((-0.012, 0.0, -0.008), (0.012, ARM, 0.012), M_CARBON),
]
ROTOR_Z = 0.05


def rot45(v):
    return np.stack([v[..., 0] * C45 + v[..., 1] * C45, -v[..., 0] * C45 + v[..., 1] * C45, v[..., 2]], -1)


def unrot45(v):
    return np.stack([v[..., 0] * C45 - v[..., 1] * C45, v[..., 0] * C45 + v[..., 1] * C45, v[..., 2]], -1)


def drone_R(yaw, pitch, roll):
    fwd, left = np.array([math.sin(yaw), math.cos(yaw), 0.0]), np.array([-math.cos(yaw), math.sin(yaw), 0.0])
    Ry = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
    Rx = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
    return np.column_stack([fwd, left, [0, 0, 1.0]]) @ Ry @ Rx


def drone_hit(Ob, Db, led_color):
    """Ob/Db already in body frame. Returns t, normal (body frame), material, led override."""
    t = np.full(Db.shape[:-1], np.inf, np.float32)
    n = np.zeros(Db.shape, np.float32)
    m = np.zeros(Db.shape[:-1], np.int8)
    with np.errstate(divide="ignore", invalid="ignore"):
        inv = 1.0 / np.where(np.abs(Db) < 1e-9, 1e-9, Db)
        for lo, hi, mat in BODY_BOXES:
            tt, nn = box_hit(Ob, Db, inv, np.array(lo), np.array(hi))
            c = tt < t
            t[c], n[c], m[c] = tt[c], nn[c], mat
        O45, D45 = rot45(Ob), rot45(Db)
        inv45 = 1.0 / np.where(np.abs(D45) < 1e-9, 1e-9, D45)
        for lo, hi, mat in ARM_BOXES:
            tt, nn = box_hit(O45, D45, inv45, np.array(lo), np.array(hi))
            c = tt < t
            t[c], n[c], m[c] = tt[c], unrot45(nn)[c], mat
        for mc in MOTORS:
            tt, nn = cyl_hit(Ob, Db, mc[0], mc[1], 0.024, -0.005, 0.04)
            c = tt < t
            t[c], n[c], m[c] = tt[c], nn[c], M_MOTOR
    return t, n, m


def drone_occ(Ob, Lb):
    hit = np.zeros(Ob.shape[0], bool)
    inv = 1.0 / np.where(np.abs(Lb) < 1e-9, 1e-9, Lb)
    for lo, hi, _ in BODY_BOXES:
        hit |= box_occ(Ob, inv, np.array(lo), np.array(hi))
    O45, L45 = rot45(Ob), rot45(Lb[None, :])[0]
    inv45 = 1.0 / np.where(np.abs(L45) < 1e-9, 1e-9, L45)
    for lo, hi, _ in ARM_BOXES:
        hit |= box_occ(O45, inv45, np.array(lo), np.array(hi))
    return hit


# ------------------------------------------------------------------------------ texturing
def wall_dist_xy(x, y):
    d = np.full(x.shape, np.inf, np.float32)
    for lo, hi, mat in BOXES:
        if mat != M_WALL:
            continue
        dx = np.maximum(np.maximum(lo[0] - x, 0), x - hi[0])
        dy = np.maximum(np.maximum(lo[1] - y, 0), y - hi[1])
        d = np.minimum(d, np.hypot(dx, dy))
    for c, rad in ROCKS:
        d = np.minimum(d, np.hypot(x - c[0], y - c[1]) - rad[0])
    return d


def line_mask(coord, period, width, footprint):
    w = np.maximum(width, footprint)
    m = np.abs(((coord + period / 2) % period) - period / 2)
    return np.clip(1.0 - m / w, 0, 1) * np.minimum(1.0, width / w)


def noise(P, s):
    return (np.sin(P[..., 0] * s * 1.7 + np.sin(P[..., 1] * s * 2.3)) *
            np.sin(P[..., 1] * s * 1.3 + np.sin(P[..., 2] * s * 2.9 + P[..., 0] * s)) * 0.5 + 0.5)


# ------------------------------------------------------------------------------- render
def render(cam, drone=None, t_anim=0.0, shadows=True):
    """drone: dict(p, R, armed, spin, led) or None. Returns (rgb float HxWx3, dist HxW)."""
    D = cam.rays()
    H, W = D.shape[:2]
    O = cam.C.astype(np.float32)
    T = np.full((H, W), np.inf, np.float32)
    N = np.zeros((H, W, 3), np.float32)
    M = np.zeros((H, W), np.int8)

    with np.errstate(divide="ignore", invalid="ignore", over="ignore"):
        inv = 1.0 / np.where(np.abs(D) < 1e-9, 1e-9, D)
        # room interior (exit faces)
        hi = np.array(ROOM, np.float32)
        lo = np.array([-ROOM[0], -ROOM[1], -1.0], np.float32)
        tax = np.where(D > 0, (hi - O) * inv, (lo - O) * inv)
        ax = tax.argmin(-1)
        T[:] = tax.min(-1)
        np.put_along_axis(N, ax[..., None], -np.sign(np.take_along_axis(D, ax[..., None], -1)), -1)
        M[:] = np.where(ax == 2, M_CEIL, M_ROOMWALL)
        # floor
        tf = np.where(D[..., 2] < -1e-6, -O[2] / D[..., 2], np.inf)
        c = tf < T
        px, py = O[0] + tf * D[..., 0], O[1] + tf * D[..., 1]
        T[c] = tf[c]
        N[c] = (0, 0, 1)
        M[c] = np.where((np.abs(px) <= HALF) & (np.abs(py) <= HALF), M_FLOOR, M_FLOOR_OUT)[c]

        def upd(sl, tt, nn, mat):
            Ts = T[sl]
            cc = tt < Ts
            Ts[cc] = tt[cc]
            N[sl][cc] = nn[cc]
            if np.isscalar(mat):
                M[sl][cc] = mat
            else:
                M[sl][cc] = mat[cc]

        for lo_, hi_, mat in BOXES:
            sl = cam.rect(box_corners(lo_, hi_))
            if sl is None:
                continue
            tt, nn = box_hit(O, D[sl], inv[sl], lo_, hi_)
            upd(sl, tt, nn, mat)
        for cx, cy, r, z0, z1 in POLES:
            sl = cam.rect(box_corners((cx - r, cy - r, z0), (cx + r, cy + r, z1)))
            if sl is None:
                continue
            tt, nn = cyl_hit(O, D[sl], cx, cy, r, z0, z1)
            upd(sl, tt, nn, M_POLE)
        for c_, rad in ROCKS:
            sl = cam.rect(box_corners(c_ - rad, c_ + rad))
            if sl is None:
                continue
            tt, nn = ell_hit(O, D[sl], c_, rad)
            upd(sl, tt, nn, M_ROCK)

        dsl = None
        if drone is not None:
            p, R = drone["p"], drone["R"]
            dsl = cam.rect(box_corners(p - 0.45, p + 0.45))
            if dsl is not None:
                Ob = (O - p) @ R
                Db = D[dsl] @ R
                tt, nb, mm = drone_hit(Ob, Db, drone["led"])
                upd(dsl, tt, (nb @ R.T).astype(np.float32), mm)

        P = O + D * np.where(np.isfinite(T), T, 0)[..., None]

        # ---- shading
        base = np.zeros((H, W, 3), np.float32)
        for mat, col in BASE.items():
            base[M == mat] = col
        fl = M == M_FLOOR
        if fl.any():
            Pf = P[fl]
            foot = T[fl] / cam.fx
            g = np.maximum(line_mask(Pf[:, 0] + HALF, 0.5, 0.006, foot), line_mask(Pf[:, 1] + HALF, 0.5, 0.006, foot))
            col = base[fl] * (0.94 + 0.06 * noise(Pf, 7.0))[:, None]
            col = col * (1 - 0.18 * g[:, None])
            dl = np.hypot(Pf[:, 0] - LANDING_XY[0], Pf[:, 1] - LANDING_XY[1])
            ring = np.zeros_like(dl)
            for rr in (0.76, 0.51, 0.23):
                ring = np.maximum(ring, np.clip(1 - np.abs(dl - rr) / np.maximum(0.022, foot), 0, 1))
            ring = np.maximum(ring, (dl < 0.06).astype(np.float32))
            col = col * (1 - ring[:, None]) + np.array([0.08, 0.55, 0.36]) * ring[:, None]
            sq = (np.abs(Pf[:, 0] - LAUNCH_XY[0]) < 0.13) & (np.abs(Pf[:, 1] - LAUNCH_XY[1]) < 0.13)
            col[sq] = (0.08, 0.08, 0.09)
            ao = 1 - 0.34 * np.exp(-np.maximum(wall_dist_xy(Pf[:, 0], Pf[:, 1]), 0) / 0.09)
            base[fl] = col * ao[:, None]
        wl = M == M_WALL
        if wl.any():
            Pw, Nw = P[wl], N[wl]
            top = Nw[:, 2] > 0.5
            along = np.where(np.abs(Nw[:, 0]) > 0.5, Pw[:, 1], Pw[:, 0])
            seam = line_mask(along + HALF, 0.61, 0.004, T[wl] / cam.fx)
            col = base[wl] * (1 - 0.12 * seam[:, None])
            col = col * (1 - 0.32 * np.exp(-Pw[:, 2] / 0.10))[:, None]
            col[top] = (0.28, 0.28, 0.30)
            base[wl] = col
        rk = M == M_ROCK
        if rk.any():
            base[rk] *= (0.72 + 0.28 * noise(P[rk], 14.0))[:, None]
        rw = M == M_ROOMWALL
        if rw.any():
            z = P[rw][:, 2]
            base[rw] = np.where((z > 1.1)[:, None], (0.25, 0.26, 0.29), (0.17, 0.18, 0.20))
        ce = M == M_CEIL
        emis = np.zeros((H, W), bool)
        if ce.any():
            Pc = P[ce]
            lamp = (np.abs((Pc[:, 0] % 2.5) - 1.25) < 0.45) & (np.abs((Pc[:, 1] % 2.5) - 1.25) < 0.18)
            e = np.zeros(ce.sum(), bool)
            e[lamp] = True
            emis[ce] = e
            base[emis] = (1.0, 0.97, 0.9)
        led = M == M_LED
        if led.any() and drone is not None:
            base[led] = drone["led"]
            emis |= led

        # ---- shadows
        S = np.ones((H, W), np.float32)
        if shadows:
            recv = (M == M_FLOOR) | (M == M_FLOOR_OUT) | (M == M_WALL) | (M == M_ROCK)
            ndl = (N * LIGHT).sum(-1)
            recv &= ndl > 0
            Lxy = math.hypot(LIGHT[0], LIGHT[1]) / LIGHT[2]
            invL = 1.0 / LIGHT
            for lo_, hi_, mat in BOXES:
                if mat != M_WALL:
                    continue
                s = hi_[2] * Lxy + 0.02
                cand = recv & (P[..., 0] > lo_[0] - s) & (P[..., 0] < hi_[0] + s) & \
                    (P[..., 1] > lo_[1] - s) & (P[..., 1] < hi_[1] + s) & (P[..., 2] < hi_[2])
                idx = np.nonzero(cand)
                if idx[0].size == 0:
                    continue
                Os = P[idx] + N[idx] * 2e-3
                occ = box_occ(Os, invL, lo_, hi_)
                S[idx[0][occ], idx[1][occ]] = 0.0
            for c_, rad in ROCKS:
                s = BOULDER_H * Lxy + rad[0] + 0.02
                cand = recv & (np.abs(P[..., 0] - c_[0]) < s) & (np.abs(P[..., 1] - c_[1]) < s) & (P[..., 2] < BOULDER_H)
                idx = np.nonzero(cand)
                if idx[0].size == 0:
                    continue
                Os = P[idx] + N[idx] * 2e-3
                tt, _ = ell_hit(Os, np.broadcast_to(LIGHT, Os.shape).astype(np.float32), c_, rad)
                occ = np.isfinite(tt)
                S[idx[0][occ], idx[1][occ]] = 0.0
            if drone is not None:
                p, R = drone["p"], drone["R"]
                sc = p[:2][None, None, :] - LIGHT[:2] * ((p[2] - P[..., 2]) / LIGHT[2])[..., None]
                cand = recv & (P[..., 2] < p[2]) & (np.hypot(P[..., 0] - sc[..., 0], P[..., 1] - sc[..., 1]) < 0.45)
                idx = np.nonzero(cand)
                if idx[0].size:
                    Os = (P[idx] + N[idx] * 2e-3 - p) @ R
                    occ = drone_occ(Os, LIGHT @ R)
                    S[idx[0][occ], idx[1][occ]] = 0.0

        ndl = np.clip((N * LIGHT).sum(-1), 0, 1)
        amb = 0.40 + 0.14 * N[..., 2]
        rgb = base * (amb + 0.66 * ndl * S)[..., None]
        for mat, ks in SPECULAR.items():
            sel = M == mat
            if sel.any():
                h = LIGHT[None, :] - D[sel]
                h /= np.linalg.norm(h, axis=-1, keepdims=True)
                spec = np.clip((N[sel] * h).sum(-1), 0, 1) ** 40 * ks * S[sel]
                rgb[sel] += spec[:, None]
        rgb[emis] = base[emis]

        # ---- rotors (translucent)
        if drone is not None and dsl is not None:
            p, R = drone["p"], drone["R"]
            Ob = (O - p) @ R
            Db = D[dsl] @ R
            Tb = T[dsl]
            sub = rgb[dsl]
            for k, mc in enumerate(MOTORS):
                tr = (ROTOR_Z - Ob[2]) / np.where(np.abs(Db[..., 2]) < 1e-9, 1e-9, Db[..., 2])
                qx, qy = Ob[0] + tr * Db[..., 0] - mc[0], Ob[1] + tr * Db[..., 1] - mc[1]
                rr = np.hypot(qx, qy)
                inside = (tr > 0) & (tr < Tb) & (rr < ROTOR_R)
                if not inside.any():
                    continue
                ang = np.arctan2(qy, qx)
                if drone["armed"]:
                    sgn = 1 if k in (0, 1) else -1
                    streak = np.clip(np.cos(2 * (ang - sgn * drone["spin"] - k)), 0, 1) ** 10
                    a = 0.10 + 0.14 * np.sqrt(rr / ROTOR_R) + 0.22 * streak * (rr / ROTOR_R)
                    a = np.where(rr > 0.96 * ROTOR_R, a + 0.18, a)
                else:
                    phi0 = 0.6 + k * 0.9
                    dist = np.abs(-math.sin(phi0) * qx + math.cos(phi0) * qy)
                    a = ((dist < 0.016 * (1 - 0.5 * rr / ROTOR_R) + 0.004)).astype(np.float32) * 0.95
                a = np.where(rr < 0.018, 0.95, a) * inside
                sub[:] = sub * (1 - a[..., None]) + np.array([0.05, 0.05, 0.06], np.float32) * a[..., None]
            rgb[dsl] = sub

        # ---- the net: see-through mesh, more opaque at grazing angles
        net_c = np.array([0.04, 0.04, 0.05], np.float32)
        planes = [(0, -HALF, 0.11), (0, HALF, 0.11), (1, -HALF, 0.11), (1, HALF, 0.11), (2, NET_H, 0.08)]
        for axis, val, a0 in planes:
            dn = D[..., axis]
            tp = (val - O[axis]) / np.where(np.abs(dn) < 1e-9, 1e-9, dn)
            Pp = O + D * np.where(tp > 0, tp, 0)[..., None]
            others = [i for i in range(3) if i != axis]
            ok = (tp > EPS) & (tp < T)
            for i in others:
                if i == 2:
                    ok &= (Pp[..., 2] >= 0) & (Pp[..., 2] <= NET_H)
                else:
                    ok &= np.abs(Pp[..., i]) <= HALF
            if not ok.any():
                continue
            cosang = np.maximum(np.abs(dn), 0.06)
            a = np.minimum(1 - (1 - a0) ** (1 / cosang), 0.62) * ok
            rgb = rgb * (1 - a[..., None]) + net_c * a[..., None]

    return np.clip(rgb, 0, 1), T


# ------------------------------------------------------------------------- color helpers
def turbo(x):
    x = np.clip(x, 0, 1)
    r = 0.13572138 + x * (4.61539260 + x * (-42.66032258 + x * (132.13108234 + x * (-152.94239396 + x * 59.28637943))))
    g = 0.09140261 + x * (2.19418839 + x * (4.84296658 + x * (-14.18503333 + x * (4.27729857 + x * 2.82956604))))
    b = 0.10667330 + x * (12.64194608 + x * (-60.58204836 + x * (110.36276771 + x * (-89.90310912 + x * 27.34824973))))
    return np.clip(np.stack([r, g, b], -1), 0, 1)


def depth_turbo(d, dmin=0.2, dmax=4.0):
    x = 1 - (np.nan_to_num(d, nan=dmax) - dmin) / (dmax - dmin)
    rgb = turbo(x)
    rgb[~np.isfinite(d)] = (0.02, 0.02, 0.03)
    return (rgb * 255).astype(np.uint8)


def status_rgb(clr):
    if clr is None or not math.isfinite(clr):
        return (150, 150, 150)
    return CRIT if clr < 0 else (WARN if clr < 0.15 else GOOD)


# ------------------------------------------------------------------------------- fonts
def _font(paths, size, prefer=None):
    for path in paths:
        for idx in range(0, 12):
            try:
                f = ImageFont.truetype(path, size, index=idx)
            except OSError:
                break
            if prefer is None or prefer.lower() in " ".join(f.getname()).lower():
                return f
    return ImageFont.load_default()


AVENIR = "/System/Library/Fonts/Avenir Next.ttc"
MONO = ["/System/Library/Fonts/SFNSMono.ttf", "/System/Library/Fonts/Menlo.ttc"]
FONTS = {}


def F(kind, size):
    key = (kind, size)
    if key not in FONTS:
        if kind == "mono":
            FONTS[key] = _font(MONO, size)
        elif kind == "bold":
            FONTS[key] = _font([AVENIR], size, "Demi Bold")
        elif kind == "heavy":
            FONTS[key] = _font([AVENIR], size, "Heavy")
        else:
            FONTS[key] = _font([AVENIR], size, "Medium")
    return FONTS[key]


def panel(d, box, title=None):
    x0, y0, x1, y1 = box
    d.rounded_rectangle(box, radius=10, fill=(8, 10, 14, 170), outline=(255, 255, 255, 38), width=1)
    if title:
        d.text((x0 + 12, y0 + 7), title, font=F("bold", 15), fill=ACCENT + (255,))
    return y0 + (30 if title else 8)


# ----------------------------------------------------------------------- worker globals
G = {}


def init_worker(shared):
    G.update(shared)


def course_xyz(pos_ned):
    n, e, dd = pos_ned
    return np.array([LAUNCH_XY[0] + e, LAUNCH_XY[1] + n, S.FLOOR_D - dd])


def depth_points(img, p, yaw, stride=2):
    fx, fy, cx, cy = S.K
    vs, us = np.mgrid[0:img.shape[0]:stride, 0:img.shape[1]:stride]
    z = img[vs, us]
    ok = np.isfinite(z) & (z > 0.15) & (z < 5.0)
    z, u, v = z[ok], us[ok].astype(float), vs[ok].astype(float)
    pc = np.stack([(u - cx) / fx * z, (v - cy) / fy * z, z], 1)
    pb = pc @ S.R_BODY_CAM.T + S.T_BODY_CAM
    fwd, left = np.array([math.sin(yaw), math.cos(yaw), 0]), np.array([-math.cos(yaw), math.sin(yaw), 0])
    pw = p + pb[:, :1] * fwd + pb[:, 1:2] * left + pb[:, 2:3] * np.array([0, 0, 1.0])
    return pw, pb, z


def draw_minimap(img, box, st):
    d = ImageDraw.Draw(img, "RGBA")
    y = panel(d, box, "MAP")
    x0, _, x1, y1 = box
    size = min(x1 - x0 - 20, y1 - y - 10)
    ox, oy = x0 + (x1 - x0 - size) / 2, y
    sc = size / (2 * HALF)

    def mp(x, yy):
        return (ox + (x + HALF) * sc, oy + (HALF - yy) * sc)

    d.rectangle([ox, oy, ox + size, oy + size], fill=(30, 32, 36, 230), outline=(200, 200, 200, 120))
    for _, a, b, c, e in WALL_BOXES:
        d.rectangle([mp(a, e), mp(b, c)], fill=(225, 222, 214, 255))
    for cx, cy, rr in BOULDERS:
        d.ellipse([mp(cx - rr, cy + rr), mp(cx + rr, cy - rr)], fill=(140, 120, 100, 255))
    lx, ly = mp(*LANDING_XY)
    for rr in (0.76, 0.51, 0.23):
        d.ellipse([lx - rr * sc, ly - rr * sc, lx + rr * sc, ly + rr * sc], outline=(40, 170, 110, 255), width=1)
    wp = G["waypoints"]
    d.line([mp(*LAUNCH_XY)] + [mp(*w) for w in wp], fill=(255, 255, 255, 60), width=1)
    tr = G["trail"][: st["trail_i"] + 1: 3]
    tc = G["trail_clr"][: st["trail_i"] + 1: 3]
    for i in range(1, len(tr)):
        d.line([mp(*tr[i - 1][:2]), mp(*tr[i][:2])], fill=status_rgb(tc[i]) + (255,), width=2)
    px, py = mp(st["p"][0], st["p"][1])
    yaw = st["yaw"]
    f = np.array([math.sin(yaw), -math.cos(yaw)])
    rgt = np.array([math.cos(yaw), math.sin(yaw)])
    fov = math.radians(37)
    for s_ in (-1, 1):
        dv = f * math.cos(fov) + s_ * rgt * math.sin(fov)
        d.line([(px, py), (px + dv[0] * 2.0 * sc, py + dv[1] * 2.0 * sc)], fill=ACCENT + (110,), width=1)
    tip = (px + f[0] * 9, py + f[1] * 9)
    l_ = (px - f[0] * 6 + rgt[0] * 6, py - f[1] * 6 + rgt[1] * 6)
    r_ = (px - f[0] * 6 - rgt[0] * 6, py - f[1] * 6 - rgt[1] * 6)
    d.polygon([tip, l_, r_], fill=ACCENT + (255,), outline=(0, 0, 0, 255))


def draw_lidar(img, box, st, pts_body):
    d = ImageDraw.Draw(img, "RGBA")
    y = panel(d, box, "PSEUDO-LIDAR  ·  from depth")
    x0, _, x1, y1 = box
    cx, cy = (x0 + x1) / 2, y1 - 14
    rmax_px = min((x1 - x0) / 2 - 12, cy - y - 6)
    sc = rmax_px / 3.0
    for r_m in (0.5, 1.0, 2.0, 3.0):
        rp = r_m * sc
        d.arc([cx - rp, cy - rp, cx + rp, cy + rp], 180, 360, fill=(255, 255, 255, 50))
        d.text((cx + rp - 4, cy - 12), f"{r_m:g}", font=F("mono", 10), fill=(255, 255, 255, 90))
    fov = math.radians(37)
    for s_ in (-1, 1):
        d.line([(cx, cy), (cx + s_ * math.sin(fov) * rmax_px, cy - math.cos(fov) * rmax_px)],
               fill=ACCENT + (90,), width=1)
    if pts_body is not None and len(pts_body):
        fw, lf, up = pts_body[:, 0], pts_body[:, 1], pts_body[:, 2]
        band = np.abs(up) < 0.25
        rng = np.hypot(fw, lf)
        col = (turbo(1 - (rng - 0.2) / 3.8) * 255).astype(int)
        for i in np.nonzero(band & (rng < 3.0))[0][::2]:
            qx, qy = cx - lf[i] * sc, cy - fw[i] * sc
            d.rectangle([qx - 1, qy - 1, qx + 1, qy + 1], fill=tuple(col[i]) + (255,))
    d.polygon([(cx, cy - 7), (cx - 5, cy + 4), (cx + 5, cy + 4)], fill=ACCENT + (255,))


def draw_attitude(d, cx, cy, r, roll, pitch):
    d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=(30, 60, 110, 255), outline=(255, 255, 255, 120), width=2)
    off = pitch / math.radians(30) * r
    ca, sa = math.cos(-roll), math.sin(-roll)
    pts = []
    for k in range(0, 181, 6):
        a = math.radians(k)
        pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    # ground half (approximate: clip by drawing a rotated big polygon inside the circle mask)
    ground = Image.new("L", (2 * r, 2 * r), 0)
    gd = ImageDraw.Draw(ground)
    big = [(-3 * r, off), (3 * r, off), (3 * r, 3 * r), (-3 * r, 3 * r)]
    rot = [(r + x * ca - y * sa, r + x * sa + y * ca) for x, y in big]
    gd.polygon(rot, fill=255)
    mask = Image.new("L", (2 * r, 2 * r), 0)
    ImageDraw.Draw(mask).ellipse([0, 0, 2 * r - 1, 2 * r - 1], fill=255)
    from PIL import ImageChops
    ground = ImageChops.multiply(ground, mask)
    return ground


def compass(d, cx, y, w, heading_deg):
    d.rounded_rectangle([cx - w / 2, y, cx + w / 2, y + 40], radius=8, fill=(8, 10, 14, 150))
    ppd = w / 120.0
    for deg in range(int(heading_deg) - 65, int(heading_deg) + 66):
        x = cx + (deg - heading_deg) * ppd
        if x < cx - w / 2 + 6 or x > cx + w / 2 - 6:
            continue
        dm = deg % 360
        if dm % 10 == 0:
            big = dm % 30 == 0
            d.line([(x, y + 40), (x, y + 40 - (12 if big else 7))], fill=(255, 255, 255, 200), width=1)
            if big:
                lab = {0: "N", 90: "E", 180: "S", 270: "W"}.get(dm, f"{dm:03d}")
                d.text((x, y + 13), lab, font=F("bold", 15 if len(lab) == 1 else 12), fill=(255, 255, 255, 235), anchor="mm")
    d.polygon([(cx, y + 40), (cx - 7, y + 49), (cx + 7, y + 49)], fill=ACCENT + (255,))
    d.text((cx, y + 62), f"{heading_deg % 360:05.1f}°", font=F("mono", 15), fill=(255, 255, 255, 230), anchor="mm")


def render_frame(st):
    W, H = OUT_W, OUT_H
    p = np.array(st["p"])
    R = drone_R(st["yaw"], st["pitch"], st["roll"])
    led = (0.2, 1.0, 0.35) if st["armed"] else (1.0, 0.2, 0.15)
    cam = Camera.look_at(st["cam_pos"], st["cam_tgt"], W, H, 72)
    rgb, T = render(cam, dict(p=p.astype(np.float32), R=R.astype(np.float32), armed=st["armed"],
                              spin=st["spin"], led=led))
    if "vig" not in G:
        yy, xx = np.mgrid[0:H, 0:W]
        G["vig"] = (1 - 0.30 * (((xx - W / 2) / (W / 2)) ** 2 + ((yy - H / 2) / (H / 2)) ** 2) / 2)[..., None]
    img = Image.fromarray((rgb * G["vig"] * 255).astype(np.uint8)).convert("RGBA")

    # ---- 3D overlays: trail, depth point cloud, camera frustum, next waypoint
    ov = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)

    def visible(P, tol=0.06, rel=1.03):
        x, y, zc, dist = cam.project(P)
        xi = np.clip(np.nan_to_num(x, nan=0).astype(int), 0, W - 1)
        yi = np.clip(np.nan_to_num(y, nan=0).astype(int), 0, H - 1)
        inb = (zc > 0.05) & (x >= 0) & (x < W) & (y >= 0) & (y < H)
        vis = inb & (dist <= T[yi, xi] * rel + tol)
        return x, y, vis

    tr = G["trail"][: st["trail_i"] + 1: 2]
    tc = G["trail_clr"][: st["trail_i"] + 1: 2]
    if len(tr):
        keep = np.linalg.norm(tr - p, axis=1) > 0.42
        cut = len(tr) if keep.all() else int(np.argmin(keep))
        tr, tc = tr[:cut], tc[:cut]
    if len(tr) > 1:
        x, y, vis = visible(tr, tol=0.03, rel=1.01)
        for i in range(1, len(tr)):
            if vis[i] and vis[i - 1]:
                c = status_rgb(tc[i])
                d.line([(x[i - 1], y[i - 1]), (x[i], y[i])], fill=c + (90,), width=9)
                d.line([(x[i - 1], y[i - 1]), (x[i], y[i])], fill=c + (255,), width=3)

    pts_body = None
    di = st["depth_i"]
    if di >= 0:
        dimg = G["depth"][di][1]
        pw, pts_body, z = depth_points(dimg, p, st["yaw"])
        inside = (np.abs(pw[:, 0]) <= HALF) & (np.abs(pw[:, 1]) <= HALF)
        pw, z = pw[inside], z[inside]
        if len(pw):
            x, y, vis = visible(pw)
            col = (turbo(1 - (z - 0.2) / 3.8) * 255).astype(int)
            for i in np.nonzero(vis)[0][::2]:
                d.rectangle([x[i] - 1.5, y[i] - 1.5, x[i] + 1.5, y[i] + 1.5], fill=tuple(col[i]) + (235,))

    if st["armed"]:
        cam_o = p + R @ np.array([0.125, 0, 0])
        fx = S.K[0]
        corners = []
        for u_, v_ in ((0, 0), (S.W, 0), (S.W, S.H), (0, S.H)):
            ray = np.array([1.0, -(u_ - S.W / 2) / fx, -(v_ - S.H / 2) / fx]) * 0.8
            corners.append(cam_o + R @ ray)
        x, y, zc, _ = cam.project(np.array([cam_o] + corners))
        if (zc > 0.05).all():
            for i in range(1, 5):
                d.line([(x[0], y[0]), (x[i], y[i])], fill=ACCENT + (110,), width=1)
            d.line([(x[i], y[i]) for i in (1, 2, 3, 4, 1)], fill=ACCENT + (110,), width=1)

    wp_i = st["legs_done"]
    if st["armed"] and wp_i < len(G["waypoints"]):
        wx, wy = G["waypoints"][wp_i]
        x, y, zc, dist = cam.project(np.array([[wx, wy, 1.2 + S.FLOOR_D]]))
        if zc[0] > 0.1:
            s_ = max(6, min(16, 30 / dist[0]))
            d.polygon([(x[0], y[0] - s_), (x[0] + s_, y[0]), (x[0], y[0] + s_), (x[0] - s_, y[0])],
                      outline=(255, 255, 255, 230), width=2)
            d.text((x[0] + s_ + 6, y[0] - 9), f"WP {wp_i + 1}", font=F("bold", 16), fill=(255, 255, 255, 230))
    img = Image.alpha_composite(img, ov)

    # ---- HUD
    hud = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    d = ImageDraw.Draw(hud)
    grad = Image.linear_gradient("L").resize((W, 120)).transpose(Image.FLIP_TOP_BOTTOM)
    hud.paste((0, 0, 0, 150), (0, 0, W, 120), grad.point(lambda v: int(v * 0.85)))
    d.text((32, 22), "DEAD RECKONING", font=F("heavy", 34), fill=(255, 255, 255, 255))
    d.text((34, 64), "Autonomous maze run  ·  DroneHacks 2026", font=F("medium", 18), fill=(220, 225, 230, 230))
    compass(d, W / 2 - 150, 18, 560, (math.degrees(st["yaw"])) % 360)
    blink = (st["vt"] % 1.0) < 0.6
    d.ellipse([W - 520, 30, W - 506, 44], fill=(235, 50, 50, 255 if blink else 90))
    d.text((W - 498, 26), "REC", font=F("bold", 18), fill=(255, 255, 255, 240))
    d.text((W - 440, 25), f"T+ {int(st['ts'] // 60):02d}:{st['ts'] % 60:04.1f}", font=F("mono", 20), fill=(255, 255, 255, 240))
    d.text((W - 290, 26), f"{SPEED:g}× SPEED  ·  SIM", font=F("bold", 16), fill=(255, 255, 255, 200))

    # telemetry
    x0, y0, x1, y1 = 24, 700, 520, 1002
    yy0 = panel(d, (x0, y0, x1, y1), "TELEMETRY")
    big = [("ALT", f"{st['alt']:.2f}", "m"), ("GND SPD", f"{st['gs']:.2f}", "m/s"), ("CLEARANCE", f"{st['clr'] * 100:.0f}" if math.isfinite(st['clr']) else "--", "cm")]
    for i, (lab, val, unit) in enumerate(big):
        bx = x0 + 16 + i * 160
        d.text((bx, yy0 + 2), lab, font=F("medium", 13), fill=(190, 198, 206, 255))
        col = status_rgb(st["clr"]) if lab == "CLEARANCE" else (255, 255, 255)
        d.text((bx, yy0 + 18), val, font=F("heavy", 36), fill=col + (255,))
        tw = d.textlength(val, font=F("heavy", 36))
        d.text((bx + tw + 4, yy0 + 36), unit, font=F("medium", 14), fill=(190, 198, 206, 255))
    mode = {C_NAV_OFFBOARD: "OFFBOARD", C_NAV_LAND: "AUTO.LAND"}.get(st["nav"], "POSCTL" if st["armed"] else "STANDBY")
    rows = [
        ("PHASE", st["phase"].upper()), ("MODE", mode),
        ("LEG", f"{min(st['legs_done'] + (1 if st['armed'] and st['legs_done'] < len(G['waypoints']) else 0), len(G['waypoints']))} / {len(G['waypoints'])}"),
        ("ARMED", "YES" if st["armed"] else "NO"),
        ("V/S", f"{st['vs']:+.2f} m/s"), ("RNG DOWN  VL53L1X", f"{max(st['p'][2] - 0.05, 0):.2f} m"),
        ("ROLL / PITCH", f"{math.degrees(st['roll']):+.1f}° / {math.degrees(st['pitch']):+.1f}°"),
        ("DEPTH MIN", f"{st['depth_min']:.2f} m" if math.isfinite(st['depth_min']) else "--"),
    ]
    for i, (k, v) in enumerate(rows):
        cx_ = x0 + 16 + (i % 2) * 168
        cy_ = yy0 + 80 + (i // 2) * 44
        d.text((cx_, cy_), k, font=F("medium", 12), fill=(170, 178, 186, 255))
        d.text((cx_, cy_ + 15), v, font=F("mono", 17), fill=(255, 255, 255, 255))
    ar = 62
    acx, acy = x1 - ar - 22, yy0 + 80 + 88
    ground = draw_attitude(d, acx, acy, ar, st["roll"], st["pitch"])
    brown = Image.new("RGBA", ground.size, (120, 82, 48, 255))
    hud.paste(brown, (int(acx - ar), int(acy - ar)), ground)
    d.ellipse([acx - ar, acy - ar, acx + ar, acy + ar], outline=(255, 255, 255, 150), width=2)
    d.line([(acx - 28, acy), (acx - 8, acy)], fill=(255, 210, 40, 255), width=3)
    d.line([(acx + 8, acy), (acx + 28, acy)], fill=(255, 210, 40, 255), width=3)
    d.ellipse([acx - 3, acy - 3, acx + 3, acy + 3], fill=(255, 210, 40, 255))
    d.text((acx, acy + ar + 12), "ATTITUDE", font=F("medium", 11), fill=(170, 178, 186, 255), anchor="mm")

    # event log
    ex0, ey0, ex1, ey1 = 540, 862, 1100, 1002
    yy = panel(d, (ex0, ey0, ex1, ey1), "MISSION LOG  ·  refnav.core.Mission")
    evs = [e for e in G["events"] if e[0] <= st["ts"]][-5:]
    for i, (te, msg) in enumerate(evs):
        d.text((ex0 + 14, yy + i * 20), f"{te:6.1f}s", font=F("mono", 13), fill=(150, 160, 170, 255))
        d.text((ex0 + 82, yy + i * 20), msg[:60], font=F("mono", 13), fill=(235, 240, 245, 255))

    # right column: FPV, depth, lidar, map
    rx0, rx1 = W - 480, W - 24
    fy = panel(d, (rx0, 98, rx1, 98 + 30 + 257 + 10), "FPV  ·  OAK-D S2 RGB")
    dy0 = 98 + 30 + 257 + 22
    dyy = panel(d, (rx0, dy0, rx1, dy0 + 30 + 278 + 34), "STEREO DEPTH  ·  what refnav sees")
    ly0 = dy0 + 30 + 278 + 34 + 12
    img = Image.alpha_composite(img, hud)

    pw_ = rx1 - rx0 - 20
    fcam_o = p + R @ np.array([0.13, 0, -0.005])
    fcam = Camera(fcam_o, R[:, 0], R[:, 2], pw_ * 2, 257 * 2, 74)
    frgb, _ = render(fcam, None, shadows=True)
    fimg = Image.fromarray((frgb * 255).astype(np.uint8)).resize((pw_, 257), Image.LANCZOS)
    img.paste(fimg, (rx0 + 10, fy))
    if di >= 0:
        dimg = G["depth"][di][1]
        dpic = Image.fromarray(depth_turbo(dimg)).resize((pw_, 278), Image.NEAREST)
    else:
        dpic = Image.new("RGB", (pw_, 278), (5, 5, 8))
    img.paste(dpic, (rx0 + 10, dyy))

    d = ImageDraw.Draw(img, "RGBA")
    d.text((rx0 + 20, fy + 8), f"{st['ts']:6.1f}s", font=F("mono", 13), fill=(255, 255, 255, 220))
    d.rectangle([rx0 + 10, fy, rx0 + 10 + pw_, fy + 257], outline=(255, 255, 255, 60))
    d.rectangle([rx0 + 10, dyy, rx0 + 10 + pw_, dyy + 278], outline=(255, 255, 255, 60))
    cb_y = dyy + 278 + 8
    bar = (turbo(np.linspace(1, 0, pw_ - 120))[None, :, :] * 255).astype(np.uint8).repeat(10, 0)
    img.paste(Image.fromarray(bar), (rx0 + 10 + 40, cb_y + 4))
    d.text((rx0 + 12, cb_y + 1), "0.2m", font=F("mono", 11), fill=(220, 220, 220, 255))
    d.text((rx0 + pw_ - 64, cb_y + 1), "4.0m", font=F("mono", 11), fill=(220, 220, 220, 255))
    d.text((rx0 + pw_ - 24, cb_y + 1), "", font=F("mono", 11))
    if di >= 0:
        valid = float(np.isfinite(G["depth"][di][1]).mean()) * 100
        d.text((rx0 + 20, dyy + 8), f"160×100  ·  {valid:.0f}% valid  ·  min {st['depth_min']:.2f} m",
               font=F("mono", 13), fill=(255, 255, 255, 235))
    half = (rx1 - rx0 - 12) // 2
    draw_lidar(img, (rx0, ly0, rx0 + half, H - 60), st, pts_body)
    draw_minimap(img, (rx0 + half + 12, ly0, rx1, H - 60), st)

    # progress bar w/ leg ticks
    d = ImageDraw.Draw(img, "RGBA")
    bx0, bx1, by = 24, W - 504, 1030
    d.rounded_rectangle([bx0, by, bx1, by + 8], radius=4, fill=(255, 255, 255, 60))
    frac = min(1.0, st["ts"] / G["t_end"])
    d.rounded_rectangle([bx0, by, bx0 + max(8, frac * (bx1 - bx0)), by + 8], radius=4, fill=ACCENT + (255,))
    for tl in G["leg_times"]:
        x = bx0 + tl / G["t_end"] * (bx1 - bx0)
        d.line([(x, by - 3), (x, by + 11)], fill=(255, 255, 255, 200 if tl <= st["ts"] else 80), width=2)
    d.text((bx0, by + 16), "Rendered from local_sim: real refnav.core.Mission decisions on the published course CAD  ·  "
           "kinematic flight model, simulated sensors, attitude derived from trajectory", font=F("medium", 13),
           fill=(200, 205, 210, 200))

    # intro / outro cards
    if st["intro"] > 0:
        a = st["intro"]
        card = Image.new("RGBA", (W, H), (0, 0, 0, 0))
        cd = ImageDraw.Draw(card)
        cd.rectangle([0, H / 2 - 120, W, H / 2 + 110], fill=(5, 8, 12, int(185 * a)))
        cd.text((W / 2, H / 2 - 48), "DEAD RECKONING", font=F("heavy", 86), fill=(255, 255, 255, int(255 * a)), anchor="mm")
        cd.text((W / 2, H / 2 + 34), "Fully autonomous, GPS-denied maze run  ·  26 waypoints  ·  DroneHacks 2026",
                font=F("medium", 28), fill=(210, 230, 240, int(255 * a)), anchor="mm")
        img = Image.alpha_composite(img, card)
    if st["outro"] > 0:
        a = st["outro"]
        card = Image.new("RGBA", (W, H), (0, 0, 0, 0))
        cd = ImageDraw.Draw(card)
        cd.rectangle([0, H / 2 - 130, W - 500, H / 2 + 120], fill=(5, 8, 12, int(195 * a)))
        cx = (W - 500) / 2
        ok = G["result"] == "complete"
        cd.text((cx, H / 2 - 60), "MISSION COMPLETE" if ok else str(G["result"]).upper()[:28], font=F("heavy", 80),
                fill=((110, 230, 140) if ok else (240, 120, 100)) + (int(255 * a),), anchor="mm")
        cd.text((cx, H / 2 + 22), G["summary"], font=F("medium", 28), fill=(235, 240, 245, int(255 * a)), anchor="mm")
        cd.text((cx, H / 2 + 70), G["summary2"], font=F("medium", 22), fill=(190, 200, 210, int(255 * a)), anchor="mm")
        img = Image.alpha_composite(img, card)

    return img.convert("RGB").tobytes()


C_NAV_OFFBOARD = S.C.NAV_OFFBOARD
C_NAV_LAND = S.C.NAV_AUTO_LAND


# ------------------------------------------------------------------------------- timeline
def gsmooth(a, sigma):
    if sigma <= 0:
        return a
    r = int(3 * sigma) + 1
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma) ** 2)
    k /= k.sum()
    pad = np.pad(a, [(r, r)] + [(0, 0)] * (a.ndim - 1), mode="edge")
    out = np.empty_like(a)
    for idx in np.ndindex(a.shape[1:]) if a.ndim > 1 else [()]:
        sl = (slice(None),) + idx
        out[sl] = np.convolve(pad[sl], k, mode="valid")
    return out


def smoothstep(x):
    x = min(1.0, max(0.0, x))
    return x * x * (3 - 2 * x)


def build_timeline(trace, route, result):
    t = np.array([e["t"] for e in trace])
    pos = np.array([course_xyz(e["pos"]) for e in trace])
    yaw = np.unwrap(np.array([e["yaw"] for e in trace]))
    dt = t[1] - t[0]
    vel = np.gradient(gsmooth(pos, 0.15 / dt), dt, axis=0)
    vel_s = gsmooth(vel, 0.25 / dt)
    acc = gsmooth(np.gradient(vel_s, dt, axis=0), 0.2 / dt)
    fwd = np.stack([np.sin(yaw), np.cos(yaw)], 1)
    left = np.stack([-np.cos(yaw), np.sin(yaw)], 1)
    a_f, a_l = (acc[:, :2] * fwd).sum(1), (acc[:, :2] * left).sum(1)
    pitch = np.clip(np.arctan(a_f / 9.81), -0.35, 0.35)
    roll = np.clip(np.arctan(-a_l / 9.81), -0.35, 0.35)
    clr = np.array([nearest_obstacle_dist(x, y) - S.DRONE_RADIUS_M for x, y, _ in pos])

    depth = [(e["t"], e["depth"]) for e in trace if e["depth"] is not None]
    depth_t = np.array([dd[0] for dd in depth])
    events = [(e["t"], ev) for e in trace for ev in e["events"]]
    leg_times = [trace[i]["t"] for i in range(1, len(trace)) if trace[i]["legs_done"] > trace[i - 1]["legs_done"]]
    waypoints = [(LAUNCH_XY[0] - l, LAUNCH_XY[1] + f) for f, l in route]
    t_end = t[-1]

    n_main = int(t_end / SPEED * FPS) + 1
    n_out = int(4.0 * FPS)
    wide_pos, wide_tgt = np.array([3.6, -5.6, 5.4]), np.array([-0.7, 0.1, 0.0])
    frames = []
    cam_p, cam_t, cyaw = None, None, yaw[0]
    for k in range(n_main + n_out):
        vt = k / FPS
        outro_u = max(0.0, (k - n_main) / FPS)
        ts = min(t_end, vt * SPEED)
        i = min(int(np.searchsorted(t, ts, side="right")) - 1, len(t) - 1)
        i = max(i, 0)
        j = min(i + 1, len(t) - 1)
        w = 0.0 if j == i else (ts - t[i]) / (t[j] - t[i])
        p = pos[i] * (1 - w) + pos[j] * w
        ya = yaw[i] * (1 - w) + yaw[j] * w
        a_ = 1 - math.exp(-(1 / FPS) / 0.55)
        cyaw = ya if k == 0 else cyaw + (ya - cyaw) * (1 - math.exp(-(1 / FPS) / 0.7))
        f2 = np.array([math.sin(cyaw), math.cos(cyaw), 0.0])
        want_p = p - f2 * 2.1 + np.array([0, 0, 2.25])
        want_t = p + f2 * 0.6 - np.array([0, 0, 0.15])
        cam_p = want_p if cam_p is None else cam_p + (want_p - cam_p) * a_
        cam_t = want_t if cam_t is None else cam_t + (want_t - cam_t) * a_
        s_in = smoothstep((vt - 0.9) / 3.0)
        s_out = smoothstep(outro_u / 2.6)
        s = s_in * (1 - s_out)
        cp = wide_pos * (1 - s) + cam_p * s
        ct = wide_tgt * (1 - s) + cam_t * s
        di = int(np.searchsorted(depth_t, ts, side="right")) - 1
        dmin = float(np.nanmin(depth[di][1])) if di >= 0 and np.isfinite(depth[di][1]).any() else math.inf
        armed = trace[i]["armed"]
        frames.append(dict(
            k=k, vt=vt, ts=ts, p=tuple(p), yaw=float(ya), pitch=float(pitch[i]) if armed else 0.0,
            roll=float(roll[i]) if armed else 0.0, armed=armed, nav=trace[i]["nav"], phase=trace[i]["phase"],
            legs_done=trace[i]["legs_done"], cam_pos=tuple(cp), cam_tgt=tuple(ct), trail_i=i, depth_i=di,
            gs=float(np.hypot(*vel_s[i, :2])), vs=float(vel_s[i, 2]) + 0.0,
            alt=round(float(-trace[i]["pos"][2]), 2) + 0.0,
            clr=float(clr[i]), depth_min=dmin, spin=vt * 9.7,
            intro=1.0 - smoothstep((vt - 2.2) / 0.8) if vt < 3.0 else 0.0,
            outro=smoothstep((outro_u - 0.4) / 0.8),
        ))
    shared = dict(trail=pos, trail_clr=clr, depth=depth, events=events, leg_times=leg_times,
                  waypoints=waypoints, t_end=t_end, result=result["result"],
                  summary2=f"{result['counts']['depth_frames']} depth frames checked  ·  "
                           f"{result['counts']['obstacle_stops']} obstacle stops  ·  "
                           f"{result['counts']['depth_holds']} depth holds",
                  summary=f"{result['legs_done']}/{result['legs']} waypoints  ·  {t_end:.1f} s  ·  "
                          f"tightest clearance {result['worst_clearance_m'] * 100:.1f} cm")
    return frames, shared


def main():
    route_path = sys.argv[1] if len(sys.argv) > 1 else str(Path(__file__).resolve().parents[1] / "route.json")
    out_path = Path(sys.argv[2]) if len(sys.argv) > 2 else Path(__file__).resolve().parents[1] / "diagrams" / "course-run-3d.mp4"
    only = int(os.environ.get("ONLY_FRAME", "-1"))
    route, _ = S.load_route(route_path)
    trace = []
    result = S.run(route_path, trace=trace)
    print(f"mission: {result['result']}  legs {result['legs_done']}/{result['legs']}  {result['flight_s']:.1f}s sim")
    frames, shared = build_timeline(trace, route, result)
    print(f"{len(frames)} frames @ {FPS} fps ({len(frames) / FPS:.1f}s video)")

    if only >= 0:
        init_worker(shared)
        Image.frombytes("RGB", (OUT_W, OUT_H), render_frame(frames[only])).save(out_path)
        print(f"wrote still {out_path}")
        return

    out_path.parent.mkdir(parents=True, exist_ok=True)
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                           "-s", f"{OUT_W}x{OUT_H}", "-r", str(FPS), "-i", "-", "-c:v", "libx264",
                           "-preset", "medium", "-crf", "19", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
                           str(out_path)], stdin=subprocess.PIPE)
    ctx = get_context("spawn")
    with ctx.Pool(max(1, (os.cpu_count() or 2) - 1), initializer=init_worker, initargs=(shared,)) as pool:
        for n, buf in enumerate(pool.imap(render_frame, frames, chunksize=2)):
            ff.stdin.write(buf)
            if n % 60 == 0:
                print(f"  frame {n}/{len(frames)}", flush=True)
    ff.stdin.close()
    ff.wait()
    print(f"wrote {out_path} ({out_path.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
