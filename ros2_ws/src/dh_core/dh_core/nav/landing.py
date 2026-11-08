"""Blue-square landing-target detection (forward RGB camera, 0 deg pitch) and odom-frame tracking. PURE numpy.

Detection, per frame:
  1. HSV threshold (configurable) for blue; orange floor and grey walls are rejected by hue/saturation.
  2. Connected components; tiny blobs and blobs touching the image border (partly out of view, size
     unknowable) are not verified.
  3. Ground footprint: each blob pixel's 3-D point from depth when valid, else the ray / floor-plane
     (z = floor_z) intersection from the camera pose. Points must lie ON the floor (|z - floor_z| <=
     floor_tol): a blue thing on a wall is rejected.
  4. Square geometry on the footprint: side lengths from PCA extents, aspect >= min_aspect, convex-hull
     fill of the PCA box >= min_fill, side within [min_side, max_side] (configurable: the real size is not
     assumed).
  5. Position uncertainty grows with range (stereo/ground-plane error) - sigma = s0 + k * range^2.
Tracking: inverse-variance fusion in the odom frame with gating; sigma inflates with time and distance
flown (VIO drift) so a target that left the FOV slowly loses confidence. `verified` needs several
consistent detections and a small sigma.
NO landing decision is made here. Descent stays disabled (see explorer.py): a forward camera loses
the target ~1.5 m before the drone is over it, so the final approach runs on the tracked estimate only.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np


@dataclass
class LandingParams:
    h_lo: float = 195.0               # hue band for blue, degrees
    h_hi: float = 255.0
    s_min: float = 0.45
    v_min: float = 0.20
    min_px: int = 25
    floor_z: float = 0.0              # VIO origin is on the ground at start
    floor_tol: float = 0.08
    min_aspect: float = 0.6
    min_fill: float = 0.75
    min_side: float = 0.20
    max_side: float = 1.50
    sigma0: float = 0.03
    sigma_k: float = 0.015            # m per m^2 of range
    min_confidence: float = 0.5


@dataclass
class Detection:
    centre: np.ndarray                # (2,) odom xy of the square centre
    side: float
    aspect: float
    fill: float
    sigma: float
    confidence: float
    n_px: int
    partial: bool


def rgb_to_hsv(rgb):
    x = rgb.astype(np.float32) / 255.0
    r, g, b = x[..., 0], x[..., 1], x[..., 2]
    mx, mn = x.max(-1), x.min(-1)
    d = mx - mn
    h = np.zeros_like(mx)
    nz = d > 1e-6
    rm = nz & (mx == r)
    gm = nz & (mx == g) & ~rm
    bm = nz & ~rm & ~gm
    h[rm] = (60 * ((g - b)[rm] / d[rm])) % 360
    h[gm] = 60 * ((b - r)[gm] / d[gm]) + 120
    h[bm] = 60 * ((r - g)[bm] / d[bm]) + 240
    s = np.where(mx > 1e-6, d / np.maximum(mx, 1e-6), 0.0)
    return h, s, mx


def label(mask):
    """4-connected components; returns a list of (rows, cols) arrays."""
    h, w = mask.shape
    seen = np.zeros_like(mask, bool)
    comps = []
    for r0, c0 in np.argwhere(mask):
        if seen[r0, c0]:
            continue
        dq = deque([(r0, c0)]); seen[r0, c0] = True
        rr, cc = [], []
        while dq:
            r, c = dq.popleft()
            rr.append(r); cc.append(c)
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                a, b = r + dr, c + dc
                if 0 <= a < h and 0 <= b < w and mask[a, b] and not seen[a, b]:
                    seen[a, b] = True
                    dq.append((a, b))
        comps.append((np.array(rr), np.array(cc)))
    return comps


def _cross2(a, b):
    return float(a[0] * b[1] - a[1] * b[0])


def _hull_area(pts):
    pts = np.unique(np.round(pts, 4), axis=0)
    if len(pts) < 3:
        return 0.0
    pts = pts[np.lexsort((pts[:, 1], pts[:, 0]))]

    def half(ps):
        out = []
        for p in ps:
            while len(out) >= 2 and _cross2(out[-1] - out[-2], p - out[-2]) <= 0:
                out.pop()
            out.append(p)
        return out
    hull = np.array(half(pts)[:-1] + half(pts[::-1])[:-1])
    x, y = hull[:, 0], hull[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def detect(rgb, depth, K, R_wc, o_w, prm: LandingParams = LandingParams()) -> List[Detection]:
    h, s, v = rgb_to_hsv(rgb)
    mask = (h >= prm.h_lo) & (h <= prm.h_hi) & (s >= prm.s_min) & (v >= prm.v_min)
    if mask.sum() < prm.min_px:
        return []
    fx, fy, cx, cy = K
    H, W = mask.shape
    out = []
    for rows, cols in label(mask):
        n = len(rows)
        if n < prm.min_px:
            continue
        partial = bool(rows.min() == 0 or cols.min() == 0 or rows.max() == H - 1 or cols.max() == W - 1)
        rays = np.stack([(cols + 0.5 - cx) / fx, (rows + 0.5 - cy) / fy, np.ones(n)], 1) @ np.asarray(R_wc).T
        z = depth[rows, cols] if depth is not None else np.full(n, np.nan)
        pts = np.asarray(o_w) + rays * z[:, None]                  # optical-z depth: point = o + z * ray
        bad = ~np.isfinite(z)
        if bad.any():                                              # no depth: floor-plane intersection
            dz = rays[bad, 2]
            t = np.where(dz < -1e-6, (prm.floor_z - o_w[2]) / dz, np.nan)
            pts[bad] = np.asarray(o_w) + rays[bad] * t[:, None]
        ok = np.isfinite(pts).all(1)
        if ok.sum() < prm.min_px:
            continue
        pts = pts[ok]
        if np.median(np.abs(pts[:, 2] - prm.floor_z)) > prm.floor_tol:
            continue                                               # not lying on the floor
        xy = pts[:, :2]
        c = xy.mean(0)
        ev, evec = np.linalg.eigh(np.cov((xy - c).T) + 1e-9 * np.eye(2))
        proj = (xy - c) @ evec
        ext = np.percentile(proj, 98, axis=0) - np.percentile(proj, 2, axis=0)
        side = float(ext.mean())
        aspect = float(ext.min() / max(ext.max(), 1e-6))
        fill = float(_hull_area(xy) / max(ext[0] * ext[1], 1e-6))
        rng = float(np.linalg.norm(c - np.asarray(o_w)[:2]))
        sigma = prm.sigma0 + prm.sigma_k * rng * rng
        conf = 1.0
        conf *= min(1.0, aspect / prm.min_aspect) ** 3
        conf *= 0.0 if not (prm.min_side <= side <= prm.max_side) else 1.0
        conf *= min(1.0, fill / prm.min_fill) ** 2
        conf *= min(1.0, n / (4 * prm.min_px))
        if partial:
            conf *= 0.5
        center = c
        out.append(Detection(center, side, aspect, min(fill, 1.0), sigma, conf, n, partial))
    return [d for d in out if d.confidence >= prm.min_confidence]


@dataclass
class TrackParams:
    gate_m: float = 0.6
    q_per_s: float = 0.0002           # sigma^2 growth per second (m^2/s): a fixed pad only drifts with VIO
                                      # (0.005 lost verification ~10 s after the pad left the view)
    q_per_m: float = 0.0004           # sigma^2 growth per metre flown (VIO drift)
    verify_n: int = 3
    verify_sigma: float = 0.12
    lost_sigma: float = 0.35          # beyond this the estimate is too poor to approach on


class TargetTracker:
    def __init__(self, prm: TrackParams = TrackParams()):
        self.prm = prm
        self.xy: Optional[np.ndarray] = None
        self.var = math.inf
        self.n = 0
        self.rejected = 0
        self.t_last = None
        self.last_pos = None

    def predict(self, t: float, pos) -> None:
        if self.xy is None or self.t_last is None:
            self.t_last, self.last_pos = t, np.asarray(pos[:2], float)
            return
        dt = max(0.0, t - self.t_last)
        dm = float(np.linalg.norm(np.asarray(pos[:2]) - self.last_pos))
        self.var += self.prm.q_per_s * dt + self.prm.q_per_m * dm
        self.t_last, self.last_pos = t, np.asarray(pos[:2], float)

    def update(self, det: Detection) -> bool:
        r = det.sigma ** 2
        if self.xy is None:
            self.xy, self.var, self.n = det.centre.copy(), r, 1
            return True
        if np.linalg.norm(det.centre - self.xy) > max(self.prm.gate_m, 3 * math.sqrt(self.var + r)):
            self.rejected += 1
            if self.rejected >= 3 and self.n < self.prm.verify_n:   # an early bad seed: restart
                self.xy, self.var, self.n, self.rejected = det.centre.copy(), r, 1, 0
            return False
        k = self.var / (self.var + r)
        self.xy = self.xy + k * (det.centre - self.xy)
        self.var = (1 - k) * self.var
        self.n += 1
        return True

    @property
    def sigma(self) -> float:
        return math.sqrt(self.var) if math.isfinite(self.var) else math.inf

    @property
    def verified(self) -> bool:
        return self.xy is not None and self.n >= self.prm.verify_n and self.sigma <= self.prm.verify_sigma

    @property
    def usable(self) -> bool:
        return self.xy is not None and self.n >= self.prm.verify_n and self.sigma <= self.prm.lost_sigma
