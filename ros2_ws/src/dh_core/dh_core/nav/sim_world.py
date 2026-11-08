"""Hidden ground-truth world for offline simulation. ONLY the simulator may use this module.

Primitives: axis-aligned boxes (walls, window frames, obstacles), hollow tubes (tunnels) along x or y,
the orange floor (z = 0) and a blue landing square on it. Exact ray casting renders depth (optical z,
NaN = no return) and RGB from the drone's camera; point distances score collisions.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np

ORANGE = np.array([230, 120, 30], np.uint8)
BLUE = np.array([30, 70, 210], np.uint8)
GREY = np.array([150, 150, 150], np.uint8)
TUBE_C = np.array([110, 110, 120], np.uint8)


@dataclass
class Tube:
    axis: int                 # 0: along x, 1: along y
    a0: float                 # start / end along the axis
    a1: float
    cu: float                 # centre: the other horizontal coordinate
    cz: float                 # centre height
    r_in: float
    r_out: float


@dataclass
class World:
    boxes: List[Tuple[float, float, float, float, float, float]] = field(default_factory=list)
    tubes: List[Tube] = field(default_factory=list)
    pad: Tuple[float, float, float] = (0.0, 0.0, 0.0)        # centre x, y and side length (0 = none)
    floor: bool = True

    # ------------------------------------------------------------------ rendering
    def render(self, o, R_wc, K, w, h, max_range=8.0, color=True):
        fx, fy, cx, cy = K
        vs, us = np.mgrid[0:h, 0:w]
        rays = np.stack([(us + 0.5 - cx) / fx, (vs + 0.5 - cy) / fy, np.ones((h, w))], axis=-1).reshape(-1, 3)
        d = rays @ np.asarray(R_wc).T
        o = np.asarray(o, float)
        best = np.full(len(d), np.inf)
        kind = np.zeros(len(d), np.int8)          # 0 none, 1 box, 2 tube, 3 floor
        with np.errstate(divide="ignore", invalid="ignore"):
            for b in self.boxes:
                lo, hi = np.array(b[0::2], float), np.array(b[1::2], float)
                t1, t2 = (lo - o) / d, (hi - o) / d
                tn = np.nanmax(np.minimum(t1, t2), axis=1)
                tf = np.nanmin(np.maximum(t1, t2), axis=1)
                hit = (tf >= np.maximum(tn, 0.0)) & (tn > 1e-6) & (tn < best)
                best[hit], kind[hit] = tn[hit], 1
            for tb in self.tubes:
                t = _tube_hit(o, d, tb)
                hit = t < best
                best[hit], kind[hit] = t[hit], 2
            if self.floor:
                tz = np.where(d[:, 2] < -1e-9, -o[2] / d[:, 2], np.inf)
                hit = (tz > 0) & (tz < best)
                best[hit], kind[hit] = tz[hit], 3
        depth = best.copy()
        depth[~np.isfinite(depth) | (depth > max_range)] = np.nan
        depth = depth.reshape(h, w).astype(np.float32)
        if not color:
            return depth, None
        rgb = np.zeros((len(d), 3), np.uint8)
        rgb[kind == 1] = GREY
        rgb[kind == 2] = TUBE_C
        fl = kind == 3
        rgb[fl] = ORANGE
        if self.pad[2] > 0 and fl.any():
            p = o + d[fl] * best[fl, None]
            on = (np.abs(p[:, 0] - self.pad[0]) <= self.pad[2] / 2) & (np.abs(p[:, 1] - self.pad[1]) <= self.pad[2] / 2)
            idx = np.flatnonzero(fl)[on]
            rgb[idx] = BLUE
        return depth, rgb.reshape(h, w, 3)

    # ------------------------------------------------------------------ truth distances
    def distance(self, p) -> float:
        """Distance from point p to the nearest solid surface (negative inside a box)."""
        p = np.asarray(p, float)
        dmin = math.inf
        for b in self.boxes:
            lo, hi = np.array(b[0::2], float), np.array(b[1::2], float)
            q = np.maximum(lo - p, 0) + np.maximum(p - hi, 0)
            out = float(np.linalg.norm(q))
            if out == 0.0:
                out = -float(min((p - lo).min(), (hi - p).min()))
            dmin = min(dmin, out)
        for tb in self.tubes:
            a = p[tb.axis]
            u = p[1 - tb.axis]
            rho = math.hypot(u - tb.cu, p[2] - tb.cz)
            da = max(tb.a0 - a, 0.0, a - tb.a1)
            dr = max(tb.r_in - rho, 0.0, rho - tb.r_out)
            dmin = min(dmin, math.hypot(da, dr))
        return dmin


def _tube_hit(o, d, tb: Tube):
    """Nearest positive ray parameter hitting a hollow tube (inner/outer surfaces and end annuli)."""
    ax, ux = tb.axis, 1 - tb.axis
    ou, oz, oa = o[ux] - tb.cu, o[2] - tb.cz, o[ax]
    du, dz, da = d[:, ux], d[:, 2], d[:, ax]
    best = np.full(len(d), np.inf)
    A = du * du + dz * dz
    B = 2 * (ou * du + oz * dz)
    with np.errstate(divide="ignore", invalid="ignore"):
        for r in (tb.r_in, tb.r_out):
            C = ou * ou + oz * oz - r * r
            disc = B * B - 4 * A * C
            ok = (disc >= 0) & (A > 1e-12)
            sq = np.sqrt(np.where(ok, disc, 0))
            for t in ((-B - sq) / (2 * A), (-B + sq) / (2 * A)):
                a = oa + t * da
                good = ok & (t > 1e-6) & (a >= tb.a0) & (a <= tb.a1) & (t < best)
                best = np.where(good, t, best)
        for a_end in (tb.a0, tb.a1):
            t = (a_end - oa) / da
            u, z = ou + t * du, oz + t * dz
            rr = u * u + z * z
            good = (t > 1e-6) & (rr >= tb.r_in ** 2) & (rr <= tb.r_out ** 2) & (t < best)
            best = np.where(good, t, best)
    return best


def wall_with_window(axis, at, lo, hi, thick, z_top, win_lo, win_hi, win_z0, win_z1):
    """A wall along `axis` (0: spans x at y=at; 1: spans y at x=at) with a rectangular window."""
    out = []
    spans = [(lo, win_lo, 0.0, z_top), (win_hi, hi, 0.0, z_top), (win_lo, win_hi, 0.0, win_z0), (win_lo, win_hi, win_z1, z_top)]
    for s0, s1, z0, z1 in spans:
        if s1 - s0 > 1e-6 and z1 - z0 > 1e-6:
            out.append((s0, s1, at, at + thick, z0, z1) if axis == 0 else (at, at + thick, s0, s1, z0, z1))
    return out


def wall_with_tube(axis, at, lo, hi, thick, z_top, tube: Tube):
    """A wall with a square cut-out exactly fitting the tube's outer diameter (the tube passes through)."""
    c, r = tube.cu, tube.r_out
    return wall_with_window(axis, at, lo, hi, thick, z_top, c - r, c + r, max(0.0, tube.cz - r), tube.cz + r)
