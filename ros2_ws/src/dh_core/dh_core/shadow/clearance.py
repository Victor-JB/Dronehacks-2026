"""Forward free distance + corridor coverage from one depth frame. PURE. Built on refnav.depth
(DepthFrame, clearance) - the same depth code main flies - rather than a second implementation.

Corridor: +-half_width_m across and +-band_m vertically around the body, along `bearing` (to the right
of the heading), levelled with the vehicle attitude. Result:
  free_distance  : the min_hits-th nearest return in the corridor (refnav.depth.clearance), +inf if none
  valid_fraction : the WORST fraction of rays with a return over corridor cross-sections from
                   `near_m` out to min(free_distance, reliable_range). A no-return patch in front of a
                   measured wall can hide something nearer, so it lowers coverage even when a wall is
                   seen; a cross-section no ray reaches (out of view) counts as 0.
LIMITATION: this protects one swept corridor. It is NOT full lateral or vertical collision protection:
obstacles beside the corridor, above/below the band, behind or outside the camera's field of view, and
anything closer than the stereo minimum range that returns nothing are invisible to it (the last case
does lower coverage, so it reads UNKNOWN, not clear).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from refnav import depth as D


@dataclass
class CorridorParams:
    half_width_m: float = 0.45       # >= vehicle radius 0.27 + margin
    band_m: float = 0.30             # vertical half-height around the body
    min_hits: int = 3
    stride: int = 4                  # main's drone.yaml value for the 320x200 OAK depth
    border_px: int = 6               # main's drone.yaml value
    rmin_m: float = 0.05             # near returns are kept: a close obstacle must never be filtered out
    rmax_m: float = 6.0
    near_m: float = 0.4              # first coverage cross-section
    coverage_step_m: float = 0.4
    reliable_range_m: float = 3.0    # keep equal to SafetyParams.reliable_range


@dataclass
class ClearanceResult:
    free_distance: float
    valid_fraction: float
    n_points: int
    bearing: float


def corridor_clearance(depth_m, K, R_body_cam, t_body_cam, R_level_body, bearing: float = 0.0,
                       prm: CorridorParams = CorridorParams()) -> ClearanceResult:
    f = D.DepthFrame(depth_m, K, R_body_cam, t_body_cam, R_level_body, stride=prm.stride, border_px=prm.border_px)
    pts = f.points(prm.rmin_m, prm.rmax_m)
    d = D.clearance(pts, bearing, prm.half_width_m, prm.band_m, prm.min_hits)
    far = min(d, prm.reliable_range_m)
    ranges = np.arange(prm.near_m, max(far, prm.near_m) + 1e-9, prm.coverage_step_m)
    if len(ranges) == 0 or ranges[-1] < far - 1e-6:
        ranges = np.append(ranges, far)
    cov = [_quadrant_coverage(f, bearing, prm.half_width_m, prm.band_m, float(r)) for r in ranges if r > 0]
    return ClearanceResult(float(d), float(min(cov)) if cov else 0.0, int(len(pts)), float(bearing))


def _quadrant_coverage(f: "D.DepthFrame", bearing: float, half_width: float, band: float, at_m: float) -> float:
    """Worst return fraction over the four quadrants (left/right x upper/lower) of the corridor
    cross-section at at_m. Aggregate coverage (refnav's corridor_coverage) can pass a corridor whose
    upper half is blind (textureless wall) because the floor fills the lower half; per-quadrant cannot.
    A quadrant no ray reaches is 0."""
    c, s = math.cos(bearing), math.sin(bearing)
    along_dir = f.dirs[:, 0] * c + f.dirs[:, 1] * s
    ok = along_dir > 1e-6
    t = np.where(ok, (at_m - (f.origin[0] * c + f.origin[1] * s)) / np.where(ok, along_dir, 1.0), -1.0)
    p = f.origin + f.dirs * t[:, None]
    across = -p[:, 0] * s + p[:, 1] * c
    win = ok & (t > 0) & (np.abs(across) < half_width) & (np.abs(p[:, 2]) < band)
    worst = 1.0
    for right in (False, True):
        for low in (False, True):
            q = win & ((across >= 0) == right) & ((p[:, 2] >= 0) == low)
            n = int(q.sum())
            worst = min(worst, float(f.valid[q].mean()) if n else 0.0)
    return worst
