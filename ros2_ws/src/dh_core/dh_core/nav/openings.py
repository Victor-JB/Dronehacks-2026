"""One opening evaluator for windows and tunnels, from accumulated depth evidence (the OnlineMap voxels).

Given a point P on a planned path and a horizontal travel direction n, it samples the vertical
cross-section perpendicular to n through P (and, for a tunnel, cross-sections along n), and asks:
  * is the free region containing the planned flight point ENCLOSED by observed obstacles (left, right,
    below, above)? Otherwise it is open space, not an opening;
  * shape: fill ratio of the free region in its bounding box (rectangle ~1.0, ellipse ~0.785);
  * centre, width, height, and orientation (the direction with the longest straight free run through the
    centre, within +-max_turn of n, so a tunnel is flown along its axis);
  * does the drone's collision envelope (2*(radius+margin) x 2*(half height+margin+z uncertainty)) fit
    entirely inside OBSERVED-free voxels at the centre, in every cross-section along the traversal?
Missing stereo returns never carve free space (VoxelMap), so invalid depth reads UNKNOWN and can never
look like an opening. Unreliable localization makes the verdict 'unverified' regardless of geometry.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Optional

import numpy as np

from .online_map import FREE, OCCUPIED, UNKNOWN, OnlineMap


@dataclass
class OpeningParams:
    half_span: float = 0.8              # cross-section sampled +-this around the point (m)
    res: float = 0.05
    margin: float = 0.05                # extra lateral/vertical margin beyond the body
    max_turn_deg: float = 35.0          # axis search around the travel direction
    min_observed: float = 0.95          # share of envelope cells that must be observed FREE
    slice_step: float = 0.10            # spacing of cross-sections along the traversal
    verify_len: float = 1.2             # verify cross-sections this far before and after the opening point


@dataclass
class Opening:
    kind: str                 # 'rectangular' | 'elliptical' | 'open' | 'none'
    centre: np.ndarray        # (3,) ENU
    normal: np.ndarray        # (3,) horizontal unit, direction of traversal
    width: float
    height: float
    fill: float
    enclosed: bool
    fits: bool
    observed: float           # worst share of envelope cells observed FREE over all slices
    length: float             # free straight run through the centre along the normal (m)
    verified: bool
    reason: str


def _section(m: OnlineMap, p, n, half, res):
    u = np.array([-n[1], n[0], 0.0])
    s = np.arange(-half, half + 1e-9, res)
    S, Z = np.meshgrid(s, s)                       # rows = vertical offset, cols = lateral offset
    pts = p[None, None, :] + S[..., None] * u + Z[..., None] * np.array([0, 0, 1.0])
    return m.state_at(pts.reshape(-1, 3)).reshape(S.shape), s


def _flood(free, seed):
    h, w = free.shape
    seen = np.zeros_like(free)
    if not free[seed]:
        return seen, False
    dq = deque([seed]); seen[seed] = True
    touches = False
    while dq:
        r, c = dq.popleft()
        if r in (0, h - 1) or c in (0, w - 1):
            touches = True
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            rr, cc = r + dr, c + dc
            if 0 <= rr < h and 0 <= cc < w and free[rr, cc] and not seen[rr, cc]:
                seen[rr, cc] = True
                dq.append((rr, cc))
    return seen, touches


def _envelope_ok(sec, s, cs, cz, hw, hh):
    """Share of envelope cells (centred at lateral cs, vertical cz offsets) that are observed FREE; and
    whether any is OCCUPIED."""
    lat = np.abs(s - cs) <= hw
    ver = np.abs(s - cz) <= hh
    env = sec[np.ix_(ver, lat)]
    if env.size == 0:
        return 0.0, True
    return float((env == FREE).mean()), bool((env == OCCUPIED).any())


def evaluate_opening(m: OnlineMap, p, n, loc_ok: bool = True, prm: OpeningParams = OpeningParams()) -> Opening:
    mp = m.prm
    p = np.array([p[0], p[1], mp.cruise_z], float)
    n = np.array([n[0], n[1], 0.0], float)
    n /= max(np.linalg.norm(n), 1e-9)
    hw = mp.robot_radius + prm.margin
    hh = mp.drone_half_height + mp.z_margin + mp.z_uncert
    sec, s = _section(m, p, n, prm.half_span, prm.res)
    seed = (len(s) // 2, len(s) // 2)
    region, touches = _flood(sec == FREE, seed)
    none = Opening("none", p, n, 0.0, 0.0, 0.0, False, False, 0.0, 0.0, False, "")
    if not region.any():
        none.reason = "planned point not observed free"
        return none
    rows, cols = np.nonzero(region)
    width = (cols.max() - cols.min() + 1) * prm.res
    height = (rows.max() - rows.min() + 1) * prm.res
    fill = float(region.sum()) / max(1, (cols.ptp() + 1) * (rows.ptp() + 1))
    cs, cz = s[cols].mean(), s[rows].mean()
    centre = p + cs * np.array([-n[1], n[0], 0.0]) + cz * np.array([0, 0, 1.0])
    if touches:
        return Opening("open", centre, n, width, height, fill, False, True, 1.0, math.inf, False,
                       "not enclosed: open space, no opening to verify")
    kind = "rectangular" if fill > 0.88 else ("elliptical" if fill > 0.6 else "irregular")
    # the flight height is fixed (no climbing through openings): centre only laterally
    centre[2] = mp.cruise_z
    axis, back, fwd = _best_axis(m, centre, n, hw, prm)
    run = back + fwd
    # every cross-section along the traversal (+-verify_len around the opening) must hold the envelope
    # in observed-free space
    worst, occ_any = 1.0, False
    for d in np.arange(-min(back, prm.verify_len), min(fwd, prm.verify_len) + 1e-9, prm.slice_step):
        sec_d, _ = _section(m, centre + d * axis, axis, prm.half_span, prm.res)
        f, o = _envelope_ok(sec_d, s, 0.0, 0.0, hw, hh)
        worst, occ_any = min(worst, f), occ_any or o
    fits = (width >= 2 * hw and height >= 2 * hh and not occ_any)
    reason = ""
    if not fits:
        reason = "envelope does not fit (%.2f x %.2f m opening, need %.2f x %.2f)" % (width, height, 2 * hw, 2 * hh)
    elif worst < prm.min_observed:
        reason = "insufficiently observed (%.0f%% of the envelope seen free)" % (100 * worst)
    elif not loc_ok:
        reason = "localization unreliable"
    return Opening(kind, centre, axis, width, height, fill, True, fits, worst, run, fits and worst >= prm.min_observed
                   and loc_ok, reason)


def _best_axis(m: OnlineMap, c, n, hw, prm: OpeningParams):
    """Direction (within +-max_turn of n) with the longest straight run through c whose cross-section
    holds the body width in observed-free band cells; ties go to the smallest turn."""
    cls = m.classify()
    v = m.vm.voxel
    lo = m.vm.lo[:2]
    best = (-1.0, n, 0.0, 0.0)
    base = math.atan2(n[1], n[0])
    for deg in sorted(np.arange(-prm.max_turn_deg, prm.max_turn_deg + 1e-9, 5.0), key=abs):
        a = base + math.radians(deg)
        d = np.array([math.cos(a), math.sin(a), 0.0])
        u = np.array([-d[1], d[0]])
        runs = {}
        for sign in (1, -1):
            k = 0
            while True:
                q = c[:2] + sign * (k + 1) * v * d[:2]
                ok = True
                for off in np.arange(-hw, hw + 1e-9, v):
                    r = q + off * u
                    j, i = np.floor((r - lo) / v).astype(int)
                    if not (0 <= i < cls.shape[0] and 0 <= j < cls.shape[1]) or cls[i, j] != FREE:
                        ok = False
                        break
                if not ok or k > 80:
                    break
                k += 1
            runs[sign] = k * v
        if runs[1] + runs[-1] > best[0] + 1e-9:
            best = (runs[1] + runs[-1], d, runs[-1], runs[1])
    return best[1], best[2], best[3]
