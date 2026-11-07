"""Map-relative re-localisation by direct scan-to-distance-field registration.

Idea from DLL (Caballero & Merino, IROS 2021): instead of matching features,
minimise the sum of squared map-distance values at the scan points,
    min_{x,y,yaw}  sum_i  rho( D( R(yaw) p_i + t ) )
where D is the Euclidean distance field of the *known* course (built from the
published model, which the rules allow) and rho is a Huber loss. Solved with
Gauss-Newton using bilinear interpolation of D and its gradient.

Input scan points can come from stereo depth (a horizontal band of the depth
image projected to the cruise plane), from a 2D LiDAR, or from ToF sensors.
Output: a corrected map pose plus a health measure (inlier ratio, residual,
Hessian conditioning) so the caller can reject degenerate fixes (e.g. inside
the featureless tube, where the along-axis direction is unobservable).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from .course import Grid2D


@dataclass
class MatchResult:
    pose: np.ndarray          # (x, y, yaw)
    residual_rms: float
    inlier_ratio: float
    min_eig: float            # smallest eigenvalue of the position block of J^T J (observability)
    iterations: int
    ok: bool


class DistanceFieldMatcher:
    def __init__(self, grid: Grid2D, max_dist: float = 0.5, huber: float = 0.08, close_radius: float = 0.08,
                 eig_threshold: float = 0.05, estimate_yaw: bool = False):
        if close_radius > 0:
            grid = grid.closed(close_radius)
        self.grid = grid
        self.eig_threshold = eig_threshold
        self.estimate_yaw = estimate_yaw
        # Signed distance to the obstacle surface (negative inside). With an
        # unsigned field, points pushed *into* a wall would cost nothing and the
        # optimum would be biased into the walls.
        outside = grid.distance()
        inside = Grid2D(grid.origin.copy(), grid.res, ~grid.occ).distance()
        sdf = np.where(grid.occ, -(inside - 0.5 * grid.res), outside - 0.5 * grid.res)
        self.D = np.clip(sdf, -max_dist, max_dist)
        gy, gx = np.gradient(self.D, grid.res)
        self.gx, self.gy = gx, gy
        self.huber = huber
        self.max_dist = max_dist

    def _interp(self, img: np.ndarray, xy: np.ndarray) -> np.ndarray:
        g = self.grid
        f = (xy - g.origin) / g.res - 0.5
        c0 = np.floor(f[:, 0]).astype(int); r0 = np.floor(f[:, 1]).astype(int)
        ny, nx = img.shape
        c0 = np.clip(c0, 0, nx - 2); r0 = np.clip(r0, 0, ny - 2)
        ax = np.clip(f[:, 0] - c0, 0, 1); ay = np.clip(f[:, 1] - r0, 0, 1)
        return ((1 - ax) * (1 - ay) * img[r0, c0] + ax * (1 - ay) * img[r0, c0 + 1] +
                (1 - ax) * ay * img[r0 + 1, c0] + ax * ay * img[r0 + 1, c0 + 1])

    def match(self, pts_body: np.ndarray, prior: np.ndarray, iters: int = 15,
              prior_sigma: Tuple[float, float] = (0.5, 0.2)) -> MatchResult:
        """pts_body: (N,2) scan points in the body frame (x fwd, y left, metres)."""
        x = np.asarray(prior, float).copy()
        P = np.asarray(pts_body, float)
        if len(P) < 10:
            return MatchResult(x, np.inf, 0.0, 0.0, 0, False)
        w_prior = np.array([1 / prior_sigma[0] ** 2] * 2 + [1 / prior_sigma[1] ** 2]) * 1e-3
        it = 0
        for it in range(1, iters + 1):
            c, s = math.cos(x[2]), math.sin(x[2])
            W = P @ np.array([[c, s], [-s, c]]) + x[:2]
            r = self._interp(self.D, W)
            gx = self._interp(self.gx, W); gy = self._interp(self.gy, W)
            # d(world)/d(yaw) = [-s px - c py, c px - s py]
            dwx = -s * P[:, 0] - c * P[:, 1]
            dwy = c * P[:, 0] - s * P[:, 1]
            J = np.stack([gx, gy, gx * dwx + gy * dwy], axis=1)
            a = np.abs(r)
            w = np.where(a <= self.huber, 1.0, self.huber / np.maximum(a, 1e-9))
            w = np.where(a >= self.max_dist - 1e-6, 0.0, w)   # far outliers carry no gradient
            nd = 3 if self.estimate_yaw else 2
            Jn = J[:, :nd]
            H = (Jn * w[:, None]).T @ Jn + np.diag(w_prior[:nd])
            g = (Jn * w[:, None]).T @ r + w_prior[:nd] * (x - prior)[:nd]
            try:
                dx = np.zeros(3)
                dx[:nd] = -np.linalg.solve(H, g)
            except np.linalg.LinAlgError:
                break
            dx[:2] = np.clip(dx[:2], -0.1, 0.1); dx[2] = np.clip(dx[2], -0.05, 0.05)
            x += dx
            if np.linalg.norm(dx[:2]) < 1e-4 and abs(dx[2]) < 1e-4:
                break
        c, s = math.cos(x[2]), math.sin(x[2])
        W = P @ np.array([[c, s], [-s, c]]) + x[:2]
        r = self._interp(self.D, W)
        inl = np.abs(r) < 0.06
        rms = float(np.sqrt(np.mean(r[inl] ** 2))) if inl.any() else np.inf
        # Observability: only keep the correction along well-constrained position
        # directions (a scan of one straight wall fixes the distance to it but
        # not the position along it).
        Hp = (J[:, :2] * w[:, None]).T @ J[:, :2] / max(len(P), 1)
        evals, evecs = np.linalg.eigh(Hp)
        delta = x[:2] - prior[:2]
        keep = np.zeros(2)
        for k in range(2):
            if evals[k] > self.eig_threshold:
                keep += (evecs[:, k] @ delta) * evecs[:, k]
        out = x.copy()
        out[:2] = prior[:2] + keep
        ok = inl.mean() > 0.6 and evals[-1] > self.eig_threshold and np.isfinite(rms)
        return MatchResult(out, rms, float(inl.mean()), float(evals[0]), it, bool(ok))


def raycast_scan(grid: Grid2D, pose: np.ndarray, fov_deg: float = 80.0, n_beams: int = 64,
                 r_min: float = 0.3, r_max: float = 4.0, noise: float = 0.02,
                 rng: Optional[np.random.Generator] = None) -> np.ndarray:
    """Synthetic forward scan (body-frame points) against an occupancy grid; for tests/sim."""
    rng = rng or np.random.default_rng()
    A = np.radians(np.linspace(-fov_deg / 2, fov_deg / 2, n_beams))
    R = np.arange(r_min, r_max, grid.res * 0.5)
    th = pose[2] + A
    xy = pose[:2][None, None, :] + R[None, :, None] * np.stack([np.cos(th), np.sin(th)], 1)[:, None, :]
    ij = np.floor((xy - grid.origin) / grid.res).astype(int)            # same cell rule as Grid2D.to_cell
    rows = np.clip(ij[..., 1], 0, grid.occ.shape[0] - 1)
    cols = np.clip(ij[..., 0], 0, grid.occ.shape[1] - 1)
    hit = grid.occ[rows, cols]                                          # (n_beams, n_ranges)
    any_hit = hit.any(1)
    first = hit.argmax(1)
    pts = []
    for k in np.nonzero(any_hit)[0]:                                    # beam order kept (same noise draws)
        rr = R[first[k]] + rng.normal(0, noise)
        pts.append([rr * math.cos(A[k]), rr * math.sin(A[k])])
    return np.array(pts)
