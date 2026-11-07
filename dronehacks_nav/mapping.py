"""dronehacks_nav.mapping - a dense 3D occupancy map from depth frames and poses. PURE (numpy only).

Each depth ray carries two pieces of evidence: the cell where it returned is OCCUPIED, and every cell it
crossed before that is FREE. A pixel with no return is no evidence at all (unknown, never free), the same
rule refnav.core flies by. Evidence accumulates as log-odds per voxel, clamped so the map can change its
mind when something moves.

The grid is a fixed box in the odom frame (ENU, metres): size_xy_m square centred on the origin, z from
z_min_m to z_max_m, voxel_m cells. Anything outside is ignored. The default 20 x 20 x 4 m at 10 cm is
1.6 M float32 cells (6.4 MB).

Cost, for the compute budget: integrate() is a handful of vectorised numpy passes over
n_rays x (max_range_m / voxel_m) samples, so ~1000 rays to 4 m is ~40 k samples per frame.
"""
from __future__ import annotations

import numpy as np

UNKNOWN, FREE, OCCUPIED = -1, 0, 1


def camera_rays(depth_m, K, stride=8, border_px=0):
    """Depth image (metres, NaN or <= 0 = no return) -> unit rays and ranges in the camera OPTICAL frame
    (x right, y down, z forward), every stride-th pixel. Range is along the ray (not optical z)."""
    fx, fy, cx, cy = K
    d = np.asarray(depth_m, np.float32)
    h, w = d.shape
    b = int(border_px)
    vs, us = np.mgrid[b:h - b:stride, b:w - b:stride]
    z = d[vs, us].ravel()
    ray = np.stack([(us.ravel() - cx) / fx, (vs.ravel() - cy) / fy, np.ones(z.size)], axis=1)
    norm = np.linalg.norm(ray, axis=1)
    ranges = np.where(np.isfinite(z) & (z > 0.0), z * norm, np.nan)
    return ray / norm[:, None], ranges


class VoxelMap:
    def __init__(self, voxel_m=0.1, size_xy_m=20.0, z_min_m=-1.0, z_max_m=3.0,
                 l_hit=0.85, l_free=0.4, l_min=-2.0, l_max=2.0, l_occupied=0.85):
        self.voxel = float(voxel_m)
        self.lo = np.array([-size_xy_m / 2, -size_xy_m / 2, z_min_m], float)
        self.shape = tuple(int(round(s / self.voxel)) for s in (size_xy_m, size_xy_m, z_max_m - z_min_m))
        self.grid = np.zeros(self.shape, np.float32)          # log-odds; exactly 0 = never observed
        self.l_hit, self.l_free = float(l_hit), float(l_free)
        self.l_min, self.l_max, self.l_occ = float(l_min), float(l_max), float(l_occupied)
        self.frames = 0

    # ------------------------------------------------------------------ indexing
    def _index(self, pts):
        """(N,3) world points -> flat indices of the cells inside the grid (others dropped)."""
        ijk = np.floor((pts - self.lo) / self.voxel).astype(np.int64)
        inside = np.all((ijk >= 0) & (ijk < np.array(self.shape)), axis=1)
        return np.ravel_multi_index(ijk[inside].T, self.shape)

    def centre(self, flat):
        ijk = np.stack(np.unravel_index(flat, self.shape), axis=1)
        return self.lo + (ijk + 0.5) * self.voxel

    # ------------------------------------------------------------------ evidence
    def integrate(self, origin, dirs, ranges, max_range_m=4.0):
        """One frame: rays from origin (3,) along unit dirs (N,3) in the world frame, ranges (N,) metres
        (NaN = no return). Returns beyond max_range_m only carve free space up to max_range_m."""
        origin = np.asarray(origin, float)
        dirs = np.asarray(dirs, float)
        r = np.asarray(ranges, float)
        seen = np.isfinite(r)
        dirs, r = dirs[seen], r[seen]
        hit = r <= max_range_m
        carve = np.minimum(r, max_range_m) - self.voxel        # stop one cell short of the return
        step = self.voxel * 0.5
        t = np.arange(step, max_range_m, step)
        free_pts = origin + dirs[:, None, :] * t[None, :, None]
        free_pts = free_pts[t[None, :] < carve[:, None]]
        free = np.unique(self._index(free_pts))
        occ = np.unique(self._index(origin + dirs[hit] * r[hit, None]))
        g = self.grid.reshape(-1)
        g[free] -= self.l_free
        g[occ] += self.l_free + self.l_hit                   # a cell both crossed and hit counts as hit
        touched = np.concatenate([free, occ])
        v = np.clip(g[touched], self.l_min, self.l_max)
        v[v == 0.0] = -1e-3                                  # an observed cell never reads back as unknown
        g[touched] = v
        self.frames += 1

    # ------------------------------------------------------------------ queries
    def occupied(self):
        """(M,3) centres of the occupied cells."""
        return self.centre(np.flatnonzero(self.grid.reshape(-1) > self.l_occ))

    def slice(self, z_lo, z_hi):
        """Top-down occupancy of the band z_lo..z_hi: (rows, cols) int8 of UNKNOWN/FREE/OCCUPIED, row 0 =
        the +y (north) edge, column 0 = the -x (west) edge, like a map on a wall. A column is OCCUPIED if
        any cell in the band is, else FREE if any was seen free, else UNKNOWN."""
        k0 = max(0, int(np.floor((z_lo - self.lo[2]) / self.voxel)))
        k1 = min(self.shape[2], int(np.ceil((z_hi - self.lo[2]) / self.voxel)))
        band = self.grid[:, :, k0:k1]
        out = np.full(self.shape[:2], UNKNOWN, np.int8)
        out[(band < 0).any(axis=2)] = FREE
        out[(band > self.l_occ).any(axis=2)] = OCCUPIED
        return np.flipud(out.T)                               # [i=x, j=y] -> [row=-y, col=x]

    def pixel(self, x, y):
        """World (x, y) -> (row, col) in slice()."""
        col = int(np.floor((x - self.lo[0]) / self.voxel))
        row = self.shape[1] - 1 - int(np.floor((y - self.lo[1]) / self.voxel))
        return row, col
