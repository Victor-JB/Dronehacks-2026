"""Online occupancy map from depth + odometry, no prior map. Wraps main's dronehacks_nav.mapping.VoxelMap
(reused unchanged: log-odds, free carving up to the return, a missing return = no evidence).

Added on top:
  * an explicit bounds box (the course need not be centred on the VIO origin),
  * the flight band: a 2-D planning grid with FREE / OCCUPIED / UNKNOWN per column,
  * the wall-overflight rule: a column is blocked if anything occupies the band, OR if something
    stands between `obstacle_min_z` and the band WITHOUT observed structure above the band (a lintel or
    tunnel roof). So a window sill or tunnel floor under an observed top is an opening; a wall top
    below the drone is not overflown. An unobserved top counts as no top (blocked),
  * visited-area memory (the drone's own body volume is known free),
  * per-column last-observed time,
  * frontiers (traversable cells bordering unknown).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from dronehacks_nav.mapping import VoxelMap, camera_rays

from ..course import Grid2D

UNKNOWN, FREE, OCCUPIED = -1, 0, 1


@dataclass
class MapParams:
    lo_xy: Tuple[float, float] = (-2.0, -2.0)
    size_xy: float = 10.0
    voxel: float = 0.05
    z_min: float = -0.10
    z_max: float = 2.40
    max_range: float = 4.0
    stride: int = 2                     # on the 160x100 sim image (= stride 4 at 320x200)
    cruise_z: float = 0.75
    drone_half_height: float = 0.12
    z_margin: float = 0.08              # clearance above/below the body
    z_uncert: float = 0.05              # altitude-estimate uncertainty
    obstacle_min_z: float = 0.15        # anything taller than this under the drone needs an observed top
    free_fraction: float = 0.5          # share of the band's voxels that must be seen free
    roof_merge_m: float = 0.15          # observed overhead structure within this distance counts as a roof
    robot_radius: float = 0.27

    @property
    def band(self) -> Tuple[float, float]:
        m = self.drone_half_height + self.z_margin + self.z_uncert
        return self.cruise_z - m, self.cruise_z + m


class OnlineMap:
    def __init__(self, prm: MapParams = MapParams()):
        self.prm = prm
        self.vm = VoxelMap(prm.voxel, prm.size_xy, prm.z_min, prm.z_max)
        self.vm.lo = np.array([prm.lo_xy[0], prm.lo_xy[1], prm.z_min], float)
        nx, ny, _ = self.vm.shape
        self.visited = np.zeros((ny, nx), bool)          # [row=y, col=x]
        self.last_seen = np.full((ny, nx), np.nan, np.float32)
        self.frames = 0

    # ------------------------------------------------------------------ evidence
    def integrate(self, depth_m, K, R_wc, o_w, t: float) -> None:
        rays, ranges = camera_rays(depth_m, K, stride=self.prm.stride)
        dirs = rays @ np.asarray(R_wc).T
        before = self._band_sum()
        self.vm.integrate(np.asarray(o_w, float), dirs, ranges, max_range_m=self.prm.max_range)
        self.last_seen[self._band_sum() != before] = t      # columns that received evidence this frame
        self.frames += 1

    def _band_sum(self):
        z0, z1 = self.prm.band
        return self.vm.grid[:, :, self._k(z0):self._k(z1) + 1].sum(axis=2).T

    def mark_visited(self, pos) -> None:
        r = self.prm.robot_radius
        g = self.grid_coords()
        X, Y = g
        m = (X - pos[0]) ** 2 + (Y - pos[1]) ** 2 <= r * r
        self.visited |= m

    # ------------------------------------------------------------------ band classification
    def _k(self, z):
        return int(np.clip(np.floor((z - self.vm.lo[2]) / self.vm.voxel), 0, self.vm.shape[2]))

    def _band_counts(self):
        p = self.prm
        g = self.vm.grid
        z0, z1 = p.band
        band = g[:, :, self._k(z0):self._k(z1) + 1]
        occ = (band > self.vm.l_occ).any(axis=2)
        nfree = (band < 0).sum(axis=2)
        nknown = (band != 0).sum(axis=2)
        return occ.T, nfree.T, band.shape[2], nknown.T

    def classify(self) -> np.ndarray:
        """(ny, nx) UNKNOWN / FREE / OCCUPIED for the flight band, with the overflight rule applied."""
        p = self.prm
        g = self.vm.grid
        occ_band, nfree, nband, _ = self._band_counts()
        z0, z1 = p.band
        below = (g[:, :, self._k(p.obstacle_min_z):self._k(z0)] > self.vm.l_occ).any(axis=2).T
        above = (g[:, :, self._k(z1) + 1:] > self.vm.l_occ).any(axis=2).T
        if above.any() and p.roof_merge_m > 0:
            # a roof/lintel is continuous but grazing views sample it sparsely: accept a top seen nearby
            above = Grid2D(self.vm.lo[:2], self.vm.voxel, above).distance() <= p.roof_merge_m
        out = np.full(occ_band.shape, UNKNOWN, np.int8)
        out[nfree >= p.free_fraction * nband] = FREE
        out[self.visited & (out == UNKNOWN)] = FREE
        out[occ_band] = OCCUPIED
        out[below & ~above] = OCCUPIED               # would be flying over a wall top
        return out

    def grid_coords(self):
        nx, ny, _ = self.vm.shape
        v = self.vm.voxel
        xs = self.vm.lo[0] + (np.arange(nx) + 0.5) * v
        ys = self.vm.lo[1] + (np.arange(ny) + 0.5) * v
        return np.meshgrid(xs, ys)

    def planning_grid(self, cls: Optional[np.ndarray] = None) -> Grid2D:
        """Grid2D (existing planner type) where OCCUPIED and UNKNOWN are both blocked."""
        c = self.classify() if cls is None else cls
        return Grid2D(self.vm.lo[:2].copy(), self.vm.voxel, c != FREE)

    def state_at(self, pts) -> np.ndarray:
        """Voxel states at 3-D points: UNKNOWN / FREE / OCCUPIED (outside the box = UNKNOWN)."""
        pts = np.atleast_2d(np.asarray(pts, float))
        ijk = np.floor((pts - self.vm.lo) / self.vm.voxel).astype(int)
        inside = np.all((ijk >= 0) & (ijk < np.array(self.vm.shape)), axis=1)
        out = np.full(len(pts), UNKNOWN, np.int8)
        v = self.vm.grid[ijk[inside, 0], ijk[inside, 1], ijk[inside, 2]]
        s = np.full(len(v), UNKNOWN, np.int8)
        s[v < 0] = FREE
        s[v > self.vm.l_occ] = OCCUPIED
        out[inside] = s
        return out

    def frontiers(self, cls: np.ndarray, traversable: np.ndarray, reach: float) -> np.ndarray:
        """(row, col) of traversable cells within `reach` metres of an UNKNOWN cell."""
        unk = cls == UNKNOWN
        d = Grid2D(self.vm.lo[:2], self.vm.voxel, unk).distance()
        return np.argwhere(traversable & (d <= reach))
