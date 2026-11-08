"""Hidden ground-truth world built from a CAD course (STEP -> triangle mesh -> surface voxels). SIMULATOR ONLY.

Same interface as sim_world.World (render / distance / pad / boxes), so sim_run.Sim flies it unchanged:
  * the solid parts are surface-voxelised at `pitch` (hollow parts such as the tube stay hollow),
  * an exact Euclidean distance transform gives distance-to-surface for collision scoring,
  * depth is rendered by sphere tracing that distance field (error < pitch),
  * the floor (z = 0, orange) and the blue landing square are analytic, as in sim_world.

Build the voxel file once with tools/sim/step_to_voxels.py (needs cascadio + trimesh, laptop only).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np

from .sim_world import BLUE, GREY, ORANGE


@dataclass
class VoxelWorld:
    occ: np.ndarray                     # (nx, ny, nz) bool, surface voxels
    lo: np.ndarray                      # world coords of voxel (0,0,0)'s corner
    pitch: float
    pad: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    floor: bool = True
    boxes: List[Tuple[float, ...]] = field(default_factory=list)   # unused (popups not supported here)
    parts: dict = field(default_factory=dict)                      # name -> bbox, for figures

    def __post_init__(self):
        from scipy.ndimage import distance_transform_edt
        self.lo = np.asarray(self.lo, float)
        # distance (m) from each voxel centre to the nearest occupied voxel's surface
        self.dist = (distance_transform_edt(~self.occ).astype(np.float32) - 0.5) * self.pitch
        self.dist = np.maximum(self.dist, 0.0)
        self.shape = np.array(self.occ.shape)

    @classmethod
    def load(cls, path, pad=None):
        z = np.load(path, allow_pickle=True)
        shape = tuple(int(v) for v in z["shape"])
        occ = np.unpackbits(z["occ"])[: int(np.prod(shape))].reshape(shape).astype(bool)
        parts = z["parts"].item() if "parts" in z else {}
        return cls(occ, z["lo"], float(z["pitch"]), tuple(pad if pad is not None else z["pad"]), parts=parts)

    # ------------------------------------------------------------------ queries
    def _lookup(self, p):
        idx = np.floor((p - self.lo) / self.pitch).astype(int)
        inside = np.all((idx >= 0) & (idx < self.shape), axis=-1)
        idx = np.clip(idx, 0, self.shape - 1)
        d = self.dist[idx[..., 0], idx[..., 1], idx[..., 2]]
        return np.where(inside, d, 1.0)                    # outside the grid: open air (steps 1 m)

    def distance(self, p) -> float:
        p = np.asarray(p, float)
        d = float(self._lookup(p[None])[0])
        if self.floor:
            d = min(d, p[2])
        return d

    def render(self, o, R_wc, K, w, h, max_range=8.0, color=True):
        fx, fy, cx, cy = K
        vs, us = np.mgrid[0:h, 0:w]
        rays = np.stack([(us + 0.5 - cx) / fx, (vs + 0.5 - cy) / fy, np.ones((h, w))], axis=-1).reshape(-1, 3)
        d = rays @ np.asarray(R_wc).T                      # optical z = 1, so t is the optical depth
        n = np.linalg.norm(d, axis=1)
        o = np.asarray(o, float)
        t = np.full(len(d), 0.02)
        alive = np.ones(len(d), bool)
        hit = np.zeros(len(d), bool)
        eps = 0.6 * self.pitch
        for _ in range(160):
            if not alive.any():
                break
            ia = np.flatnonzero(alive)
            p = o + d[ia] * t[ia, None]
            s = self._lookup(p)
            h_ = s < eps
            hit[ia[h_]] = True
            alive[ia[h_]] = False
            step = np.maximum(s - 0.5 * self.pitch, 0.5 * self.pitch) / n[ia]
            t[ia[~h_]] += step[~h_]
            gone = t > max_range
            alive &= ~gone
        best = np.where(hit, t, np.inf)
        kind = np.where(hit, 1, 0).astype(np.int8)
        if self.floor:
            with np.errstate(divide="ignore", invalid="ignore"):
                tz = np.where(d[:, 2] < -1e-9, -o[2] / d[:, 2], np.inf)
            f = (tz > 0) & (tz < best)
            best[f], kind[f] = tz[f], 3
        depth = best.copy()
        depth[~np.isfinite(depth) | (depth > max_range)] = np.nan
        depth = depth.reshape(h, w).astype(np.float32)
        if not color:
            return depth, None
        rgb = np.zeros((len(d), 3), np.uint8)
        rgb[kind == 1] = GREY
        fl = kind == 3
        rgb[fl] = ORANGE
        if self.pad[2] > 0 and fl.any():
            pp = o + d[fl] * best[fl, None]
            on = (np.abs(pp[:, 0] - self.pad[0]) <= self.pad[2] / 2) & (np.abs(pp[:, 1] - self.pad[1]) <= self.pad[2] / 2)
            rgb[np.flatnonzero(fl)[on]] = BLUE
        # shade walls by distance so the RGB view reads as 3-D
        sh = np.clip(1.2 - best / 6.0, 0.35, 1.0)
        m = kind == 1
        rgb[m] = (rgb[m] * sh[m, None]).astype(np.uint8)
        return depth, rgb.reshape(h, w, 3)
