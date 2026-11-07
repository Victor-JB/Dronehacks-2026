"""Height-aware synthetic camera scene. Truth is confined to this offline sensor model."""
import numpy as np
try:
    from scipy.ndimage import distance_transform_edt
except ImportError:
    from dh_core.course import _edt_1d

    def distance_transform_edt(free):
        d = np.where(free, 1e12, 0.0)
        for axis in range(d.ndim):
            d = np.apply_along_axis(_edt_1d, axis, d)
        return np.sqrt(d)


class VoxelScene:
    def __init__(self, occupancy, origin, resolution):
        self.occ = np.asarray(occupancy, bool)  # z, y, x
        self.origin = np.asarray(origin, float)
        self.res = float(resolution)
        self.distance = distance_transform_edt(~self.occ) * self.res

    @classmethod
    def from_course(cls, course, resolution=0.05):
        # Each height has its OWN cross-section. A union footprint extruded to the
        # ceiling turns low boulders into tall walls and fills the window aperture.
        zs = np.arange(0, course.wall_height + 0.3, resolution)
        grids = [course.occupancy(resolution, z, z + resolution) for z in zs]
        return cls(np.stack([g.occ for g in grids]), [*grids[0].origin, 0.0], resolution)

    def raycast(self, origin, directions, max_range=8.0):
        d = np.asarray(directions, float)
        d = d / np.linalg.norm(d, axis=1, keepdims=True)
        o = np.asarray(origin, float)
        floor = np.full(len(d), np.inf)
        down = d[:, 2] < -1e-6
        floor[down] = max(0., o[2]) / -d[down, 2]
        t = np.zeros(len(d))
        hit = np.zeros(len(d), bool)
        alive = np.ones(len(d), bool)
        nz, ny, nx = self.occ.shape
        for _ in range(256):
            idx = np.flatnonzero(alive)
            if not len(idx):
                break
            p = o + t[idx, None] * d[idx]
            ij = np.floor((p - self.origin) / self.res).astype(int)
            inside = ((ij >= 0) & (ij < [nx, ny, nz])).all(axis=1)
            # Once outside the bounded scene a ray sees background (or the floor).
            gone = idx[~inside]
            alive[gone] = False
            t[gone] = max_range
            idx, ij = idx[inside], ij[inside]
            if not len(idx):
                continue
            occupied = self.occ[ij[:, 2], ij[:, 1], ij[:, 0]]
            hit[idx[occupied]], alive[idx[occupied]] = True, False
            dd = self.distance[ij[:, 2], ij[:, 1], ij[:, 0]]
            # Subtract the voxel diagonal to avoid stepping across thin surfaces.
            t[idx[~occupied]] += np.maximum(dd[~occupied] - self.res * 1.74, self.res * 0.20)
            alive[idx[(t[idx] >= max_range) | (t[idx] >= floor[idx])]] = False
        use_floor = (floor <= t) & (floor <= max_range)
        hit |= use_floor
        t = np.where(use_floor, floor, np.minimum(t, max_range))
        return o + d * t[:, None], hit, use_floor
