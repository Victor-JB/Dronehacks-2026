"""dronehacks_nav.mapping: occupied where depth returned, free where it passed, unknown elsewhere."""
import math
import sys
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dronehacks_nav import mapping as M  # noqa: E402

# a 320 x 200 depth camera (the drone's decimated OAK stream), 73 deg HFOV
W, H = 320, 200
K = (216.0, 216.0, W / 2, H / 2)
# optical (x right, y down, z forward) -> world ENU for a camera looking east (+x), level
R_WORLD_OPT = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def wall_frame(dist_m, origin=(0.0, 0.0, 1.0)):
    """Depth of a wall dist_m east of a level, east-looking camera: optical z is dist_m everywhere."""
    return np.full((H, W), dist_m, np.float32)


def integrate(m, depth, origin=(0.0, 0.0, 1.0), stride=8, max_range_m=4.0):
    dirs, ranges = M.camera_rays(depth, K, stride)
    m.integrate(origin, dirs @ R_WORLD_OPT.T, ranges, max_range_m)


def cell(m, x, y, z):
    i, j, k = (int(math.floor((c - lo) / m.voxel)) for c, lo in zip((x, y, z), m.lo))
    return m.grid[i, j, k]


def test_camera_rays_centre_pixel_is_the_optical_axis():
    d = np.full((H, W), 2.0, np.float32)
    dirs, ranges = M.camera_rays(d, K, stride=1)
    c = (H // 2) * W + W // 2
    assert np.allclose(dirs[c], (0.0, 0.0, 1.0)) and ranges[c] == pytest.approx(2.0)
    assert np.all(ranges >= 2.0 - 1e-6)                       # off-axis rays are longer than optical z


def test_a_wall_is_occupied_the_space_before_it_free_behind_it_unknown():
    m = M.VoxelMap()
    for _ in range(3):
        integrate(m, wall_frame(3.0))
    assert cell(m, 3.02, 0.0, 1.0) > m.l_occ                  # the wall
    assert cell(m, 1.5, 0.0, 1.0) < 0                         # crossed: free
    assert cell(m, 4.0, 0.0, 1.0) == 0                        # behind the wall: never observed
    assert cell(m, -1.0, 0.0, 1.0) == 0                       # behind the camera


def test_no_return_is_unknown_not_free():
    m = M.VoxelMap()
    integrate(m, np.full((H, W), np.nan, np.float32))
    assert not m.grid.any() and len(m.occupied()) == 0


def test_beyond_max_range_carves_but_does_not_occupy():
    m = M.VoxelMap()
    integrate(m, wall_frame(6.0), max_range_m=4.0)
    assert cell(m, 3.0, 0.0, 1.0) < 0 and len(m.occupied()) == 0


def test_points_outside_the_grid_are_ignored():
    m = M.VoxelMap(size_xy_m=4.0)
    integrate(m, wall_frame(3.0), origin=(1.5, 0.0, 1.0))     # the wall at x = 4.5 is outside the 4 m box
    assert len(m.occupied()) == 0


def test_a_removed_obstacle_clears_again():
    m = M.VoxelMap()
    for _ in range(3):
        integrate(m, wall_frame(2.0))
    assert cell(m, 2.02, 0.0, 1.0) > m.l_occ
    for _ in range(3):
        integrate(m, wall_frame(3.5))                          # it was moved back to 3.5 m
    assert cell(m, 2.02, 0.0, 1.0) < m.l_occ                  # three clear views: no longer occupied
    for _ in range(3):
        integrate(m, wall_frame(3.5))
    assert cell(m, 2.02, 0.0, 1.0) < 0                        # and then free


def test_slice_is_a_map_on_a_wall():
    m = M.VoxelMap()
    for _ in range(3):
        integrate(m, wall_frame(3.0))
    s = m.slice(0.5, 1.5)
    r, c = m.pixel(3.02, 0.0)
    assert s[r, c] == M.OCCUPIED
    r0, c0 = m.pixel(0.0, 0.0)
    assert s[r0, c0 + 10] == M.FREE and s[r0, c0 - 10] == M.UNKNOWN   # east of the drone free, west unknown
    assert m.pixel(0.0, 5.0)[0] < r0                                     # north is up


def test_integrate_fits_the_compute_budget():
    m = M.VoxelMap()
    d = wall_frame(3.0)
    t = time.perf_counter()
    for _ in range(10):
        integrate(m, d)
    assert (time.perf_counter() - t) / 10 < 0.05                         # < 50 ms a frame at stride 8
