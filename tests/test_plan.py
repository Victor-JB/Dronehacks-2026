"""refnav.plan (occupancy grid + A*) and the mission's use of it: pure numpy, no ROS, no simulator."""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from refnav import core as C  # noqa: E402
from refnav import plan as P  # noqa: E402


def box_grid(**kw):
    """A 6 x 6 m room, a wall across the middle (n = 3) with a 1 m gap at e in [4, 5]."""
    g = P.Grid((0.0, 6.0, 0.0, 6.0), res=0.05, **kw)
    g.add_polygon([(2.9, 0.0), (3.1, 0.0), (3.1, 4.0), (2.9, 4.0)])
    g.add_polygon([(2.9, 5.0), (3.1, 5.0), (3.1, 6.0), (2.9, 6.0)])
    return g


def min_clearance(g, path):
    clr = g.clearance()
    worst = math.inf
    for a, b in zip(path, path[1:]):
        for t in np.linspace(0, 1, 40):
            r, c = g.cell(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
            worst = min(worst, clr[r, c])
    return worst


def test_distance_transform_matches_euclid():
    occ = np.zeros((60, 60), bool)
    occ[30, 30] = True
    d = P._distance(occ, 0.05)
    assert abs(d[30, 40] - 0.5) < 0.03
    assert abs(d[40, 40] - 0.05 * math.hypot(10, 10)) < 0.05 * 14.2 * 0.09     # chamfer: within ~8%
    assert d[30, 30] == 0.0
    assert abs(d[0, 0] - 0.05) < 1e-9 or d[0, 0] <= 0.05                       # the grid edge is an obstacle


def test_path_goes_through_the_gap_and_stays_clear():
    g = box_grid()
    path = g.plan((1.0, 1.0), (5.0, 1.0), 0.3)
    assert path is not None and path[0] == (1.0, 1.0) and path[-1] == (5.0, 1.0)
    assert any(abs(p[0] - 3.0) < 0.3 and 4.0 < p[1] < 5.0 for p in path) or min(p[1] for p in path) >= 0  # crosses n=3 in the gap
    assert min_clearance(g, path) >= 0.3 - 0.06


def crossing_e(path, n):
    """Where the polyline crosses the line northing == n (interpolated), or None."""
    for a, b in zip(path, path[1:]):
        if (a[0] - n) * (b[0] - n) <= 0 and a[0] != b[0]:
            return a[1] + (b[1] - a[1]) * (n - a[0]) / (b[0] - a[0])
    return None


def test_prefers_the_middle_of_a_corridor():
    g = box_grid(prefer_m=0.6)
    # the gap is e in [4.0, 5.0]; start off-centre at e=4.2: the path should still cross near 4.5
    path = g.plan((1.0, 4.2), (5.0, 4.2), 0.3)
    assert path is not None
    e = crossing_e(path, 3.0)
    assert e is not None and abs(e - 4.5) < 0.12, f"crossed the wall line at e={e}"


def test_unreachable_goal_returns_none():
    g = box_grid()
    g.add_polygon([(2.9, 3.9), (3.1, 3.9), (3.1, 5.1), (3.1, 5.1)])   # plug the gap
    g.add_polygon([(2.9, 3.9), (3.1, 3.9), (3.1, 5.1), (2.9, 5.1)])
    assert g.plan((1.0, 1.0), (5.0, 1.0), 0.3) is None


def test_inflation_is_relaxed_for_a_tight_gap_but_not_ignored():
    g = P.Grid((0.0, 6.0, 0.0, 6.0), res=0.05)
    g.add_polygon([(2.9, 0.0), (3.1, 0.0), (3.1, 2.8), (2.9, 2.8)])
    g.add_polygon([(2.9, 3.4), (3.1, 3.4), (3.1, 6.0), (2.9, 6.0)])   # a 0.6 m gap: too tight for 0.3 m inflation
    path = g.plan((1.0, 3.1), (5.0, 3.1), 0.3)
    assert path is not None and g.last_inflate < 0.3
    g2 = P.Grid((0.0, 6.0, 0.0, 6.0), res=0.05)
    g2.add_polygon([(2.9, 0.0), (3.1, 0.0), (3.1, 2.9), (2.9, 2.9)])
    g2.add_polygon([(2.9, 3.2), (3.1, 3.2), (3.1, 6.0), (2.9, 6.0)])  # 0.3 m: even the relaxed 0.18 m cannot pass
    assert g2.plan((1.0, 3.05), (5.0, 3.05), 0.3) is None


def test_observation_needs_several_frames_and_ignores_known_walls():
    g = box_grid(min_hits=3, novel_m=0.2)
    novel = np.array([[1.0, 1.0], [1.0, 1.05]])
    for _ in range(2):
        g.observe(novel)
    assert not g.occupied()[g.cell(1.0, 1.0)], "two frames are not enough"
    g.observe(novel)
    assert g.occupied()[g.cell(1.0, 1.0)]
    n = g.observe(np.array([[3.0, 3.0]]))                  # in the known wall
    assert n == 0, "points on a mapped wall are not evidence of a new obstacle"
    near = np.array([[3.18, 1.0]])                           # 8 cm off a known wall: estimator drift, not a new wall
    for _ in range(5):
        g.observe(near)
    assert not g.occupied()[g.cell(3.18, 1.0)]
    g.observe(near, force=True)                              # ...unless the aircraft had to stop for it
    assert g.occupied()[g.cell(3.18, 1.0)]


def test_a_new_obstacle_changes_the_path():
    g = box_grid(min_hits=2)
    before = g.plan((1.0, 4.5), (5.0, 4.5), 0.3)
    assert len(before) == 2                                   # straight through the gap
    blob = np.array([[2.0 + 0.05 * i, 4.5 + 0.05 * j] for i in range(8) for j in range(-6, 7)])
    g.observe(blob)
    g.observe(blob)
    after = g.plan((1.0, 4.5), (5.0, 4.5), 0.3)
    assert after is not None and len(after) > 2
    assert min_clearance(g, after) >= 0.3 - 0.06


def test_mission_builds_a_grid_in_its_own_frame_and_rotates_with_the_takeoff_heading():
    prm = C.Params(route=[(2.0, 0.0)], plan=True,
                   plan_map={"bounds": [-1.0, 4.0, -2.0, 2.0], "rects": [[1.0, -0.5, 1.2, 0.5]], "circles": []})
    m = C.Mission(prm)
    out = C.Output("wait")
    heading = math.radians(90)                                # facing east: forward = +e, left = -n
    g = m._build_grid((10.0, 20.0, -0.2, heading), out)
    assert g is not None
    # the wall is 1.0..1.2 m forward (east) of home, +-0.5 m left/right (north/south)
    assert g.prior[g.cell(10.0, 21.1)]
    assert not g.prior[g.cell(10.0, 22.0)]
    assert not g.prior[g.cell(10.0, 20.5)]
    # off-axis takeoff heading: the grid's box is bigger than the rotated flyable area, and the
    # corner cells outside the route-frame bounds must be blocked
    g45 = m._build_grid((10.0, 20.0, -0.2, math.radians(45)), out)
    assert g45.prior[0, 0] and g45.prior[-1, -1] and g45.prior[0, -1] and g45.prior[-1, 0]
    assert not g45.prior[g45.cell(10.0 + 1.0, 20.0 + 1.0)]       # 1.4 m forward of home: inside


def _centre_after(points, last_clearance=math.inf, frames=40):
    m = C.Mission(C.Params(route=[(5.0, 0.0)], center=True))
    m.leg_heading = 0.0
    m.depth_pts = np.array(points, float)
    m.last_clearance = last_clearance
    v = C.Vehicle(pos=(0.0, 0.0, -1.2), heading=0.0, pos_ok=True)
    for t in range(frames):
        m.depth_t = 1.0 + t * 0.1
        m._center_update(v)
    return m


def test_centering_balances_two_seen_walls():
    xs = np.linspace(0.4, 1.3, 12)
    # (forward, right, down): left wall 0.4 m away, right wall 0.8 m away -> move right (positive)
    m = _centre_after([[x, -0.4, 0.0] for x in xs] + [[x, 0.8, 0.0] for x in xs])
    assert 0.1 < m.center_off <= m.p.center_max_m + 1e-9
    m = _centre_after([[x, -0.8, 0.0] for x in xs] + [[x, 0.4, 0.0] for x in xs])
    assert -m.p.center_max_m - 1e-9 <= m.center_off < -0.1
    m = _centre_after([[x, -0.5, 0.0] for x in xs] + [[x, 0.5, 0.0] for x in xs])
    assert abs(m.center_off) < 1e-6


def test_centering_never_pushes_toward_a_side_it_cannot_see():
    xs = np.linspace(0.4, 1.3, 12)
    # a wall only on the left: the right is UNKNOWN (maybe a wall end beside the camera), not open
    m = _centre_after([[x, -0.4, 0.0] for x in xs])
    assert m.center_off == 0.0
    # a wall ACROSS the leg is not a side gap either
    m = _centre_after([[1.4, y, 0.0] for y in np.linspace(-0.8, 0.8, 30)], last_clearance=1.4)
    assert abs(m.center_off) < 1e-6


def test_planning_off_changes_nothing():
    m = C.Mission(C.Params(route=[(2.0, 0.0)]))
    assert m.grid is None and not m.p.plan and not m.p.center and not m.p.plan_goal
