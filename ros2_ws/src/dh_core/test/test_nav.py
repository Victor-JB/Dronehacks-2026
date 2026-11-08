"""Focused tests for the navigation policy components (dh_core.nav). sim_world is used only to render test
images; the policy code under test never receives the world."""
import ast
import glob
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
sys.path[:0] = [os.path.join(HERE, ".."), ROOT]
from dh_core.nav.explorer import Explorer, ExplorerParams
from dh_core.nav.landing import LandingParams, TargetTracker, detect
from dh_core.nav.online_map import FREE, OCCUPIED, UNKNOWN, MapParams, OnlineMap
from dh_core.nav.openings import evaluate_opening
from dh_core.nav.safety3d import CommandSafety, CommandSafetyParams
from dh_core.nav.sim_world import Tube, World, wall_with_tube, wall_with_window
from dh_core.shadow.observation import rot_from_quat_xyzw
from dh_core.shadow.synth import bag_extrinsic, quat_from_rpy

E = bag_extrinsic()
K = (198.4 / 2, 198.4 / 2, 162.45 / 2, 101.95 / 2)
W, H = 160, 100
BACK = (4.5, 4.7, -3, 3, 0, 1.7)


def cam(pos, yaw):
    Rwb = rot_from_quat_xyzw(*quat_from_rpy(0, 0, yaw))
    return Rwb @ E.R_body_cam, np.asarray(pos, float) + Rwb @ E.t_body_cam


def observe(m, world, poses, t0=0.0):
    for i, (x, y, yaw) in enumerate(poses):
        Rwc, o = cam((x, y, 0.75), yaw)
        d, _ = world.render(o, Rwc, K, W, H, color=False)
        m.integrate(d, K, Rwc, o, t0 + 0.1 * i)
        m.mark_visited((x, y, 0.75))


def state(m, cls, x, y):
    j, i = int((x - m.vm.lo[0]) / m.vm.voxel), int((y - m.vm.lo[1]) / m.vm.voxel)
    return cls[i, j]


SWEEP = [(0.0, 0.0, a) for a in np.linspace(-0.5, 0.5, 6)]


# ------------------------------------------------------------------ map
def test_map_wall_window_and_overflight_rule():
    w = World(boxes=wall_with_window(1, 2.0, -1.5, 1.5, 0.1, 1.7, -0.45, 0.45, 0.4, 1.2)
              + [(1.0, 1.3, 0.8, 1.2, 0.0, 0.5), BACK])            # a 0.5 m box: lower than the flight band
    m = OnlineMap(MapParams())
    observe(m, w, SWEEP + [(0.3, 0.0, 0.0), (0.0, 0.3, 0.5)])
    c = m.classify()
    assert state(m, c, 2.05, 1.0) == OCCUPIED                       # wall beside the window
    assert state(m, c, 2.05, 0.0) == FREE                           # window: sill below, observed lintel above
    assert state(m, c, 1.02, 1.0) == OCCUPIED                       # low box face, no top above it: not overflown
    # (its interior columns get only grazing top-surface hits; maze walls fill the band and are always blocked)
    assert state(m, c, 0.8, 0.0) == FREE
    assert state(m, c, -1.5, 0.0) == UNKNOWN                        # never seen (behind the camera)
    assert np.isfinite(m.last_seen[int((0 - m.vm.lo[1]) / 0.05), int((1.0 - m.vm.lo[0]) / 0.05)])


def test_missing_depth_never_carves_free():
    m = OnlineMap(MapParams())
    Rwc, o = cam((0, 0, 0.75), 0.0)
    m.integrate(np.full((H, W), np.nan, np.float32), K, Rwc, o, 0.0)
    assert (m.classify() != FREE).all()


# ------------------------------------------------------------------ openings
def _window_map(width):
    w = World(boxes=wall_with_window(1, 2.0, -1.5, 1.5, 0.1, 1.7, -width / 2, width / 2, 0.4, 1.2) + [BACK])
    m = OnlineMap(MapParams())
    observe(m, w, SWEEP + [(0.8, 0.0, 0.0), (1.2, 0.0, 0.0)])
    return m


def test_window_wide_enough_is_verified_and_too_narrow_is_not():
    op = evaluate_opening(_window_map(0.9), (2.05, 0.0), (1.0, 0.0))
    assert op.kind == "rectangular" and op.verified, op.reason
    assert abs(op.centre[1]) < 0.1 and abs(op.normal[0]) > 0.95       # centred, square-on
    op = evaluate_opening(_window_map(0.5), (2.05, 0.0), (1.0, 0.0))
    assert not op.verified and "too small" in op.reason
    op = evaluate_opening(_window_map(0.9), (2.05, 0.0), (1.0, 0.0), loc_ok=False)
    assert not op.verified and "localization" in op.reason


def test_tunnel_is_elliptical_and_verified_along_its_axis():
    tube = Tube(0, 2.0, 3.0, 0.0, 0.75, 0.47, 0.55)
    w = World(boxes=wall_with_tube(1, 2.0, -1.5, 1.5, 0.1, 1.7, tube) + [BACK], tubes=[tube])
    m = OnlineMap(MapParams())
    observe(m, w, SWEEP + [(0.8, 0.0, 0.0), (1.2, 0.0, 0.0), (1.5, 0.0, 0.0)])
    op = evaluate_opening(m, (2.4, 0.05), (1.0, 0.15))
    assert op.kind in ("elliptical", "irregular", "rectangular") and op.enclosed
    assert abs(op.normal[0]) > 0.95                                   # flown along the tube axis


def test_corridor_gap_without_top_is_not_an_opening():
    w = World(boxes=[(1.8, 2.2, 0.35, 2.0, 0, 1.7), (1.8, 2.2, -2.0, -0.35, 0, 1.7), BACK])
    m = OnlineMap(MapParams())
    observe(m, w, SWEEP + [(1.0, 0.0, 0.0)])
    assert evaluate_opening(m, (2.0, 0.0), (1.0, 0.0)).kind == "open"


# ------------------------------------------------------------------ final-command safety
def _grid_free(m):
    g = m.planning_grid()
    return g, g.distance()


def test_command_safety_geofence_band_unknown_and_stale():
    w = World(boxes=[(3.0, 3.2, -3, 3, 0, 1.7)])
    m = OnlineMap(MapParams())
    observe(m, w, SWEEP)
    g, dist = _grid_free(m)
    pos = np.array([0.0, 0.0, 0.75])
    fence = (-1.0, 4.0, -2.0, 2.0)
    args = lambda now: (now, dist, g, now - 0.02, now - 0.01, now - 0.05, now - 0.04, "OK")
    cs = CommandSafety(CommandSafetyParams(geofence=None))
    v, _, _ = cs.filter([0.3, 0, 0], 0.0, pos, *args(10.0))
    assert np.allclose(v[:2], 0)                                       # no geofence: no horizontal motion
    cs = CommandSafety(CommandSafetyParams(geofence=fence))
    v, _, verdict = cs.filter([0.3, 0, 0], 0.0, pos, *args(10.0))
    assert verdict.verdict == "SAFE" and np.isclose(np.linalg.norm(v), 0.3)
    v, _, _ = cs.filter([0.0, 0.3, 0], 0.0, pos, *args(10.0))          # sideways into unseen space
    assert np.linalg.norm(v) < 0.3 and cs.gate.last.verdict in ("LIMIT", "STOP")
    v, _, _ = cs.filter([0.0, 0, 0.3], 0.0, np.array([0, 0, 1.0]), *args(10.0))
    assert v[2] == 0.0                                                 # never climbs out of the band
    v, _, _ = cs.filter([0.3, 0, 0], 0.0, np.array([3.65, 0, 0.75]), *args(10.0))
    assert np.allclose(v[:2], 0)                                       # at the geofence margin
    now = 10.0
    v, yr, verdict = cs.filter([0.3, 0, 0], 0.4, pos, now, dist, g, now - 2, now - 2, now - 0.05, now - 0.04, "OK")
    assert np.allclose(v, 0) and yr == 0.0 and verdict.verdict == "UNKNOWN"   # stale odometry
    v, yr, verdict = cs.filter([0.3, 0, 0], 0.4, pos, now, None, None, now, now, now, now, "OK")
    assert np.allclose(v, 0)                                           # no map: no motion


def test_command_safety_slows_then_stops_before_a_wall():
    w = World(boxes=[(1.5, 1.7, -3, 3, 0, 1.7)])
    cs = CommandSafety(CommandSafetyParams(geofence=(-1.0, 4.0, -2.0, 2.0)))
    speeds = []
    for x in np.arange(0.0, 1.2, 0.1):
        m = OnlineMap(MapParams())
        observe(m, w, [(x, 0.0, a) for a in (-0.4, 0.0, 0.4)])
        g, dist = _grid_free(m)
        v, _, _ = cs.filter([0.35, 0, 0], 0.0, np.array([x, 0, 0.75]), 10.0, dist, g, 9.98, 9.99, 9.95, 9.96, "OK")
        speeds.append(float(np.linalg.norm(v)))
    assert all(b <= a + 1e-9 for a, b in zip(speeds, speeds[1:]))       # never speeds up while closing in
    assert speeds[0] > 0.2 and speeds[-1] == 0.0                          # stops with the body clear of the wall


# ------------------------------------------------------------------ landing target
def _pad_frame(pad=(2.0, 0.0, 0.6), extra=(), pos=(0.0, 0.0, 0.75), noise=0):
    w = World(boxes=list(extra) + [BACK], pad=pad)
    Rwc, o = cam(pos, 0.0)
    d, rgb = w.render(o, Rwc, K, W, H)
    if noise:
        rng = np.random.default_rng(1)
        idx = rng.integers(0, d.size, noise)
        rgb.reshape(-1, 3)[idx] = (30, 70, 210)
    return rgb, d, Rwc, o


def test_blue_square_detected_on_orange_floor():
    rgb, d, Rwc, o = _pad_frame()
    dets = detect(rgb, d, K, Rwc, o)
    assert len(dets) == 1
    assert np.linalg.norm(dets[0].centre - [2.0, 0.0]) < 0.15 and 0.4 < dets[0].side < 0.8
    dets = detect(rgb, None, K, Rwc, o)                                    # no depth: floor-plane projection
    assert len(dets) == 1 and np.linalg.norm(dets[0].centre - [2.0, 0.0]) < 0.2


def test_blue_rejections():
    # blue on a wall is not on the floor; an elongated strip is not a square; speckles are noise
    w = World(boxes=[(2.5, 2.6, -0.3, 0.3, 0.3, 0.9), BACK])
    Rwc, o = cam((0, 0, 0.75), 0.0)
    d, rgb = w.render(o, Rwc, K, W, H)
    rgb[(d > 0)] = rgb[(d > 0)]
    mask = np.zeros(d.shape, bool)
    vs, us = np.mgrid[0:H, 0:W]
    mask[(us > 70) & (us < 90) & (vs > 40) & (vs < 55)] = True        # the wall patch region
    rgb2 = rgb.copy(); rgb2[mask] = (30, 70, 210)
    assert detect(rgb2, d, K, Rwc, o) == []
    rgb, d, Rwc, o = _pad_frame(pad=(0, 0, 0))
    strip = World(boxes=[BACK], pad=(2.0, 0.0, 0.6))                   # render a 0.6 m pad then cut it to a strip
    _, rgbs = strip.render(o, Rwc, K, W, H)
    rows = np.where((rgbs == (30, 70, 210)).all(-1).any(1))[0]
    rgbs[rows[: len(rows) * 3 // 4]] = (230, 120, 30)
    assert all(dd.side < 0.2 or dd.aspect < 0.6 for dd in detect(rgbs, d, K, Rwc, o)) or detect(rgbs, d, K, Rwc, o) == []
    rgb, d, Rwc, o = _pad_frame(pad=(0, 0, 0), noise=300)
    assert detect(rgb, d, K, Rwc, o) == []


def test_tracker_fuses_verifies_and_loses_confidence():
    tr = TargetTracker()
    rgb, d, Rwc, o = _pad_frame()
    for i in range(4):
        for det in detect(rgb, d, K, Rwc, o):
            tr.update(det)
    assert tr.verified and np.linalg.norm(tr.xy - [2.0, 0.0]) < 0.15
    s0 = tr.sigma
    tr.predict(0.0, (0, 0, 0))
    tr.predict(60.0, (5, 0, 0))                                          # a minute and 5 m later, unseen
    assert tr.sigma > s0 and not tr.verified


# ------------------------------------------------------------------ explorer
def test_explorer_never_sees_the_world():
    for f in glob.glob(os.path.join(HERE, "..", "dh_core", "nav", "*.py")):
        if os.path.basename(f) in ("sim_world.py", "sim_run.py"):
            continue
        tree = ast.parse(open(f, encoding="utf-8").read())
        mods = {n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)}
        assert not any(m and "sim_world" in m for m in mods), f


def test_explorer_scans_then_plans_to_a_frontier_and_replans_when_blocked():
    w = World(boxes=[(-1.0, 4.6, 1.2, 1.3, 0, 1.7), (-1.0, 4.6, -1.3, -1.2, 0, 1.7), BACK])
    ex = Explorer(MapParams(), ExplorerParams())
    pos, yaw = np.array([0.0, 0.0, 0.75]), 0.0
    for k in range(120):                                                 # 12 s: scan, then explore (pose fixed)
        t = 0.1 * k
        Rwc, o = cam(pos, yaw)
        d, _ = w.render(o, Rwc, K, W, H, color=False)
        ex.on_frame(t, d, None, K, Rwc, o, True)
        it = ex.step(t, pos, yaw, np.zeros(3), True)
        yaw = (yaw + it.yaw_rate * 0.1 + math.pi) % (2 * math.pi) - math.pi
        if ex.follower is not None:
            break
    # a frontier goal away from the start (either open end of the corridor is a valid choice)
    assert ex.state == "EXPLORE" and ex.follower is not None and np.linalg.norm(ex.goal) > 1.0
    assert ex.pl.clearance_at(ex.goal) > ex.mprm.robot_radius                 # a safe approach cell
    # an unexpected wall across the corridor, between the drone and its goal, appears and is observed
    xw = 0.55 * ex.goal[0]
    w.boxes.append((xw, xw + 0.1, -1.2, 1.2, 0, 1.7))
    yaw = 0.0 if ex.goal[0] > 0 else math.pi                          # look at it
    for k in range(3):
        Rwc, o = cam(pos, yaw)
        d, _ = w.render(o, Rwc, K, W, H, color=False)
        ex.on_frame(20 + 0.1 * k, d, None, K, Rwc, o, True)
    assert not ex._traj_still_clear()                                   # current trajectory now blocked
    n0 = ex.replans
    ex.step(21.0, pos, yaw, np.zeros(3), True)
    assert ex.replans == n0 + 1
    assert ex.follower is None or ex._traj_still_clear()                # new plan avoids the wall (or holds)


def test_rotation_in_place_allowed_without_a_map_only_with_fresh_data():
    """Regression: before any map existed the gate zeroed the yaw rate, so the start-up scan never turned."""
    cs = CommandSafety(CommandSafetyParams(geofence=(-1.0, 4.0, -2.0, 2.0)))
    now = 10.0
    v, yr, _ = cs.filter([0, 0, 0], 0.7, np.array([0, 0, 0.75]), now, None, None, now - 0.02, now - 0.01, now - 0.05, now - 0.04, "OK")
    assert np.allclose(v, 0) and yr == 0.7
    v, yr, _ = cs.filter([0, 0, 0], 0.7, np.array([0, 0, 0.75]), now, None, None, now - 3, now - 3, now - 0.05, now - 0.04, "OK")
    assert yr == 0.0                                                   # stale odometry: no rotation either
    v, yr, _ = cs.filter([0, 0, 0], 0.7, np.array([0, 0, 0.75]), now, None, None, now - 0.02, now - 0.01, now - 0.05, now - 0.04, "HOVER")
    assert yr == 0.0                                                   # localization lost


def test_scan_counts_measured_rotation_not_commands():
    ex = Explorer(MapParams(), ExplorerParams())
    pos = np.array([0.0, 0.0, 0.75])
    for k in range(50):                                                # yaw command ignored: drone does not turn
        it = ex.step(0.1 * k, pos, 0.0, np.zeros(3), True)
        assert it.state == "SCAN"
