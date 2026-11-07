"""Unit tests for dh_core (run with pytest, or `python test_core.py` without pytest)."""
import math, os, sys, time
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from dh_core.course import Grid2D, _edt_sq
from dh_core.timing import time_parameterize, fillet_path
from dh_core.scoring import finished_run_points, unfinished_run_points


def test_edt_matches_bruteforce():
    rng = np.random.default_rng(0)
    occ = rng.random((40, 55)) < 0.03
    occ[0, 0] = True
    d2 = _edt_sq(occ)
    obs = np.argwhere(occ)
    for r in range(0, 40, 7):
        for c in range(0, 55, 9):
            bf = ((obs - [r, c]) ** 2).sum(1).min()
            assert abs(bf - d2[r, c]) < 1e-6


def test_scoring_matches_official_example():
    # fastest 60 s; 90 s finish via Gate B = 48.3; window = 53.3; window+return = 66.7
    assert round(finished_run_points(90, 60, False, False), 1) == 48.3
    assert round(finished_run_points(90, 60, True, False), 1) == 53.3
    assert round(finished_run_points(90, 60, True, True), 1) == 66.7
    assert round(unfinished_run_points(4, 50, 45), 1) == 34.3
    assert abs(unfinished_run_points(2, 40, 30) - 16.25) < 1e-9   # official page rounds to 16.3


def test_time_parameterization_respects_limits():
    path = np.array([[0, 0, 1], [2, 0, 1], [2, 2, 1]], float)
    sm = fillet_path(path, 0.5)
    tr = time_parameterize(sm, v_max=1.0, a_max=1.0, a_lat=1.0)
    assert tr.v.max() <= 1.0 + 1e-9
    acc = np.diff(tr.v ** 2) / (2 * np.maximum(np.diff(tr.s), 1e-9))
    assert acc.max() <= 1.0 + 1e-6 and acc.min() >= -1.0 - 1e-6
    assert tr.v[0] == 0 and tr.v[-1] == 0


def test_map_alignment_roundtrip_and_correction():
    from dh_core.tracker import MapAlignment
    a = MapAlignment()
    a.init_from_pose([1.0, 2.0, 0.0], 0.3, [-2.5, 0.9, 0.0], math.pi / 2)
    pm, ym = a.to_map([1.0, 2.0, 0.0], 0.3)
    assert np.allclose(pm, [-2.5, 0.9, 0.0]) and abs(ym - math.pi / 2) < 1e-9
    po, yo = a.to_odom(pm, ym)
    assert np.allclose(po, [1.0, 2.0, 0.0]) and abs(yo - 0.3) < 1e-9
    assert abs(np.linalg.norm(a.vec_to_odom([1.0, 0.0, 0.0])) - 1.0) < 1e-9
    # a pure position correction moves the estimate by gain*delta
    a.correct((0.2, -0.1), 0.0, pm[:2], gain=0.5)
    pm2, _ = a.to_map([1.0, 2.0, 0.0], 0.3)
    assert np.allclose(pm2[:2] - pm[:2], [0.1, -0.05])
    # a yaw correction about the drone does not move the drone's own estimate
    a.correct((0.0, 0.0), 0.2, pm2[:2], gain=1.0)
    pm3, ym3 = a.to_map([1.0, 2.0, 0.0], 0.3)
    assert np.allclose(pm3, pm2) and abs(ym3 - (math.pi / 2 + 0.2)) < 1e-9


def test_follower_tracks_straight_line():
    from dh_core.tracker import FollowerParams, TrajectoryFollower
    path = np.array([[0.0, 0.0, 1.0], [3.0, 0.0, 1.0]])
    tr = time_parameterize(fillet_path(path, 0.0), 0.8, 1.0, 1.0)
    f = TrajectoryFollower(tr, FollowerParams())
    p, v, dt = np.array([0.0, 0.0, 1.0]), np.zeros(3), 0.02
    for _ in range(int(3 * tr.duration / dt)):
        cmd, _, _, _ = f.step(p, 0.0, dt)
        v += (cmd - v) / 0.3 * dt
        p = p + v * dt
        if f.done:
            break
    assert f.done and np.linalg.norm(p[:2] - [3.0, 0.0]) < 0.1


def test_matcher_recovers_offset_in_box_room():
    from dh_core.localize import DistanceFieldMatcher, raycast_scan
    g = Grid2D.empty(np.array([-2.0, -2.0]), np.array([2.0, 2.0]), 0.025)
    g.occ[:4, :] = True; g.occ[-4:, :] = True
    g.occ[:, :4] = True; g.occ[:, -4:] = True
    g.occ[60:100, 100:104] = True          # an interior wall breaks the symmetry
    m = DistanceFieldMatcher(g)
    rng = np.random.default_rng(0)
    true = np.array([0.3, -0.2, 0.4])
    scan = raycast_scan(g, true, fov_deg=360, n_beams=180, noise=0.0, rng=rng)
    res = m.match(scan, true + np.array([0.08, -0.06, 0.0]))
    assert res.ok and np.linalg.norm(res.pose[:2] - true[:2]) < 0.03, res


def _body_to_optical(xb, yb, zb, pitch, cam_x, cam_z):
    x, z = xb - cam_x, zb - cam_z
    c, s = math.cos(pitch), math.sin(pitch)
    xc, zc = c * x - s * z, s * x + c * z
    return -yb, -zc, xc          # optical X, Y, Z


def test_stereo_scan_geometry_on_a_wall():
    from dh_core.camera_geom import disparity_to_body, optical_to_body, points_to_scan
    f, B, cx, cy, W, H = 300.0, 0.075, 256.0, 160.0, 512, 320
    pitch, cam_x, cam_z, D = math.radians(20), 0.10, -0.02, 2.0
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    xn, yn = (u - cx) / f, (v - cy) / f
    dx, dy, dz = optical_to_body(xn, yn, np.ones_like(xn), pitch, 0.0, 0.0)   # ray directions in body
    t = (D - cam_x) / dx                          # hit the wall plane x_body = D
    Z = t                                         # optical depth = scale of the (xn, yn, 1) ray
    disp = np.where(dx > 0, f * B / Z, 0.0)
    xb, yb, zb = disparity_to_body(disp, f, B, cx, cy, pitch, cam_x, cam_z)
    assert np.allclose(xb, D, atol=1e-6)
    ranges, amin, inc = points_to_scan(xb, yb, zb, cam_z, 0.15, 2 * math.atan2(cx, f), 41, 5.0)
    a = amin + np.arange(41) * inc
    ok = np.isfinite(ranges)
    assert ok.sum() > 30
    assert np.allclose(ranges[ok], D / np.cos(a[ok]), rtol=0.03)


def test_floor_line_projection_and_fit():
    from dh_core.camera_geom import rays_to_floor, fit_line
    pitch, cam_x, cam_z, h = math.radians(20), 0.10, -0.02, 0.85
    xs = np.linspace(0.9, 2.5, 60)
    for lateral, heading in ((0.20, 0.0), (-0.15, 0.2)):
        yb = lateral + np.tan(heading) * xs
        X, Y, Z = _body_to_optical(xs, yb, np.full_like(xs, -h), pitch, cam_x, cam_z)
        Xf, Yf, ok = rays_to_floor(X / Z, Y / Z, pitch, h, cam_x, cam_z)
        assert ok.all() and np.allclose(Xf, xs) and np.allclose(Yf, yb)
        lat, hd, res = fit_line(Xf, Yf)
        assert abs(lat - lateral) < 1e-6 and abs(hd - heading) < 1e-6 and res < 1e-6


def test_stereo_rectify_from_extrinsics():
    """TF-path rectification: a slightly rotated pair becomes row-aligned with disparity = f*B/Z."""
    try:
        import cv2
    except ImportError:
        print("  (skipped: OpenCV not installed)")
        return
    from dh_core.camera_geom import quat_to_rot, stereo_rectify, camera_info_has_stereo
    K = np.array([[560.0, 0, 512], [0, 560.0, 320], [0, 0, 1]])
    D = np.zeros(5)
    # right camera yawed 1 deg and rolled 0.5 deg relative to the left; 7.5 cm baseline
    a, b = math.radians(1.0) / 2, math.radians(0.5) / 2
    q_yaw = (0.0, math.sin(a), 0.0, math.cos(a)); q_roll = (0.0, 0.0, math.sin(b), math.cos(b))
    R = quat_to_rot(*q_yaw) @ quat_to_rot(*q_roll)
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)
    T = np.array([-0.075, 0.002, 0.0])
    assert not camera_info_has_stereo(np.eye(3), np.c_[K, np.zeros(3)])
    R1, R2, P1, P2, B = stereo_rectify(K, D, K, D, (1024, 640), R, T)
    assert abs(B - np.linalg.norm(T)) < 1e-3
    rng = np.random.default_rng(1)
    pl = np.c_[rng.uniform(-1, 1, 50), rng.uniform(-0.5, 0.5, 50), rng.uniform(0.6, 4.0, 50)]
    pr = pl @ R.T + T
    raw_l = (pl @ K.T)[:, :2] / pl[:, 2:3]
    raw_r = (pr @ K.T)[:, :2] / pr[:, 2:3]
    ul = cv2.undistortPoints(raw_l.reshape(-1, 1, 2), K, D, R=R1, P=P1).reshape(-1, 2)
    ur = cv2.undistortPoints(raw_r.reshape(-1, 1, 2), K, D, R=R2, P=P2).reshape(-1, 2)
    assert np.max(np.abs(ul[:, 1] - ur[:, 1])) < 0.05          # rows aligned to < 0.05 px
    z_rect = (pl @ R1.T)[:, 2]
    disp = ul[:, 0] - ur[:, 0]
    assert np.max(np.abs(disp - P1[0, 0] * B / z_rect)) < 0.05   # disparity = f B / Z


def test_clothoid_corner_is_tangent_and_curvature_continuous():
    """Clothoid fillet: starts/ends tangent to the segments, curvature 0 -> max -> 0 with no jumps,
    apex radius <= r_max, and max_tangent keeps the requested straight run."""
    path = np.array([[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [4.0, 2.0]])
    sm, kap = fillet_path(path, 0.5, transition="clothoid", ds=0.01, return_kappa=True)
    assert np.allclose(sm[0], path[0]) and np.allclose(sm[-1], path[-1])
    assert np.max(np.abs(np.diff(kap))) < 0.2, np.max(np.abs(np.diff(kap)))   # no curvature step (arc would jump 2.0)
    assert 1.9 < kap.max() < 2.1                                                # apex radius r_max = 0.5
    hd = np.unwrap(np.arctan2(np.diff(sm[:, 1]), np.diff(sm[:, 0])))
    assert np.max(np.abs(np.diff(hd))) < 0.05                                   # heading is continuous
    _, kap_arc = fillet_path(path, 0.5, transition="arc", ds=0.01, return_kappa=True)
    assert np.max(np.abs(np.diff(kap_arc))) > 1.9                               # the arc version does jump
    # max_tangent: at most 0.3 m of curve on each side of vertex 1
    sm2, kap2 = fillet_path(path, 0.5, transition="clothoid", ds=0.01, max_tangent=[None, 0.3, None, None],
                            return_kappa=True)
    curved = sm2[kap2 > 1e-6]
    near1 = curved[np.linalg.norm(curved - path[1], axis=1) < 1.0]
    assert np.max(np.linalg.norm(near1 - path[1], axis=1)) <= 0.3 + 0.02


def test_combined_acceleration_limit():
    """With a_total the speed profile keeps sqrt(a_along^2 + a_lat^2) <= a_total (exact curvature)."""
    path = np.array([[0.0, 0.0], [3.0, 0.0], [3.0, 3.0], [0.0, 3.0]])
    sm, kap = fillet_path(path, 0.6, transition="clothoid", ds=0.01, return_kappa=True)
    tr = time_parameterize(sm, v_max=1.2, a_max=1.5, a_lat=1.5, kappa=kap, a_total=1.5)
    k = np.abs(kap[np.concatenate([[True], np.linalg.norm(np.diff(sm, axis=0), axis=1) > 1e-6])])
    v, s = tr.v, tr.s
    a_al = np.diff(v ** 2) / (2 * np.maximum(np.diff(s), 1e-9))
    a_lat = (0.5 * (v[1:] + v[:-1])) ** 2 * np.maximum(k[1:], k[:-1])
    assert np.max(np.hypot(a_al, a_lat)) <= 1.5 * 1.03, np.max(np.hypot(a_al, a_lat))
    tr_sep = time_parameterize(sm, v_max=1.2, a_max=1.5, a_lat=1.5, kappa=kap)
    assert tr.duration >= tr_sep.duration                                       # the joint limit can only be slower


def test_safe_flight_corridor_is_safe_and_trajectory_stays_inside():
    """SFC (Liu et al. RA-L 2017): shrunk polygons keep >= r from every obstacle point and contain the path;
    the min-jerk QP trajectory stays inside the corridor and is rest-to-rest."""
    from dh_core.planner_sfc import SFCParams, build_corridor, obstacle_points, plan_sfc
    g = Grid2D.empty((0, 0), (4, 3), 0.05)
    g.occ[0, :] = g.occ[-1, :] = True; g.occ[:, 0] = g.occ[:, -1] = True
    g.occ[:36, 38:42] = True                                   # wall from the south, gap at the north
    path = np.array([[0.5, 0.5], [1.2, 2.3], [2.8, 2.3], [3.5, 0.5]])
    prm = SFCParams(robot_r=0.25, time_iters=6)
    obs = obstacle_points(g.occ, g.origin, g.res)
    polys, ok = build_corridor(path, obs, prm)
    assert ok
    rng = np.random.default_rng(0)
    for (A, b), p1, p2 in zip(polys, path[:-1], path[1:]):
        for f in np.linspace(0, 1, 11):
            assert np.all(A @ (p1 + f * (p2 - p1)) <= b + 1e-9)      # the segment is inside
        pts = rng.uniform((0, 0), (4, 3), (4000, 2))
        inside = pts[np.all(pts @ A.T <= b[None], axis=1)]
        if len(inside):
            dmin = np.min(np.linalg.norm(inside[:, None, :] - obs[None, :, :], axis=2))
            assert dmin >= prm.robot_r - 1e-6, dmin                  # every corridor point is >= r from obstacles
    tr, polys, st = plan_sfc(path, g.occ, g.origin, g.res, prm)
    assert st["corridor_violation_m"] < 0.01, st["corridor_violation_m"]
    assert np.allclose(tr.eval(0.0)[0], path[0], atol=1e-3) and np.allclose(tr.eval(tr.duration)[0], path[-1], atol=1e-3)
    assert np.linalg.norm(tr.eval(0.0, 1)) < 1e-2 and np.linalg.norm(tr.eval(tr.duration, 1)) < 1e-2
    assert tr.piece_max(1).max() <= prm.v_max * 1.01 and tr.piece_max(2).max() <= prm.a_max * 1.03


def test_lqmt_heuristic_is_the_relaxed_optimum():
    """MPL heuristic h2 (Liu et al. 2017, Prop. 3, free final velocity) equals the brute-force minimum."""
    from dh_core.planner_mpl import lqmt_free_end_cost
    rng = np.random.default_rng(3)
    for _ in range(300):
        d = rng.uniform(-2, 2, 2); v0 = rng.uniform(-0.8, 0.8, 2); tb = np.max(np.abs(d)) / 0.85
        Ts = np.linspace(max(tb, 1e-3), 30, 20000)
        brute = np.min(3 * np.sum((d[None] - np.outer(Ts, v0)) ** 2, 1) / Ts ** 3 + 4 * Ts)
        h = lqmt_free_end_cost(d, v0, 4.0, tb)
        assert brute * (1 - 1e-3) <= h <= brute + 1e-6


def test_pose_history_interpolation_and_rejection():
    from dh_core.health import PoseHistory, stamp_to_sec
    h = PoseHistory(span_s=2.0, max_gap_s=0.25)
    for t in (10.2, 10.0, 10.1, 10.3):                  # out-of-order inserts
        h.add(t, (t, 0.0, 0.0, math.pi - 0.01 if t < 10.15 else -math.pi + 0.01))
    p = h.at(10.05)
    assert abs(p[0] - 10.05) < 1e-9
    p = h.at(10.15)                                      # yaw interpolates across +-pi, not through 0
    assert abs(abs(p[3]) - math.pi) < 0.02, p[3]
    assert h.at(9.9) is None and h.at(10.5) is None and h.at(0.0) is None   # stale / future / unstamped
    assert h.at(10.31, future_tol=0.02) is not None
    h.add(11.0, (11.0, 0, 0, 0))                          # 0.7 s gap
    assert h.at(10.6) is None
    assert not h.add(0.0, (0, 0, 0, 0))
    import types
    assert stamp_to_sec(types.SimpleNamespace(sec=5, nanosec=250000000)) == 5.25
    assert stamp_to_sec(None) == 0.0


def test_forward_clearance_monitor_states():
    from dh_core.health import ClearanceParams, ForwardClearanceMonitor
    m = ForwardClearanceMonitor(ClearanceParams(min_clearance=0.45, timeout_s=0.5, invalid_scale=0.35,
                                                invalid_grace_s=1.5, invalid_floor_scale=0.2))
    assert m.speed_scale(0.0) == (0.0, "MISSING")        # never received: not clear
    m.update(1.0, 1.0, math.inf)
    assert m.speed_scale(1.1) == (1.0, "CLEAR")          # valid measurement, nothing ahead
    assert m.speed_scale(1.7)[1] == "STALE" and m.speed_scale(1.7)[0] == 0.0
    m.update(2.0, 2.0, math.nan)
    assert m.speed_scale(2.1) == (0.35, "INVALID")
    assert m.speed_scale(2.1, expected_blind=True)[0] == 1.0     # mapped dark zone: blind by design
    m.update(3.4, 3.4, math.nan)
    assert m.speed_scale(3.6) == (0.2, "INVALID")        # past the grace period: creep, never full speed
    m.update(4.0, 4.0, 0.6)                              # unexpected obstacle (map says 3 m)
    s, st = m.speed_scale(4.1, expected_range=3.0)
    assert st == "OBSTACLE" and abs(s - (0.6 - 0.45) / 0.45) < 1e-9
    assert m.speed_scale(4.1, expected_range=0.65) == (1.0, "MAPPED_WALL")   # the planned wall at a hairpin
    m.update(4.2, 4.2, 0.30)
    assert m.speed_scale(4.3, expected_range=0.32)[0] == 0.0                 # mapped but too close: stop
    m.update(4.1, 4.5, math.inf)                         # out-of-order (older stamp): ignored
    assert m.range == 0.30
    m.update(0.0, 5.0, math.inf)
    assert m.state(5.1) == "STALE"                       # unstamped reading is never trusted


def test_localization_health_modes():
    from dh_core.health import LocHealthParams, LocalizationHealth
    h = LocalizationHealth(LocHealthParams())
    assert h.evaluate(0.0)[0] == "HOVER"                 # no odometry yet
    h.on_odom(1.0, 1.0, (0, 0, 1))
    assert h.evaluate(1.1)[0] == "OK"
    assert h.evaluate(1.6)[0] == "HOVER" and h.evaluate(3.5)[0] == "ABORT"
    h.on_odom(4.0, 4.0, (0, 0, 1))
    h.on_odom(4.033, 4.033, (0.6, 0, 1))                 # 0.6 m in 33 ms: relocalization jump
    assert h.evaluate(4.1)[0] == "HOVER"
    h.on_odom(4.65, 4.65, (0.6, 0, 1))
    assert h.evaluate(4.7)[0] == "OK"                    # pause over, odometry fresh
    h.on_odom(4.8, 4.8, (0.6, 0, 1))
    h.on_map_fix(4.8, 0.5)
    m, s, _ = h.evaluate(4.9)
    assert m == "DEGRADED" and s < 1.0
    h.on_odom(6.0, 6.0, (0.6, 0, 1))
    h.on_vo_status(6.0, 2)                                # tracking lost outside a dark zone
    assert h.evaluate(6.1)[0] == "HOVER"
    h.set_in_dark(6.0, True)                              # ... inside the mapped tube it is expected
    assert h.evaluate(6.1)[0] == "OK"
    h.on_odom(10.5, 10.5, (0.6, 0, 1))
    assert h.evaluate(10.5)[0] == "HOVER"                 # dead-reckoning too long in the dark
    h.set_in_dark(10.5, False)
    h.on_odom(10.6, 10.6, (0.6, 0, 1))
    assert h.evaluate(10.6)[0] == "ABORT" or h.evaluate(10.6)[0] == "HOVER"
    h.on_vo_status(10.6, 1)
    h.on_odom(10.7, 10.7, (0.6, 0, 1), pos_std=0.4)
    assert h.evaluate(10.7)[0] == "DEGRADED"


def test_scan_validity_distinguishes_clear_from_unknown():
    from dh_core.camera_geom import scan_validity
    a = np.linspace(-0.6, 0.6, 25)
    half = math.radians(15)
    r, f = scan_validity(np.full(25, np.inf), np.full(25, 50), a, half)
    assert f == math.inf and np.all(np.isinf(r))        # lots of stereo data, nothing within range: clear
    r, f = scan_validity(np.full(25, np.inf), np.zeros(25, int), a, half)
    assert math.isnan(f) and np.all(np.isnan(r))        # no stereo data at all: unknown
    cnt = np.full(25, 50); cnt[np.abs(a) <= half] = 0; cnt[12] = 50
    r, f = scan_validity(np.full(25, np.inf), cnt, a, half)
    assert math.isnan(f)                                 # poor forward coverage: cannot vouch for clear
    rr = np.full(25, np.inf); rr[12] = 0.8
    r, f = scan_validity(rr, cnt, a, half)
    assert f == 0.8                                      # an obstacle is reported even with poor coverage


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            t = time.time(); fn(); print("PASS %s (%.2fs)" % (name, time.time() - t))
