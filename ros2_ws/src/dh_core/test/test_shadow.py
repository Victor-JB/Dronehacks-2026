"""Milestone 2: passive telemetry + shadow-mode safety (dh_core.shadow). Deterministic, no ROS, no hardware.
Synthetic worlds are procedural boxes; the only real data is a 16 s odometry slice of the 2026-11-07 drone bag."""
import glob
import math
import os
import random
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, ROOT)                                       # refnav (reused depth code)
from refnav import depth as D
from dh_core.shadow.clearance import CorridorParams, corridor_clearance
from dh_core.shadow.guard import check_endpoints, scan_sources
from dh_core.shadow.monitor import ShadowMonitor
from dh_core.shadow.observation import odom_from_msg, state_from_json, intrinsics_from_msg, depth_from_msg
from dh_core.shadow.replay import load_odom_trace, replay, to_jsonl
from dh_core.shadow.streams import COMPARABLE, MISMATCH, UNDETERMINED, ClockDomainMonitor, StreamMonitor
from dh_core.shadow.synth import (BAG_K, CAM_FRAME, Scenario, bag_extrinsic, bag_tree, depth_msg, info_msg, odom_msg,
                                  quat_from_rpy, render_depth, straight)
from dh_core.shadow.transforms import (StaticTransformTree, heading_dir_to_world, level_rotation_enu_flu,
                                       validate_extrinsic, world_dir_to_bearing)

WALL = (3.0, 3.5, -4.0, 4.0, 0.0, 5.0)          # wall face at x = 3.0 m, drone flies at z = 1.0 m
BACK = (-4.0, -3.5, -4.0, 4.0, 0.0, 5.0)        # far wall behind the start (gives returns when facing away)
FAR = (6.0, 6.5, -4.0, 4.0, 0.0, 5.0)           # far background: 'no obstacle' must still return depth
T0 = 1_794_095_811.0
TRACE = os.path.join(HERE, "data", "odom_trace_20261107_vio_divergence.csv")


def run(sc, extr="bag", **cfg):
    mon = ShadowMonitor()
    if extr == "bag":
        mon.set_extrinsic(bag_extrinsic())
    elif extr is not None:
        mon.set_extrinsic(*extr)
    return replay(sc.events(), mon), mon


def rel(reps, t0=T0):
    return [(r["t"] - t0, r) for r in reps]


def after(reps, t, t0=T0):
    return [r for tt, r in rel(reps, t0) if tt >= t]


# ------------------------------------------------------------------ observation adapter
def test_odom_position_heading_and_world_velocity():
    q = quat_from_rpy(0, 0, math.pi / 2)                        # facing north
    o = odom_from_msg(odom_msg(T0, [1, 2, 3], q, [1.0, 0.0, 0.0]), T0 + 0.05)   # 1 m/s forward in the BODY frame
    assert o.valid and np.allclose(o.pos, [1, 2, 3])
    assert math.isclose(o.yaw, math.pi / 2, abs_tol=1e-9)
    assert np.allclose(o.vel_world, [0, 1, 0], atol=1e-9)       # forward = north in ENU
    assert np.allclose(o.vel_body, [1, 0, 0])
    o2 = odom_from_msg(odom_msg(T0, [0, 0, 0], q, [0.0, 1.0, 0.0]), T0, twist_in_child_frame=False)
    assert np.allclose(o2.vel_world, [0, 1, 0]) and np.allclose(o2.vel_body, [1, 0, 0], atol=1e-9)


def test_odom_rejects_bad_quaternion_nonfinite_and_unstamped():
    m = odom_msg(T0, [0, 0, 0], (0, 0, 0, 0.5), [0, 0, 0])
    assert "quaternion_invalid" in odom_from_msg(m, T0).reasons
    m = odom_msg(T0, [math.nan, 0, 0], (0, 0, 0, 1), [0, 0, 0])
    assert not odom_from_msg(m, T0).valid
    m = odom_msg(None, [0, 0, 0], (0, 0, 0, 1), [0, 0, 0])
    assert "unstamped" in odom_from_msg(m, T0).reasons
    assert not odom_from_msg(object(), T0).valid                 # malformed message


def test_odom_covariance():
    assert odom_from_msg(odom_msg(T0, [0, 0, 0], (0, 0, 0, 1), [0, 0, 0], pos_var=0.0), T0).pos_std is None
    assert math.isclose(odom_from_msg(odom_msg(T0, [0, 0, 0], (0, 0, 0, 1), [0, 0, 0], pos_var=0.04), T0).pos_std, 0.2)


def test_state_json_and_intrinsics():
    s, why = state_from_json('{"stamp": 5.0, "armed": true, "nav_state": 14, "clamped": ["min_altitude"]}', 1.0)
    assert s.armed and s.nav_state == 14 and s.clamped == ["min_altitude"] and why == ""
    assert state_from_json("not json", 1.0) == (None, "state_json_invalid")
    assert state_from_json("[1,2]", 1.0)[1] == "state_json_not_object"
    i, _ = intrinsics_from_msg(info_msg(T0))
    assert i.K == BAG_K and (i.width, i.height) == (320, 200)
    bad = info_msg(T0); bad.k = [0.0] * 9
    assert intrinsics_from_msg(bad)[0] is None


def test_depth_decode_reuses_refnav_and_marks_zero_invalid():
    img = np.full((200, 320), 2.0, np.float32); img[:10] = np.nan
    d, _ = depth_from_msg(depth_msg(T0, img), T0)
    assert np.isnan(d.depth_m[:10]).all() and np.allclose(d.depth_m[10:], 2.0)
    bad = depth_msg(T0, img); bad.encoding = "rgb8"
    assert depth_from_msg(bad, T0) == (None, "depth_undecodable")


# ------------------------------------------------------------------ clocks and streams
def test_clock_comparable_like_the_bag():
    c = ClockDomainMonitor()
    for i in range(10):
        c.update(T0 + i / 30, T0 + i / 30 + 0.08 + 0.01 * (i % 3))
    assert c.state == COMPARABLE and math.isclose(c.latency(T0, T0 + 0.1), 0.1, abs_tol=1e-6)


def test_clock_mismatch_zero_based_future_and_step():
    for stamp_of in (lambda t: t - T0 + 12.0,      # zero-based / sim time vs epoch receiver
                     lambda t: t + 5.0):           # producer clock ahead
        c = ClockDomainMonitor()
        for i in range(10):
            c.update(stamp_of(T0 + i / 30), T0 + i / 30)
        assert c.state == MISMATCH and c.latency(1.0, 2.0) is None
    c = ClockDomainMonitor()
    assert c.state == UNDETERMINED and c.latency(T0, T0 + 0.1) is None
    for i in range(10):
        c.update(T0 + i, T0 + i + 0.05)
    c.update(T0 + 10 - 3.0, T0 + 10.05)            # clock steps back 3 s
    assert c.state == MISMATCH and c.steps == 1


def test_stream_rates_gaps_drops_order_from_messages_not_camera_info():
    s = StreamMonitor("depth")
    t = [T0 + i / 15 for i in range(30)]
    del t[10:13]                                    # three frames dropped
    for x in t:
        assert s.on_message(x, x + 0.05)
    assert s.gaps == 1 and s.dropped_est == 3
    assert s.on_message(t[-1] - 0.5, t[-1] + 0.1) is False and s.out_of_order == 1
    assert s.on_message(t[-1], t[-1] + 0.1) is False and s.duplicates == 1
    assert s.on_message(None, t[-1] + 0.2) is False and s.unstamped == 1
    expect = (len(t) - 1) / (t[-1] - t[0])                     # measured from depth itself, drops included
    assert math.isclose(s.summary()["rate_stamp_hz"], expect, rel_tol=1e-6) and expect < 14.0
    info = StreamMonitor("camera_info")                        # camera_info at 30 Hz says nothing about depth
    for i in range(60):
        info.on_message(T0 + i / 30, T0 + i / 30)
    assert abs(info.summary()["rate_rx_hz"] - 30) < 0.5 and abs(s.summary()["rate_rx_hz"] - 30) > 5


# ------------------------------------------------------------------ transforms
def test_bag_tf_chain_gives_forward_camera():
    R, t = bag_tree().lookup("base_link", CAM_FRAME)
    assert np.allclose(t, [0.1053, 0, 0.0304])
    assert np.allclose(R @ [0, 0, 1], [1, 0, 0], atol=1e-9)    # optical z = body forward
    assert np.allclose(R @ [1, 0, 0], [0, -1, 0], atol=1e-9)   # optical x = body right
    assert StaticTransformTree().lookup("base_link", CAM_FRAME) is None


def test_validate_extrinsic_rejects_bad_transforms():
    e = bag_extrinsic()
    assert validate_extrinsic(CAM_FRAME, e.R_body_cam, e.t_body_cam)[1] == ""
    back = np.diag([-1, 1, -1]) @ e.R_body_cam                 # camera looking backwards
    assert validate_extrinsic(CAM_FRAME, back, e.t_body_cam) == (None, "camera_not_forward")
    assert validate_extrinsic(CAM_FRAME, e.R_body_cam * 2, e.t_body_cam)[1] == "tf_not_rotation"
    assert validate_extrinsic(CAM_FRAME, e.R_body_cam, [5, 0, 0])[1] == "tf_offset_implausible"
    assert validate_extrinsic(CAM_FRAME, e.R_body_cam, [math.nan, 0, 0])[1] == "tf_nonfinite"


def _quat_wxyz_from_mat(R):
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    return (w, (R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w))


def test_level_rotation_matches_refnav_ned_convention():
    """Our ENU/FLU level rotation equals refnav.depth.level_rotation fed the equivalent PX4 NED/FRD attitude."""
    M = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1.0]]); F = np.diag([1.0, -1, -1])
    rng = random.Random(1)
    for _ in range(20):
        r, p, y = rng.uniform(-0.4, 0.4), rng.uniform(-0.4, 0.4), rng.uniform(-3, 3)
        q = quat_from_rpy(r, p, y)
        R_ned_frd = M @ D.quat_to_mat(q[3], q[0], q[1], q[2]) @ F
        assert np.allclose(level_rotation_enu_flu(q), D.level_rotation(_quat_wxyz_from_mat(R_ned_frd)), atol=1e-9)


def test_bearing_round_trip():
    for yaw in (0.0, 1.0, -2.5):
        for b in (-0.5, 0.0, 0.7):
            assert math.isclose(world_dir_to_bearing(yaw, heading_dir_to_world(yaw, b)), b, abs_tol=1e-9)


# ------------------------------------------------------------------ clearance on rendered depth
def test_corridor_clearance_matches_geometry_and_tilt():
    e = bag_extrinsic()
    for pitch in (0.0, 0.15, -0.15):
        q = quat_from_rpy(0.0, pitch, 0.0)
        img = render_depth([0, 0, 1.0], q, [WALL], e)
        res = corridor_clearance(img, BAG_K, e.R_body_cam, e.t_body_cam, level_rotation_enu_flu(q))
        assert abs(res.free_distance - 3.0) < 0.06, (pitch, res)     # wall 3.0 m ahead, levelled
        assert res.valid_fraction > 0.9


def test_corridor_clearance_no_returns_is_unknown_not_clear():
    e = bag_extrinsic(); q = quat_from_rpy(0, 0, 0)
    img = np.full((200, 320), np.nan, np.float32)
    res = corridor_clearance(img, BAG_K, e.R_body_cam, e.t_body_cam, level_rotation_enu_flu(q))
    assert res.free_distance == math.inf and res.valid_fraction == 0.0


# ------------------------------------------------------------------ shadow scenarios
def verdicts(reps):
    return [r["verdict"] for r in reps]


def test_valid_depth_healthy_odometry_far_wall_is_safe():
    reps, mon = run(Scenario(lambda t: (np.array([-1.0, 0, 1.0]), 0.0, np.zeros(3)), boxes=[WALL], duration=3.0))
    late = after(reps, 1.0)
    assert all(r["verdict"] == "SAFE" and r["reason"] == "OK" for r in late)
    assert all(abs(r["free_distance"] - 3.0) < 0.1 for r in late)          # capped at reliable range (wall 4 m)
    assert late[-1]["streams"]["odom"]["clock"] == COMPARABLE


def test_approaching_wall_safe_then_limit_then_stop_and_flags_excess_motion():
    reps, _ = run(Scenario(straight(v=0.6), boxes=[WALL], duration=4.8))   # ends just before contact
    seq = [r["verdict"] for r in after(reps, 0.6)]
    first = {v: seq.index(v) for v in ("SAFE", "LIMIT", "STOP")}
    assert first["SAFE"] < first["LIMIT"] < first["STOP"]
    lim = [r for r in after(reps, 0.6) if r["verdict"] in ("LIMIT", "STOP")]
    assert any(r["current_motion_exceeds_limit"] for r in lim)
    ms = [r["max_speed"] for r in after(reps, 0.6)]
    assert all(b <= a + 1e-9 for a, b in zip(ms, ms[1:]))                  # never relaxes while closing in
    # the free distance accounts for motion since capture: never more than the true remaining gap
    for tt, r in rel(reps):
        if r["free_distance"] is not None and tt > 0.6:
            assert r["free_distance"] <= 3.0 - 0.6 * tt + 0.02


def test_moving_away_facing_away_is_safe_backing_away_blind_is_unknown():
    reps, _ = run(Scenario(straight(v=0.5, yaw=math.pi, start=(1.5, 0, 1.0)), boxes=[WALL, BACK], duration=3.0))
    assert all(r["verdict"] == "SAFE" for r in after(reps, 1.0))           # open space ahead
    back = lambda t: (np.array([1.5 - 0.5 * t, 0, 1.0]), 0.0, np.array([-0.5, 0, 0]))   # reversing, camera on wall
    reps, _ = run(Scenario(back, boxes=[WALL], duration=3.0))
    late = after(reps, 1.0)
    assert all(r["verdict"] == "UNKNOWN" and r["max_speed"] == 0.0 for r in late)      # travel direction unseen
    assert all(r["reason"] == "DEPTH_COVERAGE_LOW" for r in late)


def test_depth_dropout_and_invalid_pixels_never_safe():
    reps, _ = run(Scenario(straight(v=0.3), boxes=[WALL], duration=4.0, depth_gaps=[(1.5, 3.0)]))
    assert all(r["verdict"] == "UNKNOWN" and r["reason"] == "DEPTH_STALE" for tt, r in rel(reps) if 2.0 <= tt <= 3.0)
    assert after(reps, 3.4)[0]["verdict"] in ("SAFE", "LIMIT")                 # recovers when frames return

    def holes(t, img):                                                          # central region returns nothing
        img[60:140, 100:220] = np.nan
        return img
    reps, _ = run(Scenario(straight(v=0.3), boxes=[WALL], duration=3.0, depth_corrupt=holes))
    assert all(r["verdict"] == "UNKNOWN" and r["reason"] == "DEPTH_COVERAGE_LOW" for r in after(reps, 0.6))

    def all_invalid(t, img):
        return np.full_like(img, np.nan) if t >= 1.5 else img
    reps, _ = run(Scenario(straight(v=0.3), boxes=[WALL], duration=3.0, depth_corrupt=all_invalid))
    assert all(r["max_speed"] == 0.0 for r in after(reps, 1.7))


def test_half_blind_corridor_is_unknown_even_with_floor_returns():
    """Textureless wall: no returns in the UPPER half of the corridor while the floor fills the lower half."""
    def upper_blind(t, img):
        img[:100, :] = np.nan
        return img
    reps, _ = run(Scenario(straight(v=0.3), boxes=[WALL], duration=3.0, depth_corrupt=upper_blind))
    assert all(r["verdict"] == "UNKNOWN" and r["reason"] == "DEPTH_COVERAGE_LOW" for r in after(reps, 0.6))


def test_stale_odometry_stops():
    reps, _ = run(Scenario(straight(v=0.3), boxes=[WALL], duration=4.0, odom_gaps=[(1.5, 3.0)]))
    mid = [r for tt, r in rel(reps) if 2.3 <= tt <= 3.0]
    assert mid and all(r["max_speed"] == 0.0 and r["verdict"] in ("STOP", "UNKNOWN") for r in mid)
    assert any(r["loc_mode"] in ("HOVER", "ABORT") for r in mid)


def test_reordered_depth_never_replaces_newer_frame():
    reps, _ = run(Scenario(straight(v=0.6), boxes=[WALL], duration=4.0, reorder_depth=True))
    assert reps[-1]["streams"]["depth"]["out_of_order"] > 5
    caps = [r["free_distance_at_capture"] for r in reps if r["free_distance_at_capture"] is not None]
    assert all(b <= a + 1e-6 for a, b in zip(caps, caps[1:]))                # closing in: never jumps back out


def test_delayed_depth_goes_stale():
    reps, _ = run(Scenario(straight(v=0.3), boxes=[WALL], duration=4.0, depth_extra_delay=[(1.5, 3.0, 0.5)]))
    mid = [r for tt, r in rel(reps) if 2.2 <= tt <= 3.0]
    assert mid and all(r["verdict"] == "UNKNOWN" for r in mid)
    assert reps[-1]["streams"]["depth"]["latency_max_s"] > 0.5


def test_missing_or_wrong_transform_is_unknown():
    sc = Scenario(straight(v=0.3), boxes=[WALL], duration=2.0)
    reps, _ = run(sc, extr=None)
    assert all(r["verdict"] == "UNKNOWN" and r["reason"] == "DEPTH_NO_TF" for r in after(reps, 0.6))
    e = bag_extrinsic()
    reps, _ = run(sc, extr=validate_extrinsic(CAM_FRAME, np.diag([-1, 1, -1]) @ e.R_body_cam, e.t_body_cam))
    assert all(r["reason"] == "DEPTH_CAMERA_NOT_FORWARD" for r in after(reps, 0.6))
    wrong = validate_extrinsic("some_other_frame", e.R_body_cam, e.t_body_cam)
    reps, _ = run(sc, extr=wrong)
    assert all(r["reason"] == "DEPTH_TF_FRAME_MISMATCH" for r in after(reps, 0.6))


def test_rotation_while_observing_obstacle():
    rot = lambda t: (np.array([1.5, 0, 1.0]), 0.5 * math.sin(t), np.zeros(3))   # yaw sweeps +-0.5 rad, hovering
    reps, _ = run(Scenario(rot, boxes=[WALL], duration=4.0))
    for tt, r in rel(reps):
        if tt > 0.6 and r["verdict"] != "UNKNOWN":
            yaw_cap = 0.5 * math.sin(tt - 0.08)                                    # pose at capture time
            hw = CorridorParams().half_width_m                                     # nearest point is at the corridor edge
            expect = (1.5 - hw * abs(math.sin(yaw_cap))) / math.cos(yaw_cap)
            assert abs(r["free_distance"] - expect) < 0.15, (tt, r["free_distance"], expect)
    assert sum(r["verdict"] == "UNKNOWN" for r in after(reps, 0.6)) == 0


def test_localization_loss_and_recovery():
    reps, _ = run(Scenario(straight(v=0.3), boxes=[WALL], duration=5.0, vo_lost=[(1.5, 2.5)]))
    lost = [r for tt, r in rel(reps) if 1.7 <= tt <= 2.5]
    assert lost and all(r["verdict"] == "STOP" and r["reason"] == "LOCALIZATION_LOST" for r in lost)
    assert all(r["verdict"] in ("SAFE", "LIMIT") for r in after(reps, 2.8))


def test_time_source_mismatch_is_rejected():
    reps, _ = run(Scenario(straight(v=0.3), boxes=[WALL], duration=2.0, depth_stamp_offset=-T0))
    assert all(r["verdict"] == "UNKNOWN" and r["reason"] == "DEPTH_TIME_DOMAIN_MISMATCH" for r in after(reps, 0.6))
    assert reps[-1]["streams"]["depth"]["latency_p50_s"] is None             # no fake latency number
    reps, _ = run(Scenario(straight(v=0.3), boxes=[WALL], duration=2.0, odom_stamp_offset=+5.0))
    assert all(r["verdict"] == "UNKNOWN" and r["reason"] == "ODOM_TIME_DOMAIN_MISMATCH" for r in after(reps, 0.6))


def test_sudden_obstacle_is_caught_within_one_depth_period():
    sc = Scenario(lambda t: (np.array([0.4 * t, 0, 1.0]), 0.0, np.array([0.4, 0, 0])), boxes=[FAR],
                  popup=(2.0, (1.6, 1.8, -0.3, 0.3, 0.0, 2.0)), duration=3.5)
    reps, _ = run(sc)
    before = [r for tt, r in rel(reps) if 1.0 <= tt < 2.0]
    assert all(r["verdict"] == "SAFE" for r in before)
    first_hit = next(tt for tt, r in rel(reps) if tt >= 2.0 and r["verdict"] in ("LIMIT", "STOP"))
    assert first_hit <= 2.0 + 1 / 15 + 0.08 + 0.1 + 1e-6                     # depth period + latency + eval tick


def test_unknown_or_invalid_never_reads_as_safe():
    """Property over every scenario above plus random corruption: no motion allowed without valid evidence."""
    rng = np.random.default_rng(3)

    def noisy(t, img):
        img[rng.random(img.shape) < 0.6] = np.nan
        return img
    scs = [Scenario(straight(v=0.6), boxes=[WALL], duration=3.0, depth_corrupt=noisy),
           Scenario(straight(v=0.3), boxes=[WALL], duration=3.0, depth_gaps=[(1, 2)], odom_gaps=[(1.5, 2.5)]),
           Scenario(straight(v=0.3), boxes=[WALL], duration=3.0, depth_stamp_offset=-T0, vo_lost=[(0.5, 1)])]
    for sc in scs:
        for extr in ("bag", None):
            reps, _ = run(sc, extr=extr)
            for r in reps:
                if r["verdict"] in ("UNKNOWN", "STOP"):
                    assert r["max_speed"] == 0.0
                if r["depth_status"] != "OK" or r["loc_mode"] in ("HOVER", "ABORT"):
                    assert r["max_speed"] == 0.0, r
                if r["valid_fraction"] is not None and r["valid_fraction"] < 0.5:
                    assert r["max_speed"] == 0.0


# ------------------------------------------------------------------ determinism / replay consistency
def test_replay_is_deterministic_and_order_independent():
    sc = Scenario(straight(v=0.6), boxes=[WALL], duration=3.0, reorder_depth=True, depth_gaps=[(1, 1.3)])
    ev = sc.events()
    a = to_jsonl(replay(ev, _mon()))
    b = to_jsonl(replay(list(reversed(ev)), _mon()))                          # input order irrelevant
    c = to_jsonl(replay(sc.events(), _mon()))                                 # regenerated stream identical
    assert a == b == c


def _mon():
    m = ShadowMonitor(); m.set_extrinsic(bag_extrinsic()); return m


def test_live_style_feed_equals_replay():
    """Feeding messages one by one (as the ROS node does) gives the same reports as the replay harness."""
    sc = Scenario(straight(v=0.6), boxes=[WALL], duration=2.0)
    ev = sorted(sc.events(), key=lambda e: e[0])
    ref = replay(ev, _mon())
    from dh_core.shadow.replay import feed
    from dh_core.shadow.monitor import deterministic
    m, out, k = _mon(), [], 0
    for r in ref:
        while k < len(ev) and ev[k][0] <= r["t"] + 1e-12:
            feed(m, ev[k][1], ev[k][2], ev[k][0]); k += 1
        out.append(deterministic(m.evaluate(r["t"])))
    assert to_jsonl(out) == to_jsonl(ref)


# ------------------------------------------------------------------ real bag slice
def test_real_bag_odometry_trace():
    """2026-11-07 drone bag, 180-196 s: healthy tracking, then a real cuVSLAM divergence."""
    reps = replay(load_odom_trace(TRACE), ShadowMonitor())
    s = reps[-1]["streams"]["odom"]
    assert s["clock"] == COMPARABLE and 28 < s["rate_rx_hz"] < 31
    assert 0.0 < s["latency_p50_s"] < 0.4 and s["latency_max_s"] < 0.45
    assert s["out_of_order"] == 0
    modes = [r["loc_mode"] for r in reps]
    i_ok = modes.index("OK")                                                   # first sample: 'no odometry yet'
    i_bad = modes.index("HOVER", i_ok)
    assert i_ok < i_bad < modes.index("ABORT", i_bad)
    assert all(r["max_speed"] == 0.0 for r in reps)                           # no depth in the bag: never permits motion
    assert all(r["verdict"] == "STOP" for r in reps[i_bad:])                  # divergence latches STOP


# ------------------------------------------------------------------ no flight-command path
SHADOW_FILES = sorted(glob.glob(os.path.join(HERE, "..", "dh_core", "shadow", "*.py")))
NODE_FILE = os.path.join(ROOT, "ros2_ws", "src", "dh_autonomy", "dh_autonomy", "shadow_node.py")


def test_static_scan_finds_no_command_path_in_shadow_code():
    files = [f for f in SHADOW_FILES if not f.endswith("guard.py")] + [NODE_FILE]
    assert len(files) >= 8
    assert scan_sources(files) == []


def test_static_scan_positive_control():
    """The scanner does catch real command paths (the existing mission backends)."""
    hits = scan_sources([os.path.join(ROOT, "ros2_ws", "src", "dh_autonomy", "dh_autonomy", "backends.py")])
    kinds = {h[2].split()[0] for h in hits}
    assert "calls" in kinds and any("/fmu/in" in h[2] or "setpoint" in h[2] for h in hits)


def test_endpoint_policy():
    assert check_endpoints(["/rosout", "/parameter_events"], [], ["/dh_shadow/get_parameters"]) == []
    v = check_endpoints(["/skybeta/setpoint/twist", "/dh_shadow/report"], ["/fmu/arm"], ["/dh_shadow/do_land"])
    assert len(v) == 4 and "FLIGHT COMMAND" in v[0]


def test_monitor_holds_no_command_objects():
    m = _mon()
    m.evaluate(T0)
    assert type(m.gate.follower).__name__ == "_InertFollower"
    for name in dir(m):
        assert "publish" not in name.lower() and "client" not in name.lower()


# ------------------------------------------------------------------ Stage 5 interface hooks
def test_latest_observation_bundle_for_stage5():
    from dh_core.stage5_interfaces import NavObservation
    sc = Scenario(straight(v=0.3), boxes=[WALL], duration=2.0)
    mon = _mon()
    assert mon.latest_observation(T0) is None                                  # no odometry yet
    reps = replay(sc.events(), mon)
    obs = mon.latest_observation(reps[-1]["t"])
    assert isinstance(obs, NavObservation) and obs.loc_mode == "OK"
    assert obs.depth is not None and obs.depth.depth_m.shape == (200, 320)
    assert obs.extrinsic.frame_id == CAM_FRAME and obs.safety["verdict"] in ("SAFE", "LIMIT")


def test_stage5_refuses_prior_maps():
    import pytest
    from dh_core.stage5_interfaces import check_no_prior_map
    check_no_prior_map(["/visual_slam/tracking/odometry", "/oak/stereo/image_raw"])
    for bad in (["route.json"], ["sim/models/course_v3/model.sdf"], ["scan.ply"]):
        with pytest.raises(ValueError):
            check_no_prior_map(bad)


def test_stage5_interfaces_have_no_command_path():
    assert scan_sources([os.path.join(HERE, "..", "dh_core", "stage5_interfaces.py")]) == []
