"""Deterministic tests for the Stage 4 safety filter (dh_core.safety_filter). No ROS, no hardware."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from dh_core.safety_filter import (LIMIT, SAFE, STOP, UNKNOWN, SafetyEvaluator, SafetyGate, SafetyParams,
                                   SensorInputs, max_speed_for_distance, stopping_distance)
from dh_core.timing import Trajectory
from dh_core.tracker import FollowerParams, TrajectoryFollower

NOW = 100.0


def good(**kw):
    d = dict(now=NOW, free_distance=3.0, depth_stamp=NOW - 0.05, depth_rx=NOW - 0.03,
             odom_stamp=NOW - 0.03, odom_rx=NOW - 0.02, pos_std=None, transform_valid=True, loc_mode="OK")
    d.update(kw)
    return SensorInputs(**d)


EV = SafetyEvaluator()


# ------------------------------------------------------------------ stopping distance
def test_stopping_distance_formula():
    assert stopping_distance(0.0, 0.4, 1.0) == 0.0
    assert math.isclose(stopping_distance(1.0, 0.4, 1.0), 0.4 + 0.5)
    assert math.isclose(stopping_distance(2.0, 0.0, 2.0), 1.0)
    assert stopping_distance(-1.0, 0.4, 1.0) == 0.0            # no negative speeds


def test_max_speed_inverts_stopping_distance():
    for usable in (0.1, 0.5, 1.0, 2.5):
        for lat, a in ((0.4, 1.0), (0.1, 3.0), (0.0, 1.5)):
            v = max_speed_for_distance(usable, lat, a)
            assert math.isclose(stopping_distance(v, lat, a), usable, rel_tol=1e-9)
    assert max_speed_for_distance(0.0, 0.4, 1.0) == 0.0
    assert max_speed_for_distance(-1.0, 0.4, 1.0) == 0.0


def test_worse_braking_or_latency_never_allows_more_speed():
    base = max_speed_for_distance(1.0, 0.2, 2.0)
    assert max_speed_for_distance(1.0, 0.2, 1.0) < base        # weaker brakes
    assert max_speed_for_distance(1.0, 0.5, 2.0) < base        # more latency
    assert max_speed_for_distance(0.8, 0.2, 2.0) < base        # less room


def test_default_braking_is_not_the_sim_3mps2_cap():
    assert SafetyParams().brake_decel < 3.0


# ------------------------------------------------------------------ verdicts
def test_safe_far_obstacle():
    v = EV.evaluate(good(free_distance=3.0), requested_speed=0.5)
    assert v.verdict == SAFE and v.max_speed == 0.5 and v.reason == "OK"


def test_limit_when_braking_distance_is_short():
    v = EV.evaluate(good(free_distance=1.2), requested_speed=1.0)
    assert v.verdict == LIMIT and 0.05 < v.max_speed < 1.0 and v.reason == "BRAKING_LIMIT"
    # and the limit is exactly the speed we can stop from
    p = SafetyParams()
    assert stopping_distance(v.max_speed, p.latency_s, p.brake_decel) <= 1.2 - p.vehicle_radius - p.uncertainty_margin + 1e-9


def test_stop_when_inside_vehicle_radius_plus_margin():
    p = SafetyParams()
    v = EV.evaluate(good(free_distance=p.vehicle_radius + p.uncertainty_margin - 0.01), requested_speed=0.3)
    assert v.verdict == STOP and v.max_speed == 0.0 and v.reason == "NO_STOPPING_ROOM"
    assert EV.evaluate(good(free_distance=0.0)).verdict == STOP


def test_unlimited_range_is_capped_at_reliable_range():
    v_inf = EV.evaluate(good(free_distance=math.inf), requested_speed=1.0)
    v_rel = EV.evaluate(good(free_distance=SafetyParams().reliable_range), requested_speed=1.0)
    assert (v_inf.verdict, v_inf.max_speed) == (v_rel.verdict, v_rel.max_speed)
    short = SafetyEvaluator(SafetyParams(reliable_range=0.8))
    assert short.evaluate(good(free_distance=math.inf), 1.0).verdict in (LIMIT, STOP)   # inf != free forever


def test_request_is_clipped_to_v_max_and_negative_is_zero():
    v = EV.evaluate(good(), requested_speed=9.0)
    assert v.max_speed <= SafetyParams().v_max
    v = EV.evaluate(good(), requested_speed=-3.0)
    assert v.verdict == SAFE and v.max_speed == 0.0


def test_max_speed_monotonic_in_free_distance():
    prev = -1.0
    for d in np.linspace(0.0, 3.0, 40):
        v = EV.evaluate(good(free_distance=float(d)), 1.0).max_speed
        assert v >= prev - 1e-12
        prev = v


# ------------------------------------------------------------------ uncertainty
def test_position_uncertainty_reduces_allowed_speed():
    a = EV.evaluate(good(free_distance=1.3, pos_std=0.0), 1.0).max_speed
    b = EV.evaluate(good(free_distance=1.3, pos_std=0.15), 1.0).max_speed
    assert b < a


def test_degraded_localization_caps_speed_and_adds_margin():
    p = SafetyParams()
    v = EV.evaluate(good(free_distance=3.0, loc_mode="DEGRADED"), 1.0)
    assert v.verdict == LIMIT and v.max_speed <= p.degraded_speed_cap + 1e-9 and v.reason == "LOCALIZATION_DEGRADED"
    ok = EV.evaluate(good(free_distance=1.0, loc_mode="OK"), 0.4).max_speed
    dg = EV.evaluate(good(free_distance=1.0, loc_mode="DEGRADED"), 0.4).max_speed
    assert dg <= ok


# ------------------------------------------------------------------ stale / missing / invalid
def check(inp, verdict, reason, req=0.5):
    v = EV.evaluate(inp, req)
    assert (v.verdict, v.max_speed, v.reason) == (verdict, 0.0, reason), v


def test_stale_depth_by_capture_stamp():
    check(good(depth_stamp=NOW - 0.31, depth_rx=NOW - 0.01), UNKNOWN, "DEPTH_STALE")


def test_stale_depth_by_receipt_time():
    # captured long ago, delivered late and the stream has since stopped: stale on both clocks
    check(good(depth_stamp=NOW - 0.6, depth_rx=NOW - 0.5), UNKNOWN, "DEPTH_STALE")
    # delivered just now but captured long ago (pipeline backlog): the scene it shows is old
    check(good(depth_stamp=NOW - 0.45, depth_rx=NOW - 0.1), UNKNOWN, "DEPTH_STALE")
    check(good(depth_stamp=NOW - 0.1, depth_rx=NOW - 0.5), UNKNOWN, "DEPTH_INVALID")  # received before captured


def test_depth_boundary_is_fresh():
    assert EV.evaluate(good(depth_stamp=NOW - 0.29, depth_rx=NOW - 0.29), 0.5).verdict == SAFE


def test_missing_depth():
    check(good(depth_stamp=None, depth_rx=None), UNKNOWN, "DEPTH_MISSING")
    check(good(free_distance=None), UNKNOWN, "DEPTH_MISSING")


def test_invalid_depth_values():
    check(good(free_distance=math.nan), UNKNOWN, "DEPTH_INVALID")
    check(good(free_distance=-0.2), UNKNOWN, "DEPTH_INVALID")
    check(good(depth_valid_fraction=math.nan), UNKNOWN, "DEPTH_INVALID")
    check(good(depth_valid_fraction=1.5), UNKNOWN, "DEPTH_INVALID")


def test_mostly_invalid_pixels_are_unknown_not_clear():
    # corridor with +inf (no return) but only 20% valid pixels must not be read as free space
    check(good(free_distance=math.inf, depth_valid_fraction=0.2), UNKNOWN, "DEPTH_COVERAGE_LOW")


def test_bad_depth_clock():
    check(good(depth_stamp=0.0, depth_rx=NOW), UNKNOWN, "DEPTH_INVALID")                 # unstamped
    check(good(depth_stamp=NOW + 5.0, depth_rx=NOW), UNKNOWN, "DEPTH_INVALID")           # from the future
    check(good(depth_stamp=math.nan, depth_rx=NOW), UNKNOWN, "DEPTH_INVALID")
    check(good(depth_stamp=NOW - 0.1, depth_rx=NOW - 0.5), UNKNOWN, "DEPTH_INVALID")     # received before it was captured


def test_stale_missing_invalid_odometry():
    check(good(odom_stamp=NOW - 0.5, odom_rx=NOW - 0.01), UNKNOWN, "ODOM_STALE")
    check(good(odom_stamp=NOW - 0.7, odom_rx=NOW - 0.6), UNKNOWN, "ODOM_STALE")
    check(good(odom_stamp=NOW - 0.01, odom_rx=NOW - 0.6), UNKNOWN, "ODOM_INVALID")    # received before captured
    check(good(odom_stamp=None, odom_rx=None), UNKNOWN, "ODOM_MISSING")
    check(good(odom_stamp=NOW + 3, odom_rx=NOW), UNKNOWN, "ODOM_INVALID")
    check(good(pos_std=math.nan), UNKNOWN, "ODOM_INVALID")
    check(good(pos_std=-1.0), UNKNOWN, "ODOM_INVALID")


def test_invalid_transform():
    check(good(transform_valid=False), UNKNOWN, "TRANSFORM_INVALID")


def test_lost_localization_stops():
    for mode in ("HOVER", "ABORT"):
        v = EV.evaluate(good(loc_mode=mode), 0.5)
        assert (v.verdict, v.max_speed, v.reason) == (STOP, 0.0, "LOCALIZATION_LOST")
    check(good(loc_mode=None), UNKNOWN, "LOCALIZATION_UNKNOWN")
    check(good(loc_mode="bogus"), UNKNOWN, "LOCALIZATION_UNKNOWN")


def test_invalid_command_and_config_fail_closed():
    assert EV.evaluate(good(), requested_speed=math.nan).reason == "COMMAND_INVALID"
    assert EV.evaluate(good(), requested_speed=math.inf).verdict == UNKNOWN
    for bad in (SafetyParams(brake_decel=0.0), SafetyParams(brake_decel=math.nan), SafetyParams(latency_s=-1),
                SafetyParams(v_max=0.0), SafetyParams(vehicle_radius=-0.1)):
        v = SafetyEvaluator(bad).evaluate(good(), 0.5)
        assert (v.verdict, v.max_speed, v.reason) == (UNKNOWN, 0.0, "CONFIG_INVALID")


def test_no_verdict_ever_allows_motion_with_bad_inputs():
    """Brute force: any single corrupted field must give max_speed 0."""
    corruptions = [dict(free_distance=math.nan), dict(free_distance=None), dict(depth_stamp=None, depth_rx=None),
                   dict(depth_stamp=NOW - 9), dict(odom_stamp=None, odom_rx=None), dict(odom_stamp=NOW - 9),
                   dict(transform_valid=False), dict(loc_mode=None), dict(loc_mode="ABORT"),
                   dict(depth_valid_fraction=0.0), dict(pos_std=math.inf)]
    for c in corruptions:
        assert EV.evaluate(good(**c), 1.0).max_speed == 0.0, c


# ------------------------------------------------------------------ follower / speed_scale
def line_traj(length=6.0, v=1.0, dt=0.05):
    t = np.arange(0.0, length / v + 1e-9, dt)
    s = v * t
    p = np.stack([s, np.zeros_like(s), np.full_like(s, 1.0)], axis=1)
    return Trajectory(p, s, np.full_like(s, v), t, np.zeros_like(s))


def make_follower(**kw):
    return TrajectoryFollower(line_traj(**kw), FollowerParams(v_cap=1.0, v_lead=0.0))


def test_raw_speed_scale_zero_still_chases_position_error():
    """CHARACTERISATION of the existing tracker: speed_scale=0 freezes the reference clock but the P term
    still drives toward a reference that is ahead. This is why SafetyGate clamps the output."""
    f = make_follower()
    f.step(np.array([0.0, 0.0, 1.0]), 0.0, 0.05)                 # reference advances
    for _ in range(10):
        f.step(np.array([0.0, 0.0, 1.0]), 0.0, 0.05)
    f.speed_scale = 0.0
    v, _, p_ref, err = f.step(np.array([0.0, 0.0, 1.0]), 0.0, 0.05)
    assert err > 0.0 and np.linalg.norm(v[:2]) > 0.05            # still moving toward p_ref


def test_raw_speed_scale_half_does_not_cap_speed():
    f = make_follower()
    f.speed_scale = 0.5
    # large position error: P term alone commands the full v_cap regardless of speed_scale
    v, *_ = f.step(np.array([-1.0, 0.0, 1.0]), 0.0, 0.05)
    assert np.linalg.norm(v[:2]) > 0.9


def test_gate_zero_verdict_zeroes_the_command_even_with_queued_motion():
    f = make_follower()
    g = SafetyGate(f)
    for _ in range(10):                                          # build up reference lead
        g.step(np.array([0.0, 0.0, 1.0]), 0.0, 0.05, good())
    tau_before = f.tau
    for bad in (good(free_distance=0.1), good(depth_stamp=NOW - 5), good(loc_mode="ABORT")):
        v, yaw, _, _, verdict = g.step(np.array([0.0, 0.0, 1.0]), 0.0, 0.05, bad)
        assert verdict.max_speed == 0.0
        assert np.all(v == 0.0) and yaw == 0.0
    assert f.tau == tau_before                                   # reference clock frozen: no run-away


def test_gate_limit_caps_command_speed():
    f = make_follower()
    g = SafetyGate(f)
    inp = good(free_distance=1.2)
    cap = EV.evaluate(inp, 1.0).max_speed
    assert 0.05 < cap < 1.0
    for _ in range(20):
        v, _, _, _, verdict = g.step(np.array([-1.0, 0.0, 1.0]), 0.0, 0.05, inp)   # big error -> P term wants v_cap
        assert verdict.verdict == LIMIT
        assert np.linalg.norm(v) <= cap + 1e-9


def test_gate_fails_closed_before_first_update():
    g = SafetyGate(make_follower())
    v, yaw = g.clamp(np.array([1.0, 0.0, 0.0]), 0.5)
    assert np.all(v == 0.0) and yaw == 0.0


def test_gate_rejects_nonfinite_commands():
    g = SafetyGate(make_follower())
    g.update(good(), 0.5)
    for bad in ([math.nan, 0, 0], [math.inf, 0, 0]):
        v, yaw = g.clamp(np.array(bad), 0.0)
        assert np.all(v == 0.0)
    v, yaw = g.clamp(np.array([0.1, 0, 0]), math.nan)
    assert np.all(v == 0.0) and yaw == 0.0


def test_gate_never_increases_a_command():
    g = SafetyGate(make_follower())
    g.update(good(), 0.5)
    v, yaw = g.clamp(np.array([0.1, 0.0, 0.0]), 0.3)
    assert np.allclose(v, [0.1, 0, 0]) and yaw == 0.3


# ------------------------------------------------------------------ closed loop: does it actually stop in time?
def fly_at_wall(wall_x, lag, decel_phys, latency_samples, dt=0.02, t_end=14.0):
    """Point-mass drone, first-order velocity lag + accel limit, pose latency, pursuing a path THROUGH a wall.
    Depth is measured from the (true) position; returns (min distance to wall surface, final speed)."""
    traj = line_traj(length=wall_x + 2.0, v=1.0)
    f = TrajectoryFollower(traj, FollowerParams(v_cap=1.0, v_lead=0.3))
    g = SafetyGate(f)
    pos, vel = np.array([0.0, 0.0, 1.0]), 0.0
    hist = [pos.copy()] * (latency_samples + 1)
    t, min_gap = 0.0, 1e9
    while t < t_end:
        now = NOW + t
        est = hist[0]                                            # delayed pose estimate
        free = max(0.0, wall_x - est[0])
        inp = good(now=now, free_distance=free, depth_stamp=now - 0.02, depth_rx=now - 0.01,
                   odom_stamp=now - 0.02, odom_rx=now - 0.01)
        v_cmd, _, _, _, _ = g.step(est, 0.0, dt, inp)
        # plant: first-order lag toward command with accel limit
        dv = (v_cmd[0] - vel) / lag * dt
        dv = float(np.clip(dv, -decel_phys * dt, decel_phys * dt))
        vel += dv
        pos = pos + np.array([vel * dt, 0.0, 0.0])
        hist.append(pos.copy()); hist.pop(0)
        min_gap = min(min_gap, wall_x - pos[0])
        t += dt
    return min_gap, vel


def test_closed_loop_never_reaches_wall_with_weaker_brakes_than_sim():
    p = SafetyParams()
    for decel_phys in (1.0, 1.5, 3.0):                           # real braking >= the filter's assumption
        gap, v_final = fly_at_wall(wall_x=3.0, lag=0.3, decel_phys=decel_phys, latency_samples=4)
        assert gap >= p.vehicle_radius, (decel_phys, gap)        # body never touches the wall
        assert v_final < 0.05


def test_closed_loop_demonstrates_the_raw_follower_would_crash():
    """Same scenario with the gate bypassed: the follower flies straight into the wall."""
    traj = line_traj(length=5.0, v=1.0)
    f = TrajectoryFollower(traj, FollowerParams(v_cap=1.0, v_lead=0.3))
    pos = np.array([0.0, 0.0, 1.0]); vel = 0.0; wall = 3.0
    for _ in range(int(14.0 / 0.02)):
        v_cmd, *_ = f.step(pos, 0.0, 0.02)
        vel += float(np.clip((v_cmd[0] - vel) / 0.3 * 0.02, -0.06, 0.06))
        pos = pos + np.array([vel * 0.02, 0, 0])
    assert pos[0] > wall - SafetyParams().vehicle_radius
