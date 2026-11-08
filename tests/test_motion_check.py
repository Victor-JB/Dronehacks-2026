"""dronehacks_nav.motion_check: commanded motion must show up in odometry, and VIO height must agree
with PX4's rangefinder height."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dronehacks_nav.motion_check import MotionCheck  # noqa: E402

DT = 0.05


def run(check, seconds, sp, pos, z_px4=None, t0=0.0):
    """sp, pos, z_px4: functions of t. -> (first failure time, reason) or (None, None)."""
    t = t0
    while t < t0 + seconds:
        why = check.step(t, sp(t), pos(t), None if z_px4 is None else z_px4(t))
        if why:
            return t, why
        t += DT
    return None, None


def test_a_vehicle_that_follows_passes():
    # climb to 1.2 m at 0.5 m/s with the vehicle 0.4 m behind the setpoint the whole way
    sp = lambda t: (0.0, 0.0, min(1.2, 0.5 * t))
    pos = lambda t: (0.0, 0.0, max(0.0, min(1.2, 0.5 * t - 0.4)))
    assert run(MotionCheck(), 10.0, sp, pos, z_px4=lambda t: pos(t)[2]) == (None, None)


def test_no_motion_freezes_after_the_window():
    # setpoint 1 m up, motors produce nothing (props off, or VIO stuck)
    t, why = run(MotionCheck(), 10.0, lambda t: (0.0, 0.0, 1.0), lambda t: (0.0, 0.0, 0.0))
    assert t is not None and 3.0 <= t < 3.2, t
    assert "moved +0.00 m toward it" in why


def test_moving_the_wrong_way_freezes():
    t, why = run(MotionCheck(), 10.0, lambda t: (2.0, 0.0, 1.0), lambda t: (-0.1 * t, 0.0, 1.0))
    assert t is not None and "toward it" in why


def test_a_moving_carrot_with_steady_lag_passes():
    # the setpoint leads by 0.6 m along x at 0.5 m/s: the gap never closes, but the vehicle moves
    sp = lambda t: (0.5 * t + 0.6, 0.0, 1.0)
    pos = lambda t: (0.5 * t, 0.0, 1.0)
    assert run(MotionCheck(), 10.0, sp, pos) == (None, None)


def test_hovering_still_is_not_a_failure():
    assert run(MotionCheck(), 10.0, lambda t: (0.0, 0.0, 1.0), lambda t: (0.05, 0.0, 1.02)) == (None, None)


def test_vio_height_disagreeing_with_the_rangefinder_freezes():
    # VIO says it climbed 1 m; the rangefinder saw nothing
    pos = lambda t: (0.0, 0.0, min(1.0, 0.5 * t))
    t, why = run(MotionCheck(), 10.0, lambda t: pos(t), pos, z_px4=lambda t: 0.0)
    assert t is not None and "rangefinder" in why


def test_the_failure_latches_until_reset():
    c = MotionCheck()
    _, why = run(c, 5.0, lambda t: (0.0, 0.0, 1.0), lambda t: (0.0, 0.0, 0.0))
    assert c.step(100.0, (0.0, 0.0, 1.0), (0.0, 0.0, 1.0)) == why
    c.reset()
    assert c.step(100.0, (0.0, 0.0, 1.0), (0.0, 0.0, 1.0)) is None


def test_same_decisions_at_unix_time():
    a = run(MotionCheck(), 10.0, lambda t: (0.0, 0.0, 1.0), lambda t: (0.0, 0.0, 0.0))
    b = run(MotionCheck(), 10.0, lambda t: (0.0, 0.0, 1.0), lambda t: (0.0, 0.0, 0.0), t0=1.79e9)
    assert b[0] is not None and abs((b[0] - 1.79e9) - a[0]) < 2 * DT
