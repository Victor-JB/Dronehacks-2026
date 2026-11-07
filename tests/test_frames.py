"""dronehacks_nav.frames: the ENU/FLU <-> NED/FRD maps between skybeta and refnav.core."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dronehacks_nav import frames as F  # noqa: E402
from refnav import depth as D  # noqa: E402


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _quat_xyzw(R):
    w, x, y, z = F._quat_wxyz_from_mat(R)
    return x, y, z, w


def test_position_round_trip_and_axes():
    assert F.enu_to_ned(1.0, 2.0, 3.0) == (2.0, 1.0, -3.0)          # east 1, north 2, up 3
    assert F.ned_to_enu(*F.enu_to_ned(1.0, 2.0, 3.0)) == (1.0, 2.0, 3.0)


@pytest.mark.parametrize("yaw_enu, yaw_ned", [(0.0, math.pi / 2), (math.pi / 2, 0.0), (math.pi, -math.pi / 2)])
def test_yaw_swap(yaw_enu, yaw_ned):
    # ENU yaw 0 faces east, which is NED yaw +90 deg
    assert F.yaw_swap(yaw_enu) == pytest.approx(yaw_ned)
    assert F.wrap(F.yaw_swap(F.yaw_swap(yaw_enu)) - yaw_enu) == pytest.approx(0.0, abs=1e-12)


def test_yaw_quaternion_round_trip():
    for yaw in (-2.5, -0.3, 0.0, 1.1, 3.0):
        assert F.yaw_from_quat_xyzw(*F.quat_xyzw_from_yaw(yaw)) == pytest.approx(yaw)


@pytest.mark.parametrize("yaw_enu", [0.0, 0.7, -2.0])
def test_attitude_heading_matches_yaw_swap(yaw_enu):
    q = F.quat_enu_flu_to_ned_frd(*F.quat_xyzw_from_yaw(yaw_enu))
    R = F._mat_from_quat_wxyz(*q)
    assert math.atan2(R[1, 0], R[0, 0]) == pytest.approx(F.yaw_swap(yaw_enu))


def test_nose_up_stays_nose_up():
    # facing ENU north (yaw 90 deg), nose up 10 deg: FLU pitches about its +y (left) axis by -10 deg;
    # in NED that is heading 0 with FRD pitched about its +y (right) axis by +10 deg
    th = math.radians(10.0)
    q_enu = _quat_xyzw(_rz(math.pi / 2) @ _ry(-th))
    q_ned = F.quat_enu_flu_to_ned_frd(*q_enu)
    assert np.allclose(F._mat_from_quat_wxyz(*q_ned), _ry(th), atol=1e-12)


def test_levelled_points_match_px4_attitude():
    # refnav.depth levels with the attitude: an odometry orientation converted here must level a
    # forward camera's points exactly as the equivalent PX4 attitude does
    th = math.radians(6.0)
    q_px4 = F._quat_wxyz_from_mat(_rz(0.4) @ _ry(th))
    q_odom = _quat_xyzw(F._M_NED_ENU.T @ F._mat_from_quat_wxyz(*q_px4) @ F._F_FLU_FRD.T)
    assert np.allclose(D.level_rotation(F.quat_enu_flu_to_ned_frd(*q_odom)), D.level_rotation(q_px4), atol=1e-12)
