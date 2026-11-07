"""dronehacks_nav.frames - skybeta's ENU/FLU world <-> refnav.core's NED/FRD world. PURE (numpy only).

skybeta hands us cuVSLAM's `odom` frame in ENU (+x East, +y North, +z Up) with an FLU body, and
takes ENU setpoints back. refnav.core was written against PX4's local NED (+x North, +y East,
+z Down) with an FRD body. These are the plain REP-103 maps between the two - the same ones
skybeta_flight_interface applies on its way to PX4, so a round trip through both is the identity:

    position:  (n, e, d) = (y, x, -z)          its own inverse up to the order of the result
    yaw:       yaw_ned = pi/2 - yaw_enu        its own inverse
    attitude:  R_ned_frd = M @ R_enu_flu @ F   M swaps x/y and flips z, F = diag(1, -1, -1)

"East" and "North" are only cuVSLAM's startup axes (no compass), and the attitude has no gravity
reference beyond the startup pose (skybeta runs cuVSLAM without IMU fusion).
"""
from __future__ import annotations

import math

import numpy as np

# world ENU -> world NED, and body FLU -> body FRD
_M_NED_ENU = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
_F_FLU_FRD = np.diag([1.0, -1.0, -1.0])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def enu_to_ned(x, y, z):
    return (y, x, -z)


def ned_to_enu(n, e, d):
    return (e, n, -d)


def yaw_swap(yaw):
    """ENU yaw -> NED yaw, and NED yaw -> ENU yaw (the map is its own inverse)."""
    return wrap(math.pi / 2 - yaw)


def yaw_from_quat_xyzw(x, y, z, w):
    """Heading of a ROS (x, y, z, w) quaternion: rotation about the world z axis."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_xyzw_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _mat_from_quat_wxyz(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _quat_wxyz_from_mat(R):
    """Rotation matrix -> (w, x, y, z) with w >= 0 (Shepperd's method)."""
    t = np.trace(R)
    if t > 0.0:
        s = 2.0 * math.sqrt(1.0 + t)
        q = (0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s)
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q = ((R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s)
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        q = ((R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s)
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        q = ((R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s)
    q = np.array(q)
    q /= np.linalg.norm(q)
    return tuple(float(c) for c in (q if q[0] >= 0 else -q))


def quat_enu_flu_to_ned_frd(x, y, z, w):
    """A ROS odometry orientation (ENU <- FLU, x y z w) -> the PX4-style attitude refnav.core and
    refnav.depth expect (NED <- FRD, w x y z)."""
    R = _M_NED_ENU @ _mat_from_quat_wxyz(w, x, y, z) @ _F_FLU_FRD
    return _quat_wxyz_from_mat(R)
