"""Frame alignment (odom <-> course map) and a trajectory follower.

MapAlignment
    cuVSLAM's `odom` frame starts wherever the SLAM node started, with an
    arbitrary heading. At the start of a run the drone sits on the launch mark
    with a known heading, so map_T_odom is fixed from that one pose. Scan
    matching / floor-line corrections then nudge it during the flight.

TrajectoryFollower
    Velocity command = feed-forward + P on position error (+ yaw-rate to face
    the direction of travel). The trajectory clock slows down, or stops, when
    the tracking error grows, so a disturbance never makes the reference run
    away from the drone (this replaces a hard-coded timing schedule).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from .timing import Trajectory


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class MapAlignment:
    def __init__(self):
        self.t = np.zeros(3)      # map = R(yaw) * odom + t
        self.yaw = 0.0
        self.valid = False

    def init_from_pose(self, odom_xyz, odom_yaw: float, map_xyz, map_yaw: float) -> None:
        self.yaw = wrap(map_yaw - odom_yaw)
        self.t = np.asarray(map_xyz, float) - self._rot(np.asarray(odom_xyz, float))
        self.valid = True

    def _rot(self, v: np.ndarray) -> np.ndarray:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]])

    def _irot(self, v: np.ndarray) -> np.ndarray:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return np.array([c * v[0] + s * v[1], -s * v[0] + c * v[1], v[2]])

    def to_map(self, p_odom, yaw_odom: float) -> Tuple[np.ndarray, float]:
        return self._rot(np.asarray(p_odom, float)) + self.t, wrap(yaw_odom + self.yaw)

    def to_odom(self, p_map, yaw_map: Optional[float] = None):
        p = self._irot(np.asarray(p_map, float) - self.t)
        return p, (None if yaw_map is None else wrap(yaw_map - self.yaw))

    def vec_to_odom(self, v_map) -> np.ndarray:
        return self._irot(np.asarray(v_map, float))

    def correct(self, d_xy, d_yaw: float, about_xy, gain: float = 0.5) -> None:
        """Apply a map-frame correction measured at map point `about_xy`.

        d_xy/d_yaw = (true map pose) - (current map estimate). The yaw part is
        applied as a rotation about the drone, so it does not move the drone's
        current position estimate.
        """
        dy = gain * d_yaw
        about = np.array([about_xy[0], about_xy[1], 0.0])
        c, s = math.cos(dy), math.sin(dy)
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        # new map = R (old map - about) + about + gain*d_xy
        self.t = R @ (self.t - about) + about + gain * np.array([d_xy[0], d_xy[1], 0.0])
        self.yaw = wrap(self.yaw + dy)


@dataclass
class FollowerParams:
    kp_xy: float = 1.2
    kp_z: float = 1.0
    k_yaw: float = 1.5
    max_yaw_rate: float = 1.0          # rad/s
    v_cap: float = 1.0                 # m/s horizontal (PX4 MPC_XY_VEL_MAX)
    vz_cap: float = 0.6
    err_slow: float = 0.15             # m: start slowing the reference clock
    err_stop: float = 0.35             # m: freeze the reference clock
    face_travel: bool = True           # keep the forward camera pointing along the path
    finish_tol: float = 0.10
    v_lead: float = 0.30               # s; feed-forward taken this far ahead to cancel PX4's velocity-loop lag


class TrajectoryFollower:
    def __init__(self, traj: Trajectory, prm: FollowerParams = FollowerParams()):
        self.traj = traj
        self.prm = prm
        self.tau = 0.0            # trajectory time (s)
        self.done = False
        self.speed_scale = 1.0    # external governor (e.g. slow down until re-localised)

    def step(self, p_map: np.ndarray, yaw_map: float, dt: float):
        """Returns (v_cmd_map (3,), yaw_rate, p_ref, err)."""
        prm = self.prm
        p_ref, v_ref, _, yaw_ref = self.traj.sample(self.tau)
        e = p_ref - p_map
        err = float(np.linalg.norm(e[:2]))
        # reference-clock governor
        if err <= prm.err_slow:
            rate = 1.0
        elif err >= prm.err_stop:
            rate = 0.0
        else:
            rate = 1.0 - (err - prm.err_slow) / (prm.err_stop - prm.err_slow)
        rate *= float(np.clip(self.speed_scale, 0.0, 1.0))
        self.tau = min(self.traj.duration, self.tau + rate * dt)
        if prm.v_lead > 0:
            _, v_ref, _, _ = self.traj.sample(self.tau + prm.v_lead)
        v = v_ref * rate
        v_cmd = np.array([v[0] + prm.kp_xy * e[0], v[1] + prm.kp_xy * e[1], v[2] + prm.kp_z * e[2]])
        n = math.hypot(v_cmd[0], v_cmd[1])
        if n > prm.v_cap:
            v_cmd[:2] *= prm.v_cap / n
        v_cmd[2] = float(np.clip(v_cmd[2], -prm.vz_cap, prm.vz_cap))
        yaw_rate = 0.0
        if prm.face_travel and np.linalg.norm(v_ref[:2]) > 0.05:
            yaw_rate = float(np.clip(prm.k_yaw * wrap(yaw_ref - yaw_map), -prm.max_yaw_rate, prm.max_yaw_rate))
        if self.tau >= self.traj.duration and np.linalg.norm(self.traj.p[-1] - p_map) < prm.finish_tol:
            self.done = True
        return v_cmd, yaw_rate, p_ref, err
