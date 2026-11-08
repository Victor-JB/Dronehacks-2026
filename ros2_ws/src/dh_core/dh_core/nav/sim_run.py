"""Closed-loop offline simulation: hidden World -> rendered depth/RGB + noisy odometry -> Explorer ->
CommandSafety (Stage 4 gate on the final command) -> organizer-interface limits -> simple dynamics.

The explorer receives ONLY images, K, the camera extrinsic and odometry. The World is used here to render
sensors and to score contact. Simplifications (NOT real-world validation): point-mass dynamics with a
first-order velocity lag and acceleration limit, perfect rendering with optional noise, odometry = truth +
noise + optional drift, no PX4, no takeoff (starts hovering at cruise height), descent disabled.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

import numpy as np

from ..shadow.observation import rot_from_quat_xyzw
from ..shadow.synth import bag_extrinsic, quat_from_rpy
from .explorer import Explorer, ExplorerParams
from .online_map import MapParams
from .safety3d import CommandSafety, CommandSafetyParams
from .sim_world import Tube, World, wall_with_tube, wall_with_window

K_SIM = (198.3984 / 2, 198.4348 / 2, 162.4510 / 2, 101.9492 / 2)     # bag intrinsics at half resolution
W_SIM, H_SIM = 160, 100


@dataclass
class SimConfig:
    dt: float = 0.05
    frame_dt: float = 0.1                # depth/RGB at 10 Hz
    ctrl_dt: float = 0.1                 # setpoints at 10 Hz (organizer recommendation)
    t_max: float = 240.0
    tau: float = 0.3                     # velocity-loop lag
    a_max: float = 2.0
    true_radius: float = 0.24            # contact if the body centre is closer than this to a surface
    odom_noise: float = 0.005
    odom_drift_per_m: float = 0.0        # random-walk drift std per sqrt(m)
    depth_noise: float = 0.0             # relative depth noise (std)
    depth_dropout: float = 0.0           # fraction of pixels with no return
    stale_odom: List[Tuple[float, float]] = field(default_factory=list)
    stale_depth: List[Tuple[float, float]] = field(default_factory=list)
    vo_lost: List[Tuple[float, float]] = field(default_factory=list)
    blue_noise_px: int = 0               # random blue speckles per frame (noisy detections)
    popups: List[Tuple[float, Tuple]] = field(default_factory=list)    # (t, box) appears at t
    seed: int = 0


@dataclass
class SimResult:
    reached: bool
    final_state: str
    time: float
    collisions: int
    min_clearance: float
    path_len: float
    pad_error: Optional[float]
    target_sigma: Optional[float]
    replans: int
    plan_ms_p50: float
    plan_ms_max: float
    max_z: float
    min_z: float
    log: List[str]
    traj: np.ndarray
    explorer: Explorer = None


def course_a(pad=(1.0, 4.6, 0.6), window=(0.2, 1.1, 0.4, 1.2), tube_r=(0.47, 0.55), extra_boxes=()):
    """6 x 6 m arena: south room with a dead-end corridor, a wall with a window, a middle room with a
    pillar, a wall with a cylindrical tunnel, a north room with the blue square. Walls 1.7 m tall."""
    H, T = 1.7, 0.1
    lo, hi = -0.7, 5.3
    boxes = [(lo - T, lo, lo, hi, 0, H), (hi, hi + T, lo, hi, 0, H), (lo, hi, lo - T, lo, 0, H), (lo, hi, hi, hi + T, 0, H)]
    boxes.append((1.2, hi, 0.8, 0.9, 0, H))                                   # dead-end corridor partition
    boxes += wall_with_window(0, 1.6, lo, hi, T, H, *window)                 # wall + window
    boxes.append((2.0, 2.5, 2.3, 2.8, 0, H))                                  # pillar
    tube = Tube(1, 3.0, 4.0, 4.2, 0.75, *tube_r)
    boxes += wall_with_tube(0, 3.4, lo, hi, T, H, tube)                      # wall + tunnel
    boxes += list(extra_boxes)
    return World(boxes=boxes, tubes=[tube], pad=pad)


class Sim:
    def __init__(self, world: World, cfg: SimConfig = SimConfig(), start=(0.0, 0.0, 0.75), yaw0=0.0,
                 mprm: Optional[MapParams] = None, eprm: Optional[ExplorerParams] = None):
        self.w, self.cfg = world, cfg
        self.rng = np.random.default_rng(cfg.seed)
        self.pos = np.array(start, float)
        self.vel = np.zeros(3)
        self.yaw, self.yaw_rate = yaw0, 0.0
        self.extr = bag_extrinsic()
        self.mprm = mprm or MapParams(lo_xy=(-1.0, -1.0), size_xy=6.6)
        self.ex = Explorer(self.mprm, eprm or ExplorerParams())
        self.safety = CommandSafety(CommandSafetyParams(band=self.mprm.band, robot_radius=self.mprm.robot_radius,
                                                          geofence=(-0.7, 5.3, -0.7, 5.3)))
        self.drift = np.zeros(3)
        self.depth_stamp = None
        self.odom_stamp = None

    def _in(self, t, wins):
        return any(a <= t < b for a, b in wins)

    def odom(self):
        n = self.rng.normal(0, self.cfg.odom_noise, 3)
        return self.pos + self.drift + n, self.yaw

    def run(self) -> SimResult:
        c = self.cfg
        t, steps = 0.0, 0
        n_frame, n_ctrl = int(round(c.frame_dt / c.dt)), int(round(c.ctrl_dt / c.dt))
        cmd_v, cmd_yr = np.zeros(3), 0.0
        collisions, in_contact, min_clear = 0, False, math.inf
        traj, path_len = [self.pos.copy()], 0.0
        max_z, min_z = self.pos[2], self.pos[2]
        active_popups = []
        while t < c.t_max:
            for tp, box in c.popups:
                if t >= tp and box not in self.w.boxes:
                    self.w.boxes.append(box)
            loc_ok = not self._in(t, c.vo_lost)
            if steps % n_frame == 0 and not self._in(t, c.stale_depth):
                q = quat_from_rpy(0.0, 0.0, self.yaw)
                Rwb = rot_from_quat_xyzw(*q)
                Rwc_true = Rwb @ self.extr.R_body_cam
                o_true = self.pos + Rwb @ self.extr.t_body_cam
                depth, rgb = self.w.render(o_true, Rwc_true, K_SIM, W_SIM, H_SIM)
                if c.depth_noise > 0:
                    depth = depth * (1 + self.rng.normal(0, c.depth_noise, depth.shape)).astype(np.float32)
                if c.depth_dropout > 0:
                    depth[self.rng.random(depth.shape) < c.depth_dropout] = np.nan
                if c.blue_noise_px > 0:
                    idx = self.rng.integers(0, depth.size, c.blue_noise_px)
                    rgb.reshape(-1, 3)[idx] = (30, 70, 210)
                # the policy maps with the ODOMETRY pose (it never knows the truth)
                p_od, yaw_od = self.odom()
                Rwb_od = rot_from_quat_xyzw(*quat_from_rpy(0.0, 0.0, yaw_od))
                self.ex.on_frame(t, depth, rgb, K_SIM, Rwb_od @ self.extr.R_body_cam, p_od + Rwb_od @ self.extr.t_body_cam,
                                 loc_ok)
                self.depth_stamp = t
            if steps % n_ctrl == 0:
                fresh_odom = not self._in(t, c.stale_odom)
                if fresh_odom:
                    self.odom_stamp = t
                p_od, yaw_od = self.odom()
                intent = self.ex.step(t, p_od, yaw_od, self.vel, loc_ok and fresh_odom)
                pl = self.ex.pl
                dist = pl.dist if pl is not None else None
                grid = pl.grid if pl is not None else None
                cmd_v, cmd_yr, verdict = self.safety.filter(
                    intent.velocity, intent.yaw_rate, p_od, t, dist, grid,
                    self.odom_stamp, self.odom_stamp, self.depth_stamp, self.depth_stamp,
                    "OK" if loc_ok else "HOVER")
                if self.ex.state == "OVER_TARGET" and np.linalg.norm(self.vel) < 0.05:
                    break
            # dynamics: first-order velocity lag with an acceleration limit; yaw-rate lag
            dv = (cmd_v - self.vel) / c.tau * c.dt
            nrm = np.linalg.norm(dv)
            if nrm > c.a_max * c.dt:
                dv *= c.a_max * c.dt / nrm
            self.vel += dv
            self.yaw_rate += (cmd_yr - self.yaw_rate) / 0.2 * c.dt
            step = self.vel * c.dt
            self.pos += step
            self.yaw = (self.yaw + self.yaw_rate * c.dt + math.pi) % (2 * math.pi) - math.pi
            path_len += float(np.linalg.norm(step[:2]))
            if c.odom_drift_per_m > 0:
                self.drift[:2] += self.rng.normal(0, c.odom_drift_per_m * math.sqrt(max(np.linalg.norm(step[:2]), 1e-9)), 2)
            d = self.w.distance(self.pos)
            min_clear = min(min_clear, d)
            if d < c.true_radius and not in_contact:
                collisions += 1
            in_contact = d < c.true_radius
            max_z, min_z = max(max_z, self.pos[2]), min(min_z, self.pos[2])
            if steps % 4 == 0:
                traj.append(self.pos.copy())
            t += c.dt
            steps += 1
        pad = self.w.pad
        err = float(np.hypot(self.pos[0] - pad[0], self.pos[1] - pad[1])) if pad[2] > 0 else None
        pt = np.array(self.ex.plan_times) * 1e3 if self.ex.plan_times else np.array([0.0])
        return SimResult(self.ex.state == "OVER_TARGET" and err is not None and err <= pad[2] / 2, self.ex.state, t,
                         collisions, min_clear, path_len, err,
                         self.ex.tracker.sigma if self.ex.tracker.xy is not None else None,
                         self.ex.replans, float(np.median(pt)), float(pt.max()), max_z, min_z, self.ex.log,
                         np.array(traj), self.ex)
