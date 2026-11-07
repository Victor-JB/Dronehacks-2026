"""Lightweight closed-loop simulator for strategy evaluation (no ROS, no physics engine).

Models, in the horizontal plane:
  * our tracker:   v_cmd = v_ff + Kp (p_ref - p_est), saturated
  * PX4 velocity loop: first-order lag tau with an acceleration limit
  * pose estimate: noise + latency + drift (random walk per metre flown)
  * "dark zones" (tube): visual odometry lost -> estimate drifts with a
    constant velocity error until the zone is left; optionally re-anchored
    afterwards (models a perception-based correction)
Contact is checked against the *un-inflated* obstacle distance field using the
true drone radius. This is a screening tool for comparing strategies, not a
substitute for SITL/real tests: all noise numbers are assumptions to vary.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from .course import Grid2D
from .timing import Trajectory


@dataclass
class SimParams:
    dt: float = 0.02
    kp: float = 1.2                 # our outer-loop gain (1/s)
    v_sat: float = 1.0              # PX4 MPC_XY_VEL_MAX
    tau: float = 0.30               # PX4 velocity-loop time constant (s)
    a_phys: float = 4.0             # max horizontal accel (m/s^2)
    latency: float = 0.07           # pose latency (s), EKF2_EV_DELAY
    noise: float = 0.01             # pose noise std (m)
    drift_per_m: float = 0.01       # random-walk drift std per sqrt(metre) flown
    dark_vel_err: float = 0.08      # std of velocity error while VO is lost (m/s)
    reanchor_after_dark: bool = False
    reanchor_residual: float = 0.03  # std of residual error after re-anchoring (m)
    true_radius: float = 0.24       # m, physical half-span used for contact
    crash_depth: float = 0.05       # penetration beyond which a touch is a crash
    use_feedforward: bool = True
    follower: Optional[object] = None   # dh_core.tracker.FollowerParams -> use the flight code's follower


@dataclass
class SimResult:
    time: float
    touches: int
    crashed: bool
    min_clearance: float
    max_track_err: float
    final_err: float


@dataclass
class ScanAiding:
    """Map-relative correction from a range sensor + DistanceFieldMatcher."""
    matcher: object                 # dh_core.localize.DistanceFieldMatcher
    scan_grid: Grid2D               # occupancy used to synthesise scans
    rate_hz: float = 5.0
    fov_deg: float = 80.0           # 80 = OAK-D mono HFOV; 360 = 2D LiDAR
    r_max: float = 4.0
    works_in_dark: bool = False     # passive stereo: False; LiDAR/ToF: True
    gain: float = 0.6               # fraction of the observable correction applied


def simulate(traj: Trajectory, dist: np.ndarray, grid: Grid2D, prm: SimParams,
             dark_zones: Optional[List[np.ndarray]] = None, rng: Optional[np.random.Generator] = None,
             time_scale: float = 1.0, aiding: Optional[ScanAiding] = None) -> SimResult:
    rng = rng or np.random.default_rng()
    dark_zones = dark_zones or []
    follower = None
    if prm.follower is not None:
        from .tracker import TrajectoryFollower
        follower = TrajectoryFollower(traj, prm.follower)
    p = traj.p[0, :2].copy()
    v = np.zeros(2)
    bias = np.zeros(2)
    dark_v = None
    hist: List[Tuple[float, np.ndarray]] = []
    t = 0.0
    T = traj.duration / time_scale
    touches, in_contact, crashed = 0, False, False
    min_clear, max_err = np.inf, 0.0
    flown = 0.0
    t_limit = T + 3.0 if follower is None else 2.0 * T + 10.0
    while t < t_limit:
        # --- estimate (with latency)
        in_dark = any(lo[0] <= p[0] <= hi[0] and lo[1] <= p[1] <= hi[1] for lo, hi in dark_zones)
        if in_dark:
            if dark_v is None:
                dark_v = rng.normal(0, prm.dark_vel_err, 2)
            bias = bias + dark_v * prm.dt
        elif dark_v is not None:
            dark_v = None
            if prm.reanchor_after_dark:
                bias = rng.normal(0, prm.reanchor_residual, 2)
        hist.append((t, p + bias + rng.normal(0, prm.noise, 2)))
        t_meas = t - prm.latency
        est = hist[0][1]
        for tt, pe in reversed(hist):
            if tt <= t_meas:
                est = pe
                break
        if len(hist) > 50:
            hist = hist[-50:]
        # --- reference & control
        if follower is not None:
            yaw_now = traj.yaw[traj.nearest_index(np.array([est[0], est[1], traj.p[0, 2]]))]
            v3, _, p_ref, _ = follower.step(np.array([est[0], est[1], traj.p[0, 2]]), yaw_now, prm.dt)
            v_cmd = v3[:2]
        else:
            p_ref, v_ref, _, _ = traj.sample(t * time_scale)
            v_ff = v_ref[:2] * time_scale if prm.use_feedforward else 0.0
            v_cmd = v_ff + prm.kp * (p_ref[:2] - est)
        n = np.linalg.norm(v_cmd)
        if n > prm.v_sat:
            v_cmd *= prm.v_sat / n
        dv = (v_cmd - v) / prm.tau * prm.dt
        dvn = np.linalg.norm(dv)
        if dvn > prm.a_phys * prm.dt:
            dv *= prm.a_phys * prm.dt / dvn
        v = v + dv
        step = v * prm.dt
        p = p + step
        flown += np.linalg.norm(step)
        bias = bias + rng.normal(0, prm.drift_per_m * np.sqrt(np.linalg.norm(step) + 1e-12), 2)
        # --- map-relative aiding (scan matching against the known course)
        if aiding is not None and (in_dark is False or aiding.works_in_dark):
            period = max(1, int(round(1.0 / (aiding.rate_hz * prm.dt))))
            if int(round(t / prm.dt)) % period == 0:
                from .localize import raycast_scan
                yaw = traj.yaw[traj.nearest_index(np.array([p[0], p[1], traj.p[0, 2]]))]
                true_pose = np.array([p[0], p[1], yaw])
                n_beams = 64 if aiding.fov_deg <= 120 else 180
                scan = raycast_scan(aiding.scan_grid, true_pose, aiding.fov_deg, n_beams,
                                    r_max=aiding.r_max, noise=0.02, rng=rng)
                prior = np.array([p[0] + bias[0], p[1] + bias[1], yaw])
                res = aiding.matcher.match(scan, prior)
                if res.ok:
                    bias = bias + aiding.gain * (res.pose[:2] - prior[:2])
        # --- contact check
        r, c = grid.to_cell(p)
        clear = dist[r, c] - prm.true_radius
        min_clear = min(min_clear, clear)
        err = np.linalg.norm(p - p_ref[:2])
        max_err = max(max_err, err)
        if clear < 0 and not in_contact:
            touches += 1
            in_contact = True
        elif clear >= 0:
            in_contact = False
        if clear < -prm.crash_depth:
            crashed = True
            break
        t += prm.dt
        if follower is not None:
            if follower.done:
                break
        elif t >= T and np.linalg.norm(p - traj.p[-1, :2]) < 0.05:
            break
    return SimResult(time=t, touches=touches, crashed=crashed, min_clearance=float(min_clear),
                     max_track_err=float(max_err), final_err=float(np.linalg.norm(p - traj.p[-1, :2])))
