"""PX4-like multirotor + estimator model for local_sim.

Replaces the old kinematic FakePX4 (which slid toward each setpoint at <= 1 m/s with no forces, so
the aircraft never tilted and never overshot). This models:

  airframe     rigid-body translation, thrust along the body -z axis, first-order attitude and
               motor response, linear + quadratic drag, Ornstein-Uhlenbeck turbulence, flat-floor
               ground contact. ~2 kg, thrust/weight 2 (X500-class with Jetson + OAK-D).
  controller   PX4 multicopter cascade with PX4's DEFAULT gains: position P (MPC_XY_P 0.95,
               MPC_Z_P 1.0) -> velocity PID (MPC_XY_VEL_P/I/D_ACC 1.8/0.4/0.2, MPC_Z_VEL_* 4/2/0)
               -> acceleration -> thrust vector, tilt-limited (MPC_TILTMAX_AIR). Position-only
               Offboard setpoints get NO velocity feed-forward, exactly as refnav sends them.
  modes        Offboard needs a setpoint stream first; arm; AUTO_LAND descends at MPC_LAND_SPEED;
               auto-disarm after touchdown (COM_DISARM_LAND).
  estimator    what PX4/the mission believe vs. truth: launch placement error (position + heading;
               the whole hardcoded route rotates with the heading error), VIO drift as a random
               walk per metre flown, heading drift, and jitter. The controller and the mission
               only ever see the estimate; collisions are judged on truth.

All numbers are assumptions, named below, so they can be argued with. None of this is PX4 or
cuVSLAM itself -- it is a model of their first-order behaviour.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from refnav import core as C

G = 9.81
GVEC = np.array([0.0, 0.0, G])      # NED: gravity points +d


@dataclass
class QuadParams:
    mass: float = 2.0                 # kg
    tw_ratio: float = 2.0             # max thrust / weight
    drag_lin: float = 0.35            # N per m/s (rotor induced drag dominates at low speed)
    drag_quad: float = 0.12           # N per (m/s)^2
    tau_att: float = 0.10             # s, attitude (thrust-direction) response
    tau_motor: float = 0.03           # s
    yaw_rate_max: float = math.radians(200)
    tilt_max: float = math.radians(35)
    # PX4 defaults
    xy_p: float = 0.95
    z_p: float = 1.0
    xy_vel_p: float = 1.8
    xy_vel_i: float = 0.4
    xy_vel_d: float = 0.2
    z_vel_p: float = 4.0
    z_vel_i: float = 2.0
    xy_vel_max: float = 12.0
    z_vel_max_up: float = 3.0
    z_vel_max_dn: float = 1.5
    land_speed: float = 0.7
    disarm_after_land_s: float = 2.0
    # disturbances
    turb_force_n: float = 0.12        # 1-sigma turbulence force
    turb_tau_s: float = 1.0


@dataclass
class EstimatorParams:
    place_xy_sigma: float = 0.02               # m, how precisely the drone sits on the launch mark
    place_yaw_sigma: float = math.radians(1.5)  # rad, heading placement error
    drift_per_sqrt_m: float = 0.02             # m per sqrt(m flown), per axis (~0.5% of 17 m RMS)
    yaw_drift_per_sqrt_m: float = math.radians(0.15)
    jitter_m: float = 0.004


def rot_from_thrust_yaw(t_dir, yaw):
    """NED <- FRD rotation whose -z (thrust) axis is t_dir and whose heading is yaw."""
    zb = -t_dir / np.linalg.norm(t_dir)
    xc = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    yb = np.cross(zb, xc)
    yb /= np.linalg.norm(yb)
    xb = np.cross(yb, zb)
    return np.column_stack([xb, yb, zb])


def quat_from_R(R):
    """(w, x, y, z) for a rotation matrix (FRD -> NED, PX4 convention)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return (0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s)
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    if i == 0:
        s = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return ((R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s)
    if i == 1:
        s = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s)
    s = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s)


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class PX4Quad:
    """Same interface as the old FakePX4 (apply / vehicle), plus truth accessors.

    Frames: WORLD = NED aligned with the course (n = course +y), origin at the launch MARK.
    ESTIMATE = PX4's local frame: origin where the drone actually sat, heading 0 = the way it
    actually faced (cuVSLAM starts at identity; no compass indoors)."""

    def __init__(self, rng, qp=None, ep=None, noise=True):
        self.qp, self.ep, self.rng = qp or QuadParams(), ep or EstimatorParams(), rng
        ep_ = self.ep
        self.noise = noise
        self.place = np.array([rng.normal(0, ep_.place_xy_sigma), rng.normal(0, ep_.place_xy_sigma), 0.0]) if noise else np.zeros(3)
        self.yaw0 = rng.normal(0, ep_.place_yaw_sigma) if noise else 0.0
        self.p = self.place.copy()                 # truth, world NED
        self.v = np.zeros(3)
        self.yaw = self.yaw0                       # truth heading, world
        self.t_dir = np.array([0.0, 0.0, -1.0])    # thrust direction (world), up
        self.thrust = 0.0
        self.R = rot_from_thrust_yaw(self.t_dir, self.yaw)
        self.a_prev = np.zeros(3)
        self.integ = np.zeros(3)
        self.turb = np.zeros(3)
        self.bias = np.zeros(3)                    # estimator position bias (estimate frame)
        self.yaw_bias = 0.0
        self.armed, self.nav, self.landed = False, 4, True
        self.stream_since = None
        self.land_hold = None
        self.touch_t = None
        self.sp = None                             # last offboard setpoint (estimate frame)
        self.dist_flown = 0.0

    # ---------------------------------------------------------------- estimate <-> world
    @property
    def phi(self):                                 # estimate-frame heading offset vs world
        return self.yaw0 - self.yaw_bias

    def est_pos(self):
        return Rz(-self.yaw0) @ (self.p - self.place) + self.bias

    def est_vel(self):
        return Rz(-self.yaw0) @ self.v

    def est_yaw(self):
        return wrap(self.yaw - self.phi)

    def est_R(self):
        return Rz(-self.phi) @ self.R

    # ---------------------------------------------------------------- control + physics
    def _control(self, dt):
        qp = self.qp
        pe, ve = self.est_pos(), self.est_vel()
        hover = qp.mass * G
        if self.nav == C.NAV_AUTO_LAND:
            if self.land_hold is None:
                self.land_hold = pe.copy()
            e = self.land_hold - pe
            v_sp = np.array([qp.xy_p * e[0], qp.xy_p * e[1], qp.land_speed])
            yaw_sp = self.est_yaw()
        else:
            n, e_, d, yaw_sp = self.sp
            err = np.array([n, e_, d]) - pe
            v_sp = np.array([qp.xy_p * err[0], qp.xy_p * err[1], qp.z_p * err[2]])
        h = np.hypot(v_sp[0], v_sp[1])
        if h > qp.xy_vel_max:
            v_sp[:2] *= qp.xy_vel_max / h
        v_sp[2] = min(max(v_sp[2], -qp.z_vel_max_up), qp.z_vel_max_dn)
        ev = v_sp - ve
        kp = np.array([qp.xy_vel_p, qp.xy_vel_p, qp.z_vel_p])
        ki = np.array([qp.xy_vel_i, qp.xy_vel_i, qp.z_vel_i])
        kd = np.array([qp.xy_vel_d, qp.xy_vel_d, 0.0])
        self.integ = np.clip(self.integ + ev * dt, -2.0, 2.0)
        a_sp_e = kp * ev + ki * self.integ - kd * (Rz(-self.yaw0) @ self.a_prev)
        a_sp = Rz(self.yaw0) @ a_sp_e                      # back to world
        F = qp.mass * (a_sp - GVEC)
        Fn = np.linalg.norm(F)
        t_des = F / Fn if Fn > 1e-6 else np.array([0, 0, -1.0])
        hz = -t_des[2]
        hxy = math.hypot(t_des[0], t_des[1])
        tilt = math.atan2(hxy, hz)
        if tilt > qp.tilt_max:
            s = math.sin(qp.tilt_max) / hxy
            t_des = np.array([t_des[0] * s, t_des[1] * s, -math.cos(qp.tilt_max)])
        T_cmd = min(max(Fn, 0.05 * hover), qp.tw_ratio * hover)
        yaw_sp_w = wrap(yaw_sp + self.phi)
        return t_des, T_cmd, yaw_sp_w

    def _physics(self, t_des, T_cmd, yaw_sp_w, dt):
        qp = self.qp
        self.t_dir = self.t_dir + (t_des - self.t_dir) * min(1.0, dt / qp.tau_att)
        self.t_dir /= np.linalg.norm(self.t_dir)
        self.thrust += (T_cmd - self.thrust) * min(1.0, dt / qp.tau_motor)
        ey = wrap(yaw_sp_w - self.yaw)
        self.yaw = wrap(self.yaw + max(-qp.yaw_rate_max * dt, min(qp.yaw_rate_max * dt, ey * min(1.0, dt / 0.15))))
        self.R = rot_from_thrust_yaw(self.t_dir, self.yaw)
        if self.noise:
            a_ = dt / qp.turb_tau_s
            self.turb += -self.turb * a_ + self.rng.normal(0, qp.turb_force_n * math.sqrt(2 * a_), 3) * np.array([1, 1, 0.5])
        sp = np.linalg.norm(self.v)
        drag = -(qp.drag_lin + qp.drag_quad * sp) * self.v
        a = (self.thrust * self.t_dir + drag + self.turb) / qp.mass + GVEC
        on_ground = self.p[2] >= 0.0
        if on_ground and a[2] > 0:                          # floor pushes back
            a = np.zeros(3)
            self.v[:] = 0.0
        self.v += a * dt
        step = self.v * dt
        self.p += step
        if self.p[2] > 0.0:
            self.p[2] = 0.0
            self.v[2] = min(self.v[2], 0.0)
            self.v[:2] *= 0.0
        self.a_prev = a
        ds = float(np.hypot(step[0], step[1]))
        self.dist_flown += ds
        if self.noise and ds > 0:
            ep = self.ep
            self.bias[:2] += self.rng.normal(0, ep.drift_per_sqrt_m * math.sqrt(ds), 2)
            self.yaw_bias += self.rng.normal(0, ep.yaw_drift_per_sqrt_m * math.sqrt(ds))

    def apply(self, t, out, dt, substeps=10):
        if out.stream:
            self.stream_since = t if self.stream_since is None else self.stream_since
        else:
            self.stream_since = None
        if out.setpoint is not None:
            self.sp = out.setpoint
        for cmd in out.commands:
            if cmd == "offboard" and self.stream_since is not None and t - self.stream_since >= 0.5:
                self.nav = C.NAV_OFFBOARD
            elif cmd == "arm" and self.landed:
                self.armed = True
            elif cmd == "land":
                self.nav = C.NAV_AUTO_LAND
            elif cmd == "disarm" and self.landed:
                self.armed = False
        h = dt / substeps
        for k in range(substeps):
            tk = t + k * h
            flying = self.armed and ((self.nav == C.NAV_OFFBOARD and self.sp is not None) or self.nav == C.NAV_AUTO_LAND)
            if flying:
                t_des, T_cmd, yaw_sp = self._control(h)
            else:
                t_des, T_cmd, yaw_sp = np.array([0, 0, -1.0]), 0.0, self.yaw
                self.integ[:] = 0
            self._physics(t_des, T_cmd, yaw_sp, h)
            ground = self.p[2] >= -0.02 and np.linalg.norm(self.v) < 0.25
            self.landed = ground and (not self.armed or self.thrust < 0.95 * self.qp.mass * G or self.nav == C.NAV_AUTO_LAND)
            if self.armed and self.nav == C.NAV_AUTO_LAND and ground:
                self.touch_t = tk if self.touch_t is None else self.touch_t
                if tk - self.touch_t >= self.qp.disarm_after_land_s:
                    self.armed = False
            else:
                self.touch_t = None

    def vehicle(self):
        pe = self.est_pos()
        if self.noise:
            pe = pe + self.rng.normal(0, self.ep.jitter_m, 3)
        return C.Vehicle(pos=tuple(pe), heading=self.est_yaw(), pos_ok=True, q=quat_from_R(self.est_R()),
                         armed=self.armed, nav_state=self.nav, landed=self.landed)


class KinematicPX4:
    """The original FakePX4 behaviour, kept for comparison (model="kinematic")."""

    def __init__(self, pos=(0.0, 0.0, 0.0), yaw=0.0):
        self.p = np.array(pos, float)
        self.yaw = yaw
        self.v = np.zeros(3)
        self.armed, self.nav, self.landed = False, 4, True
        self.stream_since = None
        self.place, self.yaw0, self.bias, self.yaw_bias = np.zeros(3), 0.0, np.zeros(3), 0.0

    @property
    def R(self):
        return rot_from_thrust_yaw(np.array([0, 0, -1.0]), self.yaw)

    def est_pos(self):
        return self.p.copy()

    def est_yaw(self):
        return self.yaw

    def apply(self, t, out, dt):
        if out.stream:
            self.stream_since = t if self.stream_since is None else self.stream_since
        else:
            self.stream_since = None
        for cmd in out.commands:
            if cmd == "offboard" and self.stream_since is not None and t - self.stream_since >= 0.5:
                self.nav = C.NAV_OFFBOARD
            elif cmd == "arm":
                self.armed = True
            elif cmd == "land":
                self.nav = C.NAV_AUTO_LAND
        p0 = self.p.copy()
        if self.armed and self.nav == C.NAV_OFFBOARD and out.setpoint is not None:
            n, e, d, yaw = out.setpoint
            step = np.array([n, e, d]) - self.p
            dist = np.linalg.norm(step)
            self.p += step * min(1.0, 1.0 * dt / dist) if dist > 1e-9 else 0
            err = wrap(yaw - self.yaw)
            self.yaw += max(-math.radians(60) * dt, min(math.radians(60) * dt, err))
            self.landed = self.p[2] > -0.05
        elif self.nav == C.NAV_AUTO_LAND and self.armed:
            self.p[2] = min(0.0, self.p[2] + 0.7 * dt)
            if self.p[2] >= 0.0:
                self.landed, self.armed = True, False
        self.v = (self.p - p0) / dt

    def vehicle(self):
        return C.Vehicle(pos=tuple(self.p), heading=self.yaw, pos_ok=True, q=quat_from_R(self.R),
                         armed=self.armed, nav_state=self.nav, landed=self.landed)
