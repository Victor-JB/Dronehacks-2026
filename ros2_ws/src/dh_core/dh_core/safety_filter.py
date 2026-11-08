"""Stage 4 independent safety filter (pure Python + numpy; no ROS, no I/O, no publishers).

The evaluator answers one question: given what the sensors say right now, how fast may the drone
move toward the obstacle ahead of it? It is deliberately independent of the planner and the tracker,
and it fails closed: anything missing, stale, invalid or inconsistent gives UNKNOWN with 0 m/s.

Verdicts
    SAFE     requested speed is allowed as asked
    LIMIT    allowed, but only up to `max_speed` (< requested)
    STOP     known hazard or known-bad localization: `max_speed` is 0
    UNKNOWN  cannot be assessed (missing / stale / invalid data): `max_speed` is 0

Stopping model (worst case along the direction of travel)
    d_stop(v) = v * latency + v^2 / (2 * brake_decel)
    usable    = free_distance - vehicle_radius - uncertainty_margin - k_std * pos_std
    allowed v = largest v with d_stop(v) <= usable

`brake_decel` is a configurable, CONSERVATIVE real-world figure. The simulator's 3 m/s^2 acceleration
cap is NOT a guarantee about the real airframe and must not be used as the default here.
`latency` is the whole command -> motion-change chain (pose latency + PX4 velocity-loop lag + our
loop period). Both values must be measured on the real drone before any flight use (see
docs/STAGE4_SAFETY.md); the defaults are placeholders chosen to be pessimistic.

Convention for the depth-derived free distance (REP 117, same as health.ForwardClearanceMonitor):
    finite >= 0 : nearest obstacle in the swept corridor ahead, metres
    +inf        : valid measurement, nothing within the sensor's reliable range
    NaN / <0    : invalid
+inf is never treated as unlimited: free space beyond `reliable_range` was not observed.

`SafetyGate` applies a verdict to a `tracker.TrajectoryFollower` and, crucially, hard-clamps the
follower's OUTPUT. `speed_scale` alone is not enough (see the gate's docstring).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

SAFE, LIMIT, STOP, UNKNOWN = "SAFE", "LIMIT", "STOP", "UNKNOWN"

# reason codes
R_OK = "OK"
R_BRAKING_LIMIT = "BRAKING_LIMIT"
R_NO_ROOM = "NO_STOPPING_ROOM"
R_DEPTH_MISSING = "DEPTH_MISSING"
R_DEPTH_STALE = "DEPTH_STALE"
R_DEPTH_INVALID = "DEPTH_INVALID"
R_DEPTH_COVERAGE = "DEPTH_COVERAGE_LOW"
R_ODOM_MISSING = "ODOM_MISSING"
R_ODOM_STALE = "ODOM_STALE"
R_ODOM_INVALID = "ODOM_INVALID"
R_TRANSFORM_INVALID = "TRANSFORM_INVALID"
R_LOC_UNKNOWN = "LOCALIZATION_UNKNOWN"
R_LOC_LOST = "LOCALIZATION_LOST"
R_LOC_DEGRADED = "LOCALIZATION_DEGRADED"
R_CMD_INVALID = "COMMAND_INVALID"
R_CONFIG_INVALID = "CONFIG_INVALID"

# localization modes (health.py): OK, DEGRADED, HOVER, ABORT
_LOC_BAD = {"HOVER", "ABORT"}


@dataclass
class SafetyParams:
    vehicle_radius: float = 0.27        # m, half diagonal incl. props (route.RouteParams.robot_radius)
    uncertainty_margin: float = 0.10    # m, fixed margin for depth range error / calibration
    k_pos_std: float = 2.0              # extra margin = k * reported odometry position std
    latency_s: float = 0.40             # s, command -> velocity change (pessimistic placeholder; MEASURE)
    brake_decel: float = 1.0            # m/s^2 guaranteed braking (pessimistic placeholder; MEASURE)
    v_max: float = 1.0                  # m/s hard speed ceiling (organizer interface caps twist at 1.5)
    stop_speed: float = 0.05            # m/s below which an allowed speed counts as STOP
    reliable_range: float = 3.0         # m, depth is trusted only this far ahead
    depth_timeout_s: float = 0.30       # s, depth older than this (stamp or receipt) is stale
    odom_timeout_s: float = 0.40        # s
    min_valid_fraction: float = 0.50    # fraction of the corridor's depth pixels that must be valid
    degraded_margin: float = 0.15       # m, extra margin while localization is DEGRADED
    degraded_speed_cap: float = 0.5     # m/s cap while localization is DEGRADED
    future_tol_s: float = 0.02          # stamps this far ahead of `now` are invalid clock data

    def validate(self) -> Optional[str]:
        vals = (self.vehicle_radius, self.uncertainty_margin, self.k_pos_std, self.latency_s, self.brake_decel,
                self.v_max, self.stop_speed, self.reliable_range, self.depth_timeout_s, self.odom_timeout_s,
                self.min_valid_fraction, self.degraded_margin, self.degraded_speed_cap)
        if not all(math.isfinite(v) for v in vals):
            return "non-finite parameter"
        if self.brake_decel <= 0 or self.v_max <= 0 or self.latency_s < 0 or self.reliable_range <= 0:
            return "non-positive braking/speed/range or negative latency"
        if min(self.vehicle_radius, self.uncertainty_margin, self.k_pos_std, self.degraded_margin) < 0:
            return "negative margin"
        return None


@dataclass
class SensorInputs:
    """Everything the evaluator needs, as plain numbers. All times are seconds on ONE clock."""
    now: float
    free_distance: Optional[float] = None   # m along the travel direction (see module docstring)
    depth_stamp: Optional[float] = None     # capture time of the depth frame
    depth_rx: Optional[float] = None        # time this node received it
    depth_valid_fraction: float = 1.0       # valid pixels / pixels in the swept corridor
    odom_stamp: Optional[float] = None
    odom_rx: Optional[float] = None
    pos_std: Optional[float] = None         # m, odometry position std if reported
    transform_valid: bool = True            # depth->body->odom transform available and finite
    loc_mode: Optional[str] = None          # health.LocalizationHealth mode: OK/DEGRADED/HOVER/ABORT


@dataclass
class Verdict:
    verdict: str
    max_speed: float
    reason: str
    stop_distance: float = math.nan      # at the requested (or ceiling) speed
    usable_distance: float = math.nan

    @property
    def allows_motion(self) -> bool:
        return self.verdict in (SAFE, LIMIT) and self.max_speed > 0.0


def stopping_distance(v: float, latency_s: float, brake_decel: float) -> float:
    """Distance covered before the vehicle is at rest if the hazard is noticed now."""
    v = max(0.0, float(v))
    return v * latency_s + v * v / (2.0 * brake_decel)


def max_speed_for_distance(usable: float, latency_s: float, brake_decel: float) -> float:
    """Largest v with stopping_distance(v) <= usable (closed form of the quadratic)."""
    if not usable > 0.0:
        return 0.0
    return max(0.0, brake_decel * (-latency_s + math.sqrt(latency_s * latency_s + 2.0 * usable / brake_decel)))


def _age_ok(stamp, rx, now, timeout, future_tol) -> Tuple[bool, str]:
    """(ok, why) for a stamp/receipt pair. why in {'missing','invalid','stale',''}."""
    if stamp is None or rx is None:
        return False, "missing"
    vals = (stamp, rx, now)
    if not all(isinstance(x, (int, float)) and math.isfinite(x) for x in vals):
        return False, "invalid"
    if stamp <= 0.0 or stamp > now + future_tol or rx > now + future_tol or rx < stamp - future_tol:
        return False, "invalid"                       # unstamped, from the future, or received before sent
    if now - stamp > timeout or now - rx > timeout:
        return False, "stale"
    return True, ""


class SafetyEvaluator:
    def __init__(self, prm: SafetyParams = SafetyParams()):
        self.prm = prm

    def evaluate(self, inp: SensorInputs, requested_speed: Optional[float] = None) -> Verdict:
        """`requested_speed` = speed (m/s) the controller wants toward the hazard; None = judge against v_max."""
        p = self.prm
        bad = p.validate()
        if bad:
            return Verdict(UNKNOWN, 0.0, R_CONFIG_INVALID)
        if requested_speed is not None and not (isinstance(requested_speed, (int, float))
                                                and math.isfinite(requested_speed)):
            return Verdict(UNKNOWN, 0.0, R_CMD_INVALID)
        v_req = p.v_max if requested_speed is None else float(np.clip(requested_speed, 0.0, p.v_max))

        # ---- localization (known-bad first: it is a definite STOP, not just unknown)
        if inp.loc_mode is None:
            return Verdict(UNKNOWN, 0.0, R_LOC_UNKNOWN)
        if inp.loc_mode in _LOC_BAD:
            return Verdict(STOP, 0.0, R_LOC_LOST)
        if inp.loc_mode not in ("OK", "DEGRADED"):
            return Verdict(UNKNOWN, 0.0, R_LOC_UNKNOWN)       # unrecognised mode string: never assume OK

        ok, why = _age_ok(inp.odom_stamp, inp.odom_rx, inp.now, p.odom_timeout_s, p.future_tol_s)
        if not ok:
            return Verdict(UNKNOWN, 0.0, {"missing": R_ODOM_MISSING, "invalid": R_ODOM_INVALID,
                                          "stale": R_ODOM_STALE}[why])
        if inp.pos_std is not None and (not math.isfinite(inp.pos_std) or inp.pos_std < 0):
            return Verdict(UNKNOWN, 0.0, R_ODOM_INVALID)
        if inp.transform_valid is not True:
            return Verdict(UNKNOWN, 0.0, R_TRANSFORM_INVALID)

        # ---- depth
        ok, why = _age_ok(inp.depth_stamp, inp.depth_rx, inp.now, p.depth_timeout_s, p.future_tol_s)
        if not ok:
            return Verdict(UNKNOWN, 0.0, {"missing": R_DEPTH_MISSING, "invalid": R_DEPTH_INVALID,
                                          "stale": R_DEPTH_STALE}[why])
        d = inp.free_distance
        if d is None:
            return Verdict(UNKNOWN, 0.0, R_DEPTH_MISSING)
        if not isinstance(d, (int, float)) or math.isnan(d) or d < 0.0:
            return Verdict(UNKNOWN, 0.0, R_DEPTH_INVALID)
        vf = inp.depth_valid_fraction
        if not isinstance(vf, (int, float)) or not math.isfinite(vf) or not (0.0 <= vf <= 1.0):
            return Verdict(UNKNOWN, 0.0, R_DEPTH_INVALID)
        if vf < p.min_valid_fraction:
            return Verdict(UNKNOWN, 0.0, R_DEPTH_COVERAGE)   # mostly no-return pixels: unknown is not free

        # ---- budget
        free = min(float(d), p.reliable_range)               # +inf -> only the reliable range was observed
        margin = p.vehicle_radius + p.uncertainty_margin
        if inp.pos_std is not None:
            margin += p.k_pos_std * inp.pos_std
        cap = p.v_max
        degraded = inp.loc_mode == "DEGRADED"
        if degraded:
            margin += p.degraded_margin
            cap = min(cap, p.degraded_speed_cap)
        usable = free - margin
        v_allowed = min(cap, max_speed_for_distance(usable, p.latency_s, p.brake_decel))
        sd = stopping_distance(min(v_req, cap), p.latency_s, p.brake_decel)

        if v_allowed < p.stop_speed:
            return Verdict(STOP, 0.0, R_NO_ROOM, sd, usable)
        if v_allowed >= v_req - 1e-9:
            return Verdict(SAFE, v_req, R_OK, sd, usable)
        reason = R_LOC_DEGRADED if (degraded and v_allowed >= cap - 1e-9) else R_BRAKING_LIMIT
        return Verdict(LIMIT, v_allowed, reason, sd, usable)


class SafetyGate:
    """Applies verdicts to a TrajectoryFollower and clamps its output.

    Why a gate is needed: `TrajectoryFollower.step` multiplies only the reference clock and the
    feed-forward term by `speed_scale`. The position-error P term (kp * e) is added afterwards and is
    limited only by the fixed `v_cap`, so with speed_scale = 0 the follower still commands up to
    `v_cap` toward a reference that is ahead of the drone (queued motion), and with 0.5 it can still
    command full speed. The gate therefore (1) sets `speed_scale` so the reference clock does not run
    away, and (2) hard-limits the command that is actually sent.
    """

    def __init__(self, follower, evaluator: Optional[SafetyEvaluator] = None):
        self.follower = follower
        self.ev = evaluator or SafetyEvaluator()
        self.last: Verdict = Verdict(UNKNOWN, 0.0, R_DEPTH_MISSING)      # fail closed before the first update

    def update(self, inp: SensorInputs, v_ref_speed: Optional[float] = None) -> Verdict:
        """Evaluate and set follower.speed_scale. v_ref_speed = current reference speed (m/s), if known."""
        want = v_ref_speed if v_ref_speed is not None else self.ev.prm.v_max
        self.last = self.ev.evaluate(inp, requested_speed=want)
        if self.last.max_speed <= 0.0:
            scale = 0.0
        else:
            scale = float(np.clip(self.last.max_speed / max(want, 1e-6), 0.0, 1.0))
        self.follower.speed_scale = scale
        return self.last

    def clamp(self, v_cmd, yaw_rate: float = 0.0) -> Tuple[np.ndarray, float]:
        """Limit the follower's output to the last verdict. Never raises the command.

        STOP/UNKNOWN -> zero velocity (all axes) and zero yaw rate. LIMIT -> the 3-D speed is capped
        at max_speed (conservative: vertical motion could also approach a hazard). Non-finite commands
        become zero. A verdict is required: before the first update() the gate stops.
        """
        v = np.asarray(v_cmd, float).copy()
        if v.shape != (3,) or not np.all(np.isfinite(v)) or not math.isfinite(yaw_rate):
            return np.zeros(3), 0.0
        vmax = self.last.max_speed if self.last.allows_motion else 0.0
        if vmax <= 0.0:
            return np.zeros(3), 0.0
        n = float(np.linalg.norm(v))
        if n > vmax:
            v *= vmax / n
        return v, float(yaw_rate)

    def step(self, p_map, yaw_map: float, dt: float, inp: SensorInputs):
        """follower.step with the gate applied. Returns (v_cmd, yaw_rate, p_ref, err, verdict)."""
        # use the reference speed one step ahead as the request: the follower will command about this
        p_ref, v_ref, _, _ = self.follower.traj.sample(self.follower.tau)
        verdict = self.update(inp, float(np.linalg.norm(v_ref)))
        v_cmd, yaw_rate, p_ref, err = self.follower.step(np.asarray(p_map, float), yaw_map, dt)
        v_cmd, yaw_rate = self.clamp(v_cmd, yaw_rate)
        return v_cmd, yaw_rate, p_ref, err, verdict
