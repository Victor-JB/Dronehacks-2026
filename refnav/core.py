"""refnav.core - the reference mission's flight logic. PURE: no ROS, no clock reads, no files.

Everything arrives through calls that carry their own time, on ONE clock chosen by whoever runs
it: simulation time in the simulator and in replays of recorded data (ROS use_sim_time + /clock),
system time on hardware. The same code therefore runs against all three.

The mission: wait until PX4 has a valid position and heading, the estimator reports healthy and
the depth camera is live -> stream a hold setpoint, request Offboard and arm -> climb -> fly a
pre-planned route of legs (each leg: turn to face it, check the depth ahead, advance a carrot at a
bounded speed while the live depth shows the way clear) -> land.

Safety behaviour (each one is a test in tests/test_refnav_core.py):
* fail closed on estimator health: absent, stale or unhealthy refuses to arm; unhealthy or silent
  for health_bad_land_s in the air lands;
* no translation without OBSERVED free space: the carrot advances only while a depth frame no older
  than depth_stale_s returned depth on at least min_corridor_coverage of the rays crossing the
  corridor ahead (no return is unknown space, never free space); a blank, sparse (valid fraction
  < min_valid_frac), future-stamped, stale-on-arrival or out-of-order frame refreshes nothing; no
  usable depth for depth_lost_land_s lands;
* an obstacle closer than stop_m ahead stops the aircraft and ends the route (it lands where it
  is: route blocked; planning around obstacles is the student's job). Optional, off by default:
  the carrot slows inside slow_m and never goes closer than stop_m to the obstacle; the aircraft
  holds up to blocked_wait_s for it to clear before ending; and anything farther than the leg's
  end plus clear_past_leg_m is ignored (the wall a corner waypoint is planned in front of);
* PX4 leaving Offboard while airborne (a pilot or a failsafe) is never fought: the mission stops
  streaming and commanding and ends ("yielded");
* PX4 owns the landing: once AUTO_LAND is observed the mission stops streaming setpoints (PX4's
  land detector reads the trajectory_setpoint topic; a position-only stream blanks its
  commanded-descent test and the aircraft never detects touchdown);
* a landing REQUEST from outside (the interactive runtime's `land`, request_land) goes through this
  same landing: airborne it lands where it is; before arming it ends without arming; while arming it
  ends with a normal checked disarm. It never adds a controller.

Mode note (competition fallback): when require_depth is False, the mission skips every depth gate
above and the obstacle-stop check in _go -- it is pure dead-reckoning off PX4's own position/heading
estimate, flying the pre-planned route blind. No "no translation without observed free space"
guarantee holds in this mode. Only ever pair it with a real, pre-verified route.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from . import depth as D

NAV_OFFBOARD = 14
NAV_AUTO_LAND = 18
AIRBORNE = ("climb", "hover", "await_start", "turn", "check", "go", "hold",
            "vault_find", "vault_approach", "vault_climb", "vault_cross", "vault_descend", "vault_settle")


@dataclass
class Params:
    record_only: bool = False            # walkthrough: never asks to fly (stays in wait); the stack records the walk
    require_depth: bool = True           # false: pure dead-reckoning on PX4 position only — no depth gating, no obstacle stop
    route: list = field(default_factory=list)   # [(forward_m, left_m)] from the takeoff point and heading
    height_m: float = 1.2
    speed_mps: float = 0.6
    lead_max_m: float = 1.0                     # the carrot never leads the aircraft by more
    yaw_rate_dps: float = 45.0
    face_tol_deg: float = 8.0
    arrive_m: float = 0.3
    hold_s: float = 1.0
    climb_tol_m: float = 0.15
    climb_rate_mps: float = 0.5
    climb_forward_m: float = 0.0                # climb to a point this far ahead of the takeoff point (0: straight up):
                                                # keeps a takeoff drift aft from carrying the props into a wall behind the mark
    hover_s: float = 0.0                        # hold at height_m this long after the climb (0: straight on)
    start_gate: bool = False                    # after the climb, hover until request_start() (the live view's START)
    start_wait_s: float = 60.0                  # no START this long after reaching the hover: end the mission (0: never)
    health_quiet_s: float = 3.0                 # healthy this long before arming
    health_stale_s: float = 1.0
    health_bad_land_s: float = 1.0
    depth_stale_s: float = 0.5
    depth_lost_land_s: float = 3.0
    stop_m: float = 0.9
    slow_m: float = 0.0                         # carrot slows when an obstacle is nearer (<= stop_m: off)
    blocked_wait_s: float = 0.0                 # hold this long at an obstacle before ending (0: end at once)
    look_around_deg: float = 0.0                # while held at an obstacle, sweep the heading +-this about the leg
                                                # (fresh views: a false match or a person moving clears)
    skip_blocked_leg: bool = False              # still blocked after blocked_wait_s: fly on to the NEXT waypoint from
                                                # where it is (that leg is checked like any other) instead of landing
    max_skips: int = 2                          # at most this many skipped legs, then a block ends the route
    clear_past_leg_m: float = math.inf          # obstacles beyond leg end + this are expected, not obstacles
    corridor_half_m: float = 0.6
    band_m: float = 0.4
    min_obstacle_px: int = 1200                 # an obstacle covers at least this many image pixels (~35 x 35):
                                                # a 5 cm pole does out to ~4 m, sparse stereo junk does not
    depth_stride: int = 8                       # every n-th pixel of the depth image is used
    min_valid_frac: float = 0.3                 # below this a frame is unusable (flight data: >= 0.76)
    min_corridor_coverage: float = 0.6          # of the corridor's rays must return depth (flight data: >= 0.86)
    depth_future_tol_s: float = 0.05            # a capture stamp later than now + this is refused
    depth_border_px: int = 24                   # image edge ignored (stereo matching is weakest there)
    confirm_frames: int = 2                     # an obstacle must be in this many consecutive frames to stop
    stream_lead_s: float = 1.0                  # setpoints stream this long before Offboard is requested
    wait_s: float = 300.0                       # nothing ready this long: give up without arming (0: never)
    arm_wait_s: float = 30.0                    # READY but not in Offboard this long: give up (0: never)
    cmd_resend_s: float = 1.0
    land_timeout_s: float = 60.0
    vault_only: bool = False                    # after climb/hover, find wall ahead (even above a bottom gap), climb over, cross, and land
    vault_on_obstacle: bool = False             # when blocked by an obstacle on a route leg, vault over it instead of stopping
    vault_detect_max_m: float = 3.5             # max distance ahead to lock onto a wall during vault_find
    vault_standoff_m: float = 1.2               # distance in front of the wall to hold before climbing over it
    vault_climb_margin_m: float = 0.45          # vertical clearance buffer above the top edge of the wall
    vault_overstep_m: float = 1.2               # distance past the wall face to fly before descending/landing
    vault_max_alt_m: float = 2.8                # maximum flight altitude ceiling (under skybeta's 3.0 m envelope)
    vault_descend_alt_m: float = 0.6            # altitude to descend to on the far side before landing (0: skip descend)
    vault_climb_rate_mps: float = 0.4           # vertical climb/descent rate during vault
    vault_speed_mps: float = 0.5                # horizontal speed during vault


@dataclass
class Vehicle:
    """The latest PX4 reports, updated by the adapter as messages arrive."""
    pos: tuple | None = None          # NED metres (vehicle_local_position)
    heading: float | None = None      # rad
    pos_ok: bool = False              # xy_valid and z_valid and a heading (good for control, or PX4 yaw-aligned)
    q: tuple | None = None            # attitude, FRD -> NED, (w, x, y, z)
    armed: bool | None = None
    nav_state: int | None = None
    landed: bool | None = None


@dataclass
class Output:
    phase: str
    setpoint: tuple | None = None     # (n, e, d, yaw) for the trajectory setpoint
    stream: bool = False              # publish offboard_control_mode + the setpoint this tick
    commands: list = field(default_factory=list)   # "offboard" | "arm" | "land" | "disarm" (checked)
    events: list = field(default_factory=list)


class Health:
    """Estimator health from a boolean stream, failing closed: nothing heard, stale or unhealthy
    is never permission, and a gap restarts the quiet window."""

    def __init__(self, quiet_s, stale_s):
        self.quiet_s, self.stale_s = float(quiet_s), float(stale_s)
        self.last_t = self.last_ok = self.ok_since = self.bad_since = None

    def on_msg(self, t, ok):
        ok = bool(ok)
        if ok:
            if not self.last_ok or self.ok_since is None or (self.last_t is not None and t - self.last_t > self.stale_s):
                self.ok_since = t
            self.bad_since = None
        else:
            self.ok_since = None
            if self.bad_since is None:
                self.bad_since = t
        self.last_t, self.last_ok = t, ok

    def fresh(self, now):
        return self.last_t is not None and now - self.last_t <= self.stale_s

    def arm_ok(self, now):
        if self.last_t is None:
            return False, "no estimator health received"
        if not self.fresh(now):
            return False, f"estimator health stale ({now - self.last_t:.1f} s)"
        if not self.last_ok:
            return False, "estimator unhealthy"
        if now - self.ok_since < self.quiet_s:
            return False, f"estimator healthy for only {now - self.ok_since:.1f}/{self.quiet_s:.0f} s"
        return True, "ok"

    def bad_for(self, now):
        if self.last_t is None:
            return math.inf
        if not self.fresh(now):
            return now - (self.last_t + self.stale_s)
        if not self.last_ok:
            return now - self.bad_since
        return 0.0


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Mission:
    def __init__(self, params: Params):
        self.p = params
        self.health = Health(params.health_quiet_s, params.health_stale_s)
        self.phase = "wait"
        self.why = ""
        self.sp = None
        self.home = None              # (n, e, d, heading) where Offboard took over (provisional before)
        self.targets = []             # NED (n, e) of each leg's end
        self.leg = 0
        self.legs_done = 0
        self.t_phase = None
        self.t_started = None         # the first time this mission saw (its clock): where a replay starts
        self.t_last = None
        self.t_cmd = None
        self.offboard_seen = False
        self.land_reason = None
        self.land_auto_seen = False
        self.result = None            # final: "complete" | "blocked" | "landed: <why>" | "refused: <why>" | "yielded: <why>"
        self.depth_ready_since = None # start of continuously usable depth before control requests
        self.depth_t = None           # capture stamp of the latest USABLE depth frame (depth alive)
        self.frame = None             # that frame (refnav.depth.DepthFrame) and its obstacle points
        self.depth_pts = None
        self.corr_t = None            # stamp of the latest frame that OBSERVED the current corridor
        self._judged_t = None         # the latest frame judged against the corridor (each frame once)
        self.last_coverage = None
        self.leg_start = None
        self.leg_dir = None
        self.leg_len = 0.0
        self.blocked_t = None         # when the current obstacle hold began (blocked_wait_s)
        self.skips = 0                # legs skipped because blocked (skip_blocked_leg)
        self.leg_heading = 0.0
        self.s = 0.0
        self.counts = {"depth_frames": 0, "obstacle_stops": 0, "depth_holds": 0, "unconfirmed_obstacles": 0,
                       "depth_unusable": 0, "depth_future": 0, "depth_stale": 0, "depth_out_of_order": 0,
                       "corridor_unobserved": 0}
        self.clr_hist = []            # [(depth stamp, clearance)] along the current leg, newest last
        self.last_clearance = None
        self.land_request = None      # why an outside landing request was made (request_land), until acted on
        self.start_requested = False  # request_start() accepted while hovering in await_start
        self.started = False          # past the start gate (or no gate): the route / vault is running
        self.depth_reject = None      # (key, detail): why the latest refused depth frame was refused
        self.last_valid_frac = None   # fraction of pixels returning depth in the latest decoded frame
        self.vault_target = None      # far-side (North, East) landing target coordinate for wall vault
        self.vault_wall_dist = None   # horizontal distance to the detected wall face during vault
        self._vault_seen_n = 0        # consecutive frames confirming the wall during vault_find
        self._vault_clear_n = 0       # consecutive frames confirming clearance above the wall during vault_climb
        self._vault_judged_t = None   # latest depth stamp evaluated by _vault

    def request_start(self):
        """The operator's START (start_gate): accepted only while hovering in await_start.
        Returns (accepted, message)."""
        if self.phase != "await_start":
            return False, f"START ignored: the mission is in {self.phase}, not hovering at the start gate"
        self.start_requested = True
        return True, "START accepted: flying the route"

    def request_land(self, why="requested"):
        """An outside request to end the flight through the mission's own landing (no second controller)."""
        if self.land_request is None:
            self.land_request = str(why) or "requested"

    def depth_refused(self, key, detail):
        """A depth frame was refused (here, or before it reached the mission: no calibration, no
        attitude, an undecodable image). The reason is what the wait reports, with its numbers."""
        self.depth_reject = (str(key), str(detail))
        # Obsolete transport packets do not contradict the newer usable stream.
        # Its existing freshness deadline still expires if current data stops.
        if key not in ("out_of_order", "stale"):
            self.depth_ready_since = None

    # ------------------------------------------------------------------ inputs
    def on_health(self, t, ok):
        self.health.on_msg(t, ok)

    def on_depth(self, stamp, depth_m, K, R_body_cam, t_body_cam, q_wxyz, now=None):
        """One depth frame with its own calibration; stamp = capture time on the mission clock, now =
        that clock at arrival (None: not judged for being from the future). -> True if it was usable."""
        if q_wxyz is None:
            self.depth_refused("attitude_missing", "no attitude for this depth frame")
            return False
        stamp = float(stamp)
        if now is not None and stamp > now + self.p.depth_future_tol_s:
            self.counts["depth_future"] += 1
            self.depth_refused("future", f"stamped {stamp - now:.2f} s in the future "
                                         f"(depth_future_tol_s {self.p.depth_future_tol_s:g} s)")
            return False
        if now is not None and now - stamp > self.p.depth_stale_s:
            self.counts["depth_stale"] += 1                  # too old to act on when it arrives
            self.depth_refused("stale", f"{now - stamp:.2f} s old on arrival (depth_stale_s {self.p.depth_stale_s:g} s)")
            return False
        if self.depth_t is not None and stamp <= self.depth_t:
            self.counts["depth_out_of_order"] += 1
            self.depth_refused("out_of_order", "stamped no later than the last frame used")
            return False
        f = D.DepthFrame(depth_m, K, R_body_cam, t_body_cam, D.level_rotation(q_wxyz), stride=self.p.depth_stride,
                         border_px=self.p.depth_border_px)
        self.last_valid_frac = f.valid_frac
        if f.valid_frac < self.p.min_valid_frac:
            self.counts["depth_unusable"] += 1
            self.depth_refused("too_little_depth", f"depth on {f.valid_frac:.0%} of the pixels, this mission needs "
                                                   f"{self.p.min_valid_frac:.0%} (min_valid_frac)")
            return False
        if (self.depth_ready_since is None or self.depth_t is None
                or stamp - self.depth_t > self.p.depth_stale_s):
            self.depth_ready_since = stamp
        self.frame, self.depth_t, self.depth_pts = f, stamp, f.points(0.3, 6.0)
        self.depth_reject = None                             # a refusal is only reported until a frame is used
        self.counts["depth_frames"] += 1
        return True

    def depth_fresh(self, now):
        """A usable depth frame is recent (the camera is alive), whatever it looked at."""
        return self.depth_t is not None and now - self.depth_t <= self.p.depth_stale_s

    def depth_ready_for_arm(self, now):
        """Same quiet interval as estimator health, measured in usable capture history.

        A rare good snapshot among unusable frames cannot authorize motors. This
        only gates the first control request; airborne/authority policy is unchanged.
        """
        return (self.depth_fresh(now) and self.depth_ready_since is not None
                and math.isfinite(self.p.health_quiet_s) and self.p.health_quiet_s >= 0
                and self.depth_t - self.depth_ready_since >= self.p.health_quiet_s)

    def corridor_fresh(self, now):
        """A recent frame OBSERVED the corridor of the current leg."""
        return self.corr_t is not None and now - self.corr_t <= self.p.depth_stale_s

    # ------------------------------------------------------------------ step
    def _set(self, phase, now, out, why=""):
        out.events.append(f"{self.phase} -> {phase}" + (f": {why}" if why else ""))
        self.phase, self.t_phase = phase, now
        if why:
            self.why = why

    def _end(self, now, out, result):
        self.result = result
        self._set("done", now, out, result)

    def _land(self, now, out, why):
        self.land_reason = why
        self._set("land", now, out, why)

    def _resend(self, now):
        if self.t_cmd is None or now - self.t_cmd >= self.p.cmd_resend_s:
            self.t_cmd = now
            return True
        return False

    def _capture_home(self, v):
        """home, setpoint and the route's targets from the vehicle's position and heading right now."""
        n, e, d = v.pos
        self.home = (n, e, d, v.heading)
        self.sp = [n, e, d, v.heading]
        c, s = math.cos(v.heading), math.sin(v.heading)
        self.targets = [(n + f * c + l * s, e + f * s - l * c) for f, l in self.p.route]

    def _requalify(self, now, out, why):
        # Cancel the old priming interval and its held position. The original
        # preflight deadline remains t_started; this cannot renew its budget.
        self.sp, self.home, self.targets = None, None, []
        self._set("wait", now, out, why)

    def step(self, now, v: Vehicle) -> Output:
        p = self.p
        out = Output(self.phase)
        dt = 0.0 if self.t_last is None else min(max(now - self.t_last, 0.0), 0.25)
        self.t_last = now
        if self.t_phase is None:
            self.t_phase = now
        if self.t_started is None:
            self.t_started = now
        if self.phase == "done":
            return out

        if self.phase in ("wait", "arm") and self.t_cmd is None:
            if v.armed is True:
                self._end(now, out, "yielded: vehicle armed before this mission requested control")
            elif p.wait_s > 0 and now - self.t_started > p.wait_s:
                self._end(now, out, f"refused: {self.why or 'not ready'} after {p.wait_s:.0f} s")
            elif self.phase == "arm" and v.armed is None:
                self._requalify(now, out, "waiting for an explicit disarmed PX4 report")

        # ---- airborne supervision, before anything else may command -------------------
        if self.phase in AIRBORNE or (self.phase == "land" and not self.land_auto_seen):
            # only the mission's OWN landing may put PX4 in AUTO_LAND; anywhere else that is PX4 (a
            # failsafe) or a pilot taking over, and streaming on would blank PX4's land detector
            allowed = (NAV_OFFBOARD, NAV_AUTO_LAND) if self.phase == "land" else (NAV_OFFBOARD,)
            touched_down = self.phase == "land" and v.armed is False    # disarmed on the floor: not a takeover
            if self.offboard_seen and v.nav_state is not None and v.nav_state not in allowed and not touched_down:
                self._end(now, out, f"yielded: PX4 left Offboard (nav_state {v.nav_state}): a pilot or a failsafe has it")
                out.phase = self.phase
                return out
        if self.land_request is not None and self.phase in ("wait", "arm") + AIRBORNE:
            why = f"landing requested: {self.land_request}"
            if self.phase == "wait":
                self._end(now, out, f"refused: {why} before arming")
            elif self.phase == "arm":
                if v.armed:
                    out.commands.append("disarm")          # a normal, checked disarm: on the ground, never forced
                self._end(now, out, f"refused: {why} while arming")
            else:
                self._land(now, out, why)
            self.land_request = None
            out.phase = self.phase
        if self.phase in AIRBORNE:
            bad = self.health.bad_for(now)
            if bad > p.health_bad_land_s:
                self._land(now, out, f"estimator unhealthy or silent for {bad:.1f} s")
            elif not v.pos_ok or v.pos is None:
                self._land(now, out, "PX4 position or heading invalid")
            elif p.require_depth and (self.depth_t is None or now - self.depth_t > p.depth_lost_land_s):
                self._land(now, out, f"depth silent for more than {p.depth_lost_land_s:.0f} s")

        ph = self.phase
        if ph == "wait":
            ok, why = self.health.arm_ok(now)
            if p.record_only:
                self.why = "walkthrough mode: recording only, this mode never asks to fly"
            elif p.wait_s > 0 and now - self.t_started > p.wait_s:
                self._end(now, out, f"refused: {self.why or 'not ready'} after {p.wait_s:.0f} s")
            elif v.armed is not False:
                self.why = "waiting for an explicit disarmed PX4 report"
            elif not v.pos_ok or v.pos is None or v.heading is None:
                self.why = "waiting for PX4 position and heading"
            elif not ok:
                self.why = why
            elif p.require_depth and not self.depth_fresh(now):
                self.why = "waiting for live depth: " + (
                    f"the latest frame was refused: {self.depth_reject[1]}" if self.depth_reject else
                    "no depth frame has arrived" if self.depth_t is None else
                    f"the latest usable frame is {now - self.depth_t:.1f} s old")
            elif p.require_depth and not self.depth_ready_for_arm(now):
                self.why = f"waiting for {p.health_quiet_s:g} s of continuously usable depth"
            else:
                self._capture_home(v)
                self._set("arm", now, out)
        elif ph == "arm":
            ok, why = self.health.arm_ok(now)
            # as test_files: until Offboard takes over, the setpoint is wherever the vehicle is now
            # (moving it after READY does not leave a stale target behind)
            if v.pos is not None and v.heading is not None and not (v.armed and v.nav_state == NAV_OFFBOARD):
                n, e, d = v.pos
                self.sp = [n, e, d, v.heading]
            if not ok:
                if self.t_cmd is None and v.armed is False:
                    self._requalify(now, out, f"{why}; requalifying before any control request")
                else:
                    self._end(now, out, f"refused: {why} before arming")
            elif self.t_cmd is None and p.require_depth and not self.depth_ready_for_arm(now):
                if v.armed is False:
                    self._requalify(now, out, "depth not continuously usable; requalifying before any control request")
                else:
                    self._end(now, out, "yielded: depth lost before this mission requested control")
            elif p.arm_wait_s > 0 and now - self.t_phase > p.arm_wait_s:
                if v.armed:
                    out.commands.append("disarm")          # a normal, checked disarm: on the ground, never forced
                self._end(now, out, f"refused: not armed in Offboard after {p.arm_wait_s:.0f} s")
            elif v.armed and v.nav_state == NAV_OFFBOARD:
                self.offboard_seen = True
                self._capture_home(v)                     # the route starts where Offboard took over
                if p.climb_forward_m:
                    hn, he, _, hh = self.home
                    self.sp[0] = hn + p.climb_forward_m * math.cos(hh)
                    self.sp[1] = he + p.climb_forward_m * math.sin(hh)
                self._set("climb", now, out)
            elif now - self.t_phase >= p.stream_lead_s and self._resend(now):
                out.commands += ["offboard", "arm"]
        elif ph == "climb":
            target = self.home[2] - p.height_m
            self.sp[2] = max(target, self.sp[2] - p.climb_rate_mps * dt)
            if self.sp[2] <= target + 1e-9 and abs(v.pos[2] - target) < p.climb_tol_m:
                if p.hover_s > 0:
                    self._set("hover", now, out, f"holding {p.hover_s:.0f} s at {p.height_m:.1f} m")
                else:
                    self._next_leg(now, out, v)
        elif ph == "hover":
            if now - self.t_phase >= p.hover_s:
                self._next_leg(now, out, v)
        elif ph == "await_start":
            if self.start_requested:
                self.started = True
                self._next_leg(now, out, v)
            elif p.start_wait_s > 0 and now - self.t_phase > p.start_wait_s:
                self._land(now, out, f"no START within {p.start_wait_s:.0f} s")
        elif ph == "turn":
            err = _wrap(self.leg_heading - self.sp[3])
            step = math.radians(p.yaw_rate_dps) * dt
            self.sp[3] = _wrap(self.sp[3] + max(-step, min(step, err)))
            if abs(err) < 1e-3 and abs(_wrap(self.leg_heading - v.heading)) < math.radians(p.face_tol_deg):
                self._set("check", now, out)
        elif ph == "check":
            if not p.require_depth:
                self.s = 0.0
                self._set("go", now, out)
            # a depth frame captured AFTER the turn ended looks along this leg
            elif self.depth_t is not None and self.depth_t >= self.t_phase and self.depth_fresh(now):
                if self.clr_hist and self.clr_hist[0][0] < self.t_phase:
                    self.clr_hist, self.corr_t = [], None    # frames from before the turn ended look elsewhere
                clr = self._clearance(v)                     # None: not enough observed frames yet, wait
                if clr is not None and self.corridor_fresh(now):
                    self.last_clearance = clr
                    if clr < p.stop_m:
                        self._blocked(now, out, v, clr, "at leg")
                    else:
                        self.blocked_t = None
                        self.sp[3] = self.leg_heading
                        self.s = 0.0
                        self._set("go", now, out)
        elif ph == "go":
            self._go(now, dt, out, v)
        elif ph in ("vault_find", "vault_approach", "vault_climb", "vault_cross", "vault_descend", "vault_settle"):
            self._vault(now, dt, out, v)
        elif ph == "hold":
            if now - self.t_phase >= p.hold_s:
                self.legs_done += 1
                self.leg += 1
                self._next_leg(now, out, v)

        if self.phase == "land":
            if v.nav_state == NAV_AUTO_LAND:
                self.land_auto_seen = True
            if not self.land_auto_seen and self._resend(now):
                out.commands.append("land")
            # down: disarmed after PX4's own AUTO_LAND, or after the adapter's own descent in Offboard
            # (skybeta has no land command: dronehacks_nav lands with a downward velocity setpoint)
            if (self.land_auto_seen or self.offboard_seen) and v.armed is False:
                why = self.land_reason or ""
                self._end(now, out, "complete" if why in ("route complete", "vault complete") else f"landed: {why}")
            elif now - self.t_phase > p.land_timeout_s:
                self._end(now, out, f"landed: {self.land_reason}; landing not confirmed within {p.land_timeout_s:.0f} s")

        out.phase = self.phase
        out.setpoint = tuple(self.sp) if self.sp is not None else None
        # PX4 owns the landing once AUTO_LAND is seen: nothing streams from then on
        out.stream = self.phase in ("arm",) + AIRBORNE or (self.phase == "land" and not self.land_auto_seen)
        return out

    def _start_vault(self, now, out, phase="vault_find", dist=None, why="initiating wall vault"):
        self.vault_wall_dist = dist
        self._vault_seen_n = 0
        self._vault_clear_n = 0
        self._vault_judged_t = None
        self.last_clearance, self._judged_t = dist, None
        self._set(phase, now, out, why)

    def _next_leg(self, now, out, v):
        p = self.p
        if (p.start_gate and not self.started and self.phase in ("climb", "hover")
                and (self.targets or p.vault_only)):
            self._set("await_start", now, out, f"hovering at {p.height_m:.2f} m: waiting for START "
                      + (f"(ends after {p.start_wait_s:.0f} s)" if p.start_wait_s > 0 else ""))
            return
        if self.p.vault_only and self.phase in ("climb", "hover", "await_start"):
            self.leg_heading = self.home[3] if self.home is not None else v.heading
            self._start_vault(now, out, "vault_find", None, "climbing to detect wall above bottom gap")
            return
        if self.leg >= len(self.targets):
            self._land(now, out, "route complete")
            return
        start = (self.sp[0], self.sp[1])
        tn, te = self.targets[self.leg]
        dn, de = tn - start[0], te - start[1]
        self.leg_len = math.hypot(dn, de)
        self.leg_start = start
        self.leg_dir = (dn / self.leg_len, de / self.leg_len) if self.leg_len > 1e-6 else (1.0, 0.0)
        self.leg_heading = math.atan2(de, dn) if self.leg_len > 1e-6 else self.sp[3]
        self.clr_hist, self.corr_t = [], None
        self.blocked_t = None
        self._set("turn", now, out, f"leg {self.leg + 1}/{len(self.targets)}")

    def _clearance(self, v):
        """Clearance along the leg CONFIRMED by the last confirm_frames frames that OBSERVED the
        corridor: the largest of their clearances, so an obstacle counts only when every one of
        those frames shows it (a stereo false match lasts a frame; a wall does not go away). A frame
        whose rays through the corridor (at stop_m + lead_max_m ahead) mostly returned nothing has
        observed nothing there and is not evidence of free space. None = unknown (do not move)."""
        p = self.p
        bearing = _wrap(self.leg_heading - v.heading)        # positive = to the right (NED yaw)
        if self.frame is not None and self._judged_t != self.depth_t:
            self._judged_t = self.depth_t
            cov, _ = self.frame.corridor_coverage(bearing, p.corridor_half_m, p.band_m, p.stop_m + p.lead_max_m)
            self.last_coverage = cov
            if cov >= p.min_corridor_coverage:
                hits = max(3, round(p.min_obstacle_px / p.depth_stride ** 2))
                c = D.clearance(self.depth_pts, bearing, p.corridor_half_m, p.band_m, hits)
                self.clr_hist = (self.clr_hist + [(self.depth_t, c)])[-max(1, p.confirm_frames):]
                self.corr_t = self.depth_t
            else:
                self.counts["corridor_unobserved"] += 1
        if len(self.clr_hist) < self.p.confirm_frames:
            return None
        confirmed = max(c for _, c in self.clr_hist)
        if min(c for _, c in self.clr_hist) < self.p.stop_m <= confirmed:
            self.counts["unconfirmed_obstacles"] += 1
        if math.isfinite(p.clear_past_leg_m) and v.pos is not None:
            n0, e0 = self.leg_start
            proj = (v.pos[0] - n0) * self.leg_dir[0] + (v.pos[1] - e0) * self.leg_dir[1]
            if confirmed > max(0.0, self.leg_len - proj) + p.clear_past_leg_m:
                return math.inf                          # past the leg's end: planned geometry, not in the way
        return confirmed

    def _blocked(self, now, out, v, clr, where):
        """An obstacle nearer than stop_m: hold where the aircraft is for up to blocked_wait_s
        (it may clear: a person walking through), then end the route (or vault over it if
        vault_on_obstacle is enabled). True = ended/vaulting."""
        p = self.p
        self.sp[0], self.sp[1] = v.pos[0], v.pos[1]
        if self.blocked_t is None:
            self.blocked_t = now
            self.counts["obstacle_stops"] += 1
        if now - self.blocked_t >= p.blocked_wait_s:
            if (p.skip_blocked_leg and not p.vault_on_obstacle and self.skips < p.max_skips
                    and self.leg + 1 < len(self.targets)):
                self.skips += 1
                self.sp[3] = self.leg_heading
                self._set("hold", now, out, f"leg {self.leg + 1} blocked ({clr:.2f} m) for {p.blocked_wait_s:g} s: "
                          f"skipping to waypoint {self.leg + 2} (skip {self.skips}/{p.max_skips})")
                self.leg += 1
                self._next_leg(now, out, v)
                return True
            if p.vault_on_obstacle:
                next_ph = "vault_approach" if clr > p.vault_standoff_m + 0.05 else "vault_climb"
                self._start_vault(now, out, next_ph, clr, f"wall detected at {clr:.2f} m, initiating vault")
            else:
                self._land(now, out, f"route blocked: obstacle {clr:.2f} m ahead {where} {self.leg + 1}")
            return True
        if p.look_around_deg > 0:                           # look around while held: fresh views of the corridor
            a = math.radians(p.look_around_deg)
            period = max(2.0, 4 * a / math.radians(p.yaw_rate_dps))
            self.sp[3] = _wrap(self.leg_heading + a * math.sin(2 * math.pi * (now - self.blocked_t) / period))
        self.why = f"holding: obstacle {clr:.2f} m ahead, {now - self.blocked_t:.1f}/{p.blocked_wait_s:g} s"
        return False

    def _go(self, now, dt, out, v):
        p = self.p
        n0, e0 = self.leg_start
        un, ue = self.leg_dir
        proj = (v.pos[0] - n0) * un + (v.pos[1] - e0) * ue
        if not p.require_depth:
            self.s = min(self.s + p.speed_mps * dt, self.leg_len, max(proj, 0.0) + p.lead_max_m)
        else:
            clr = self._clearance(v) if self.depth_fresh(now) else None
            if clr is not None and not self.corridor_fresh(now):
                clr = None                                       # the latest evidence of this corridor is too old
            if clr is not None:
                self.last_clearance = clr
                slowing = p.slow_m > p.stop_m
                if clr < p.stop_m or (slowing and clr < p.stop_m + 0.05):
                    # stop where the aircraft is (not where the carrot was); inside the slow zone, a
                    # carrot that can no longer advance 5 cm is a stop too, or it would creep forever
                    self._blocked(now, out, v, clr, "on leg")
                    return
                self.blocked_t = None
                self.sp[3] = self.leg_heading                    # back to facing the leg after a look-around
                speed, lead = p.speed_mps, p.lead_max_m
                if slowing and clr < p.slow_m:
                    speed *= (clr - p.stop_m) / (p.slow_m - p.stop_m)
                    lead = min(lead, clr - p.stop_m)         # the carrot never enters the stop zone
                    self.s = min(self.s, max(proj, 0.0) + lead)
                self.s = min(self.s + speed * dt, self.leg_len, max(proj, 0.0) + lead)
            else:
                self.counts["depth_holds"] += 1                  # no live evidence: the carrot waits
        self.sp[0], self.sp[1] = n0 + un * self.s, e0 + ue * self.s
        tn, te = self.targets[self.leg]
        if self.s >= self.leg_len - 1e-6 and math.hypot(v.pos[0] - tn, v.pos[1] - te) < p.arrive_m:
            self._set("hold", now, out, f"arrived leg {self.leg + 1}")

    def _vault_wall_distance(self, v, upper_band_m=None):
        """Distance to the nearest obstacle in the forward corridor from the latest depth frame.
        When upper_band_m is None, uses +-band_m around the aircraft's current height; otherwise
        checks z_down in [-upper_band_m, +band_m] so an elevated wall above a bottom gap can also
        be seen while climbing."""
        p = self.p
        if self.depth_pts is None or len(self.depth_pts) == 0:
            return math.inf
        ref_h = self.leg_heading if self.leg_heading is not None else v.heading
        bearing = _wrap(ref_h - v.heading)
        c, s = math.cos(bearing), math.sin(bearing)
        pts = self.depth_pts
        along = pts[:, 0] * c + pts[:, 1] * s
        across = -pts[:, 0] * s + pts[:, 1] * c
        z_hi = p.band_m
        z_lo = -p.band_m if upper_band_m is None else -float(upper_band_m)
        # Ignore floor returns when scanning: anything within 0.25 m of home[2] (ground) is floor
        if self.home is not None and v.pos is not None:
            floor_down = (self.home[2] - 0.25) - v.pos[2]
            z_hi = min(z_hi, floor_down)
        sel = (along > 0.3) & (np.abs(across) < p.corridor_half_m) & (pts[:, 2] >= z_lo) & (pts[:, 2] <= z_hi)
        hits = max(3, round(p.min_obstacle_px / p.depth_stride ** 2))
        d = np.sort(along[sel])
        return float(d[hits - 1]) if len(d) >= hits else math.inf

    def _vault(self, now, dt, out, v):
        """Maneuver to identify a wall in front of the aircraft (even when there is an empty gap
        beneath the wall at low altitude), climb until above the wall's top edge, fly forward over
        it, descend, and land on the far side."""
        p = self.p
        ph = self.phase
        ref_heading = self.leg_heading if self.leg_heading is not None else v.heading
        self.sp[3] = ref_heading
        ceiling_d = self.home[2] - p.vault_max_alt_m
        new_frame = self.depth_fresh(now) and self.depth_t is not None and self._vault_judged_t != self.depth_t

        if ph == "vault_find":
            # Hold horizontal position and climb out of any bottom gap until the wall is confirmed
            # in the horizontal band ahead of the aircraft.
            if new_frame:
                self._vault_judged_t = self.depth_t
                d_band = self._vault_wall_distance(v)
                self.last_clearance = d_band
                if d_band <= p.vault_detect_max_m:
                    self._vault_seen_n += 1
                    self.vault_wall_dist = d_band if self.vault_wall_dist is None else min(self.vault_wall_dist, d_band)
                else:
                    self._vault_seen_n = 0
            if self._vault_seen_n >= max(1, p.confirm_frames):
                d_w = self.vault_wall_dist
                self.sp[2] = v.pos[2]
                if d_w > p.vault_standoff_m + 0.15:
                    self._set("vault_approach", now, out,
                              f"wall identified {d_w:.2f} m ahead at height {self.home[2] - v.pos[2]:.2f} m; approaching to {p.vault_standoff_m:.2f} m")
                else:
                    self._vault_clear_n = 0
                    self._set("vault_climb", now, out,
                              f"wall identified {d_w:.2f} m ahead at height {self.home[2] - v.pos[2]:.2f} m; climbing above wall")
                return
            if self.sp[2] <= ceiling_d and abs(v.pos[2] - ceiling_d) < p.climb_tol_m:
                self._land(now, out, "abort: no wall found below maximum flight ceiling")
                return
            self.sp[2] = max(ceiling_d, self.sp[2] - p.vault_climb_rate_mps * dt)

        elif ph == "vault_approach":
            # Advance at the height where the wall was seen until within vault_standoff_m of the wall.
            if not self.depth_fresh(now):
                self.counts["depth_holds"] += 1
                return
            if new_frame:
                self._vault_judged_t = self.depth_t
                d_band = self._vault_wall_distance(v)
                self.last_clearance = d_band
                if math.isfinite(d_band):
                    self.vault_wall_dist = d_band
            d_w = self.vault_wall_dist
            if d_w is not None and d_w <= p.vault_standoff_m:
                self.sp[0], self.sp[1] = v.pos[0], v.pos[1]
                self._vault_clear_n = 0
                self._set("vault_climb", now, out, f"standoff reached ({d_w:.2f} m), climbing above wall")
                return
            un, ue = math.cos(ref_heading), math.sin(ref_heading)
            step = p.vault_speed_mps * dt
            # Keep carrot within lead_max_m of vehicle
            lead_n = (self.sp[0] - v.pos[0]) * un + (self.sp[1] - v.pos[1]) * ue
            if lead_n < p.lead_max_m:
                self.sp[0] += un * step
                self.sp[1] += ue * step

        elif ph == "vault_climb":
            # Climb vertically in place until the wall at vault_wall_dist drops below vault_climb_margin_m
            # for confirm_frames consecutive depth frames.
            self.sp[0], self.sp[1] = v.pos[0], v.pos[1]
            if new_frame:
                self._vault_judged_t = self.depth_t
                hits = max(3, round(p.min_obstacle_px / p.depth_stride ** 2))
                bearing = _wrap(ref_heading - v.heading)
                c, s = math.cos(bearing), math.sin(bearing)
                pts = self.depth_pts if self.depth_pts is not None else np.empty((0, 3))
                if len(pts):
                    along = pts[:, 0] * c + pts[:, 1] * s
                    across = -pts[:, 0] * s + pts[:, 1] * c
                    dist_max = (self.vault_wall_dist or p.vault_standoff_m) + 0.6
                    # Any obstacle point near the wall that is above +vault_climb_margin_m (down) means
                    # we have not yet cleared the top of the wall by vault_climb_margin_m.
                    in_wall_band = ((along > 0.25) & (along < dist_max)
                                    & (np.abs(across) < p.corridor_half_m)
                                    & (pts[:, 2] < p.vault_climb_margin_m)
                                    & (pts[:, 2] > -p.band_m))
                    blocking = int(in_wall_band.sum()) >= hits
                    # Update wall distance if still visible in the band or below us
                    wall_col = (along > 0.25) & (along < dist_max) & (np.abs(across) < p.corridor_half_m)
                    if int(wall_col.sum()) >= hits:
                        self.vault_wall_dist = float(np.sort(along[wall_col])[hits - 1])
                else:
                    blocking = False
                self.last_clearance = self._vault_wall_distance(v)
                if not blocking:
                    self._vault_clear_n += 1
                else:
                    self._vault_clear_n = 0

            if self._vault_clear_n >= max(1, p.confirm_frames):
                # Hold the cleared altitude (clamped to ceiling_d) and traverse forward over the wall
                self.sp[2] = max(ceiling_d, v.pos[2])
                cross_dist = (self.vault_wall_dist or p.vault_standoff_m) + p.vault_overstep_m
                un, ue = math.cos(ref_heading), math.sin(ref_heading)
                self.vault_target = (v.pos[0] + un * cross_dist, v.pos[1] + ue * cross_dist)
                self._set("vault_cross", now, out,
                          f"cleared wall top at height {self.home[2] - v.pos[2]:.2f} m; crossing {cross_dist:.2f} m forward")
                return
            if self.sp[2] <= ceiling_d and abs(v.pos[2] - ceiling_d) < p.climb_tol_m:
                self._land(now, out, "abort: wall exceeds maximum flight ceiling")
                return
            self.sp[2] = max(ceiling_d, self.sp[2] - p.vault_climb_rate_mps * dt)

        elif ph == "vault_cross":
            if not self.depth_fresh(now):
                self.counts["depth_holds"] += 1
                return
            tn, te = self.vault_target
            dn, de = tn - v.pos[0], te - v.pos[1]
            dist_remaining = math.hypot(dn, de)
            if dist_remaining < p.arrive_m:
                self.sp[0], self.sp[1] = tn, te
                if p.vault_descend_alt_m > 0:
                    self._set("vault_descend", now, out,
                              f"crossed wall; descending to {p.vault_descend_alt_m:.2f} m before landing")
                else:
                    self._set("vault_settle", now, out, "arrived at far side, stabilizing before landing")
            else:
                un, ue = dn / dist_remaining, de / dist_remaining
                step_sp = math.hypot(tn - self.sp[0], te - self.sp[1])
                lead = (self.sp[0] - v.pos[0]) * un + (self.sp[1] - v.pos[1]) * ue
                mv = min(step_sp, p.vault_speed_mps * dt, max(0.0, p.lead_max_m - lead))
                self.sp[0] += un * mv
                self.sp[1] += ue * mv

        elif ph == "vault_descend":
            if self.vault_target:
                self.sp[0], self.sp[1] = self.vault_target[0], self.vault_target[1]
            target_d = self.home[2] - p.vault_descend_alt_m
            self.sp[2] = min(target_d, self.sp[2] + p.vault_climb_rate_mps * dt)
            if self.sp[2] >= target_d - 1e-9 and abs(v.pos[2] - target_d) < p.climb_tol_m:
                self._set("vault_settle", now, out, "descended on far side, stabilizing before landing")

        elif ph == "vault_settle":
            if self.vault_target:
                self.sp[0], self.sp[1] = self.vault_target[0], self.vault_target[1]
            if now - self.t_phase >= 1.0:
                self._land(now, out, "vault complete")

    def status(self, now):
        return {"phase": self.phase, "why": self.why, "start_gate": self.p.start_gate,
                "start_wait_left_s": (round(max(0.0, self.p.start_wait_s - (now - self.t_phase)), 1)
                                      if self.phase == "await_start" and self.p.start_wait_s > 0 else None), "result": self.result, "started_t": self.t_started, "leg": self.leg,
                "legs": len(self.targets), "legs_done": self.legs_done, "skips": self.skips, "home": self.home,
                "targets": [list(t) for t in self.targets], "setpoint": list(self.sp) if self.sp else None,
                "vault_target": list(self.vault_target) if self.vault_target else None,
                "vault_wall_dist": None if self.vault_wall_dist is None else round(self.vault_wall_dist, 2),
                "clearance_m": (None if self.last_clearance is None or math.isinf(self.last_clearance)
                                else round(self.last_clearance, 2)),
                "depth_age_s": None if self.depth_t is None else round(now - self.depth_t, 3),
                "corridor_age_s": None if self.corr_t is None else round(now - self.corr_t, 3),
                "corridor_coverage": None if self.last_coverage is None else round(self.last_coverage, 2),
                "depth_valid_frac": None if self.last_valid_frac is None else round(self.last_valid_frac, 3),
                "depth_reject": None if self.depth_reject is None else self.depth_reject[1],
                "health_bad_s": (None if math.isinf(self.health.bad_for(now)) else round(self.health.health_bad_s if hasattr(self.health, "health_bad_s") else self.health.bad_for(now), 2)),
                **self.counts}
