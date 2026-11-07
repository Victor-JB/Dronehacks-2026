"""dronehacks_nav.motion_check - does commanding the vehicle actually move it? PURE (no ROS, no clock).

Two checks over a sliding window of window_s, fed once per tick while Offboard is ours:

* response: whenever the setpoint has stayed more than err_m from the vehicle for the whole window,
  the vehicle must have moved at least min_progress_m TOWARD it. Catches motors/props that produce no
  motion and a VIO that reports the same pose whatever happens. A setpoint the vehicle follows with a
  steady lag (a moving carrot) passes: what counts is displacement, not the gap closing.
* height: the odometry's height change must agree with PX4's within z_mismatch_m. PX4's height comes
  from the downward rangefinder (EKF2_HGT_REF=2), independent of VIO, so a disagreement is VIO drift or
  a jump. Flying over an object taller than z_mismatch_m also trips it.

All positions are ENU metres; frames need only be consistent within one call. The first failure latches
until reset().
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class Sample:
    t: float
    sp: tuple
    pos: tuple
    z_px4: float | None


class MotionCheck:
    def __init__(self, err_m=0.3, window_s=3.0, min_progress_m=0.1, z_mismatch_m=0.3):
        self.err_m, self.window_s = float(err_m), float(window_s)
        self.min_progress_m, self.z_mismatch_m = float(min_progress_m), float(z_mismatch_m)
        self.reset()

    def reset(self):
        self.samples = []
        self.failure = None

    def step(self, now, sp, pos, z_px4=None):
        """One tick: setpoint and odometry position (ENU), PX4's height if known. -> failure or None."""
        if self.failure is not None:
            return self.failure
        self.samples.append(Sample(float(now), tuple(sp), tuple(pos), None if z_px4 is None else float(z_px4)))
        # keep the newest sample at or before the window's start, and everything after it
        while len(self.samples) > 1 and self.samples[1].t <= now - self.window_s:
            self.samples.pop(0)
        a, b = self.samples[0], self.samples[-1]
        if b.t - a.t < self.window_s:
            return None                                   # not a full window yet

        if all(math.dist(s.sp, s.pos) > self.err_m for s in self.samples):
            gap = [p - q for p, q in zip(a.sp, a.pos)]
            n = math.hypot(*gap)
            moved = sum((p - q) * g / n for p, q, g in zip(b.pos, a.pos, gap))
            if moved < self.min_progress_m:
                self.failure = (f"commanded {math.dist(b.sp, b.pos):.2f} m away for {b.t - a.t:.1f} s but the "
                                f"vehicle moved {moved:+.2f} m toward it (needs {self.min_progress_m:.2f})")
                return self.failure

        if a.z_px4 is not None and b.z_px4 is not None:
            dz_vio, dz_px4 = b.pos[2] - a.pos[2], b.z_px4 - a.z_px4
            if abs(dz_vio - dz_px4) > self.z_mismatch_m:
                self.failure = (f"over {b.t - a.t:.1f} s VIO height changed {dz_vio:+.2f} m but PX4 "
                                f"(rangefinder) {dz_px4:+.2f} m")
                return self.failure
        return None
