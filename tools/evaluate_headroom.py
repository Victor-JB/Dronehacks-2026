"""Experiment 2: speed headroom, tunnel aiding quality and re-anchoring at a fixed PX4 cap.

PX4 caps horizontal speed (MPC_XY_VEL_MAX, 1.0 m/s on the bench drone). If the
plan already flies at the cap, the tracker has no authority left to correct
errors. This sweeps the planned-speed fraction.
"""
import os, itertools
import numpy as np
import _path  # noqa: F401
from _path import COURSE_OBJ, HERE
from dh_core.course import Course
from dh_core.route import RouteParams, plan_mission, dark_zones
from dh_core.sim import SimParams, simulate

N = int(os.environ.get("DH_MC", "60"))
OVERHEAD = 6.0
course = Course.from_obj(COURSE_OBJ)
cap = float(os.environ.get("DH_CAP", "1.0"))
lines = ["# Experiment 2 - planned-speed headroom below the PX4 cap (cap = %.2f m/s, N=%d)\n" % (cap, N),
         "dark_vel_err = velocity error while visual odometry is lost in the tube (0.08 = no aiding, 0.03 = e.g. IMU fusion / ToF aiding).\n",
         "| route | plan fraction | dark vel err | re-anchor | planned time (s) | mean time incl. penalties (s) | P(touch) | P(crash) |",
         "|---|---|---|---|---|---|---|---|"]
for route, frac, dve, re in itertools.product(("gate", "window"), (1.0, 0.85, 0.7), (0.08, 0.03), (False, True)):
    v = cap * frac
    rp = RouteParams(route=route, v_max=v, v_narrow=min(0.5, v), v_dark=min(0.6, v))
    plan = plan_mission(course, rp)
    traj = plan.legs[0].traj
    dist = plan.grid.distance()
    zones = [b[:, :2] for b in dark_zones(course, rp.dark_zone_hints)]
    sp = SimParams(v_sat=cap, dark_vel_err=dve, reanchor_after_dark=re)
    rng = np.random.default_rng(7)
    res = [simulate(traj, dist, plan.grid, sp, zones, rng) for _ in range(N)]
    crash = np.mean([r.crashed for r in res]); touch = np.mean([r.touches > 0 for r in res])
    tm = np.mean([r.time + 5 * r.touches for r in res if not r.crashed] or [np.nan]) + OVERHEAD
    row = "| %s | %.2f | %.2f | %s | %.1f | %.1f | %.2f | %.2f |" % (route, frac, dve, re, traj.duration + OVERHEAD, tm, touch, crash)
    print(row); lines.append(row)
open(os.path.join(HERE, "..", "docs", "strategy_results_headroom.md"), "w").write("\n".join(lines) + "\n")
