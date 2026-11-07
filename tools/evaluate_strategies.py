"""Monte-Carlo comparison of route / speed / estimation-quality strategies.

Prints a table and writes docs/strategy_results.md. All error magnitudes are
assumptions (see SimParams); the point is the *relative* trend.
"""
import os, sys, time, itertools
import numpy as np
import _path  # noqa: F401
from _path import COURSE_OBJ, HERE
from dh_core.course import Course
from dh_core.route import RouteParams, plan_mission, dark_zones
from dh_core.sim import SimParams, simulate

N = int(os.environ.get("DH_MC", "60"))
OVERHEAD = 6.0   # s: takeoff to cruise (~1.5 s), landing + disarm (~3 s), slack
course = Course.from_obj(COURSE_OBJ)
rows = []
t0 = time.time()
drift_levels = {"nominal": dict(drift_per_m=0.01, dark_vel_err=0.08),
                "pessimistic": dict(drift_per_m=0.03, dark_vel_err=0.20)}
for route, cap, (dname, dprm), reanchor in itertools.product(
        ("gate", "window"), (0.5, 0.75, 1.0, 1.25, 1.5), drift_levels.items(), (False, True)):
    rp = RouteParams(route=route, v_max=cap, v_narrow=min(0.5, cap), v_dark=min(0.6, cap))
    plan = plan_mission(course, rp)
    traj = plan.legs[0].traj
    dist = plan.grid.distance()
    zones = [b[:, :2] for b in dark_zones(course, rp.dark_zone_hints)]
    sp = SimParams(v_sat=cap, reanchor_after_dark=reanchor, **dprm)
    rng = np.random.default_rng(42)
    res = [simulate(traj, dist, plan.grid, sp, zones, rng) for _ in range(N)]
    crash = np.mean([r.crashed for r in res])
    touch = np.mean([r.touches > 0 for r in res])
    tmean = np.mean([r.time + 5 * r.touches for r in res if not r.crashed] or [np.nan]) + OVERHEAD
    mclr = np.percentile([r.min_clearance for r in res], 10)
    rows.append((route, cap, dname, reanchor, traj.duration + OVERHEAD, tmean, touch, crash, mclr))
    print("%-6s cap %.2f %-11s reanchor=%-5s plan %.1fs sim %.1fs P(touch) %.2f P(crash) %.2f clr10 %.3f"
          % rows[-1][:4] + (rows[-1][4], rows[-1][5], touch, crash, mclr) if False else
          "%-6s cap %.2f %-11s reanchor=%-5s plan %5.1fs sim %5.1fs P(touch) %.2f P(crash) %.2f clr_p10 %+.3f"
          % (route, cap, dname, reanchor, traj.duration + OVERHEAD, tmean, touch, crash, mclr))
print("total %.0fs" % (time.time() - t0))
out = os.path.join(HERE, "..", "docs", "strategy_results.md")
with open(out, "w") as f:
    f.write("# Monte-Carlo strategy results (N=%d per row)\n\n" % N)
    f.write("Times include %.0f s takeoff/landing overhead; touched runs add 5 s per touch. "
            "clr_p10 = 10th-percentile minimum clearance between the drone's physical radius (0.24 m) and the nearest obstacle.\n\n" % OVERHEAD)
    f.write("| route | speed cap (m/s) | estimator drift | re-anchor after tunnel | planned time (s) | mean sim time (s) | P(touch) | P(crash) | clr_p10 (m) |\n|---|---|---|---|---|---|---|---|---|\n")
    for r in rows:
        f.write("| %s | %.2f | %s | %s | %.1f | %.1f | %.2f | %.2f | %+.3f |\n" % r)
print("wrote", out)
