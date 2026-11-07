"""Experiment 3: closed-loop value of map-relative scan matching.

Compares no aiding, a forward 80-deg stereo-derived scan (blind in the dark
tube), and a 360-deg 2D LiDAR (works in the dark), using the real
DistanceFieldMatcher on synthetic scans of the official course.
"""
import os, itertools, time
import numpy as np
import _path  # noqa: F401
from _path import COURSE_OBJ, HERE
from dh_core.course import Course
from dh_core.route import RouteParams, plan_mission, dark_zones
from dh_core.sim import SimParams, ScanAiding, simulate
from dh_core.localize import DistanceFieldMatcher

N = int(os.environ.get("DH_MC", "30"))
OVERHEAD = 6.0
course = Course.from_obj(COURSE_OBJ)
fine = course.occupancy(0.025, 0.58, 1.12)
matcher = DistanceFieldMatcher(fine)
configs = {"none": None,
           "stereo80": ScanAiding(matcher, fine, fov_deg=80, works_in_dark=False),
           "lidar360": ScanAiding(matcher, fine, fov_deg=360, works_in_dark=True)}
lines = ["# Experiment 3 - closed-loop map-relative aiding (N=%d per row, PX4 cap 1.0 m/s, planned at 0.85)\n" % N,
         "| route | aiding | drift | planned time (s) | mean time incl. penalties (s) | P(touch) | P(crash) |",
         "|---|---|---|---|---|---|---|"]
t0 = time.time()
for route, (aname, aid), (dname, dprm) in itertools.product(
        ("window", "gate"), configs.items(),
        {"nominal": dict(drift_per_m=0.01, dark_vel_err=0.08),
         "pessimistic": dict(drift_per_m=0.03, dark_vel_err=0.20)}.items()):
    rp = RouteParams(route=route, v_max=0.85, v_narrow=0.5, v_dark=0.6)
    plan = plan_mission(course, rp)
    traj = plan.legs[0].traj
    dist = plan.grid.distance()
    zones = [b[:, :2] for b in dark_zones(course, rp.dark_zone_hints)]
    sp = SimParams(v_sat=1.0, **dprm)
    rng = np.random.default_rng(11)
    res = [simulate(traj, dist, plan.grid, sp, zones, rng, aiding=aid) for _ in range(N)]
    crash = np.mean([r.crashed for r in res]); touch = np.mean([r.touches > 0 for r in res])
    tm = np.mean([r.time + 5 * r.touches for r in res if not r.crashed] or [np.nan]) + OVERHEAD
    row = "| %s | %s | %s | %.1f | %.1f | %.2f | %.2f |" % (route, aname, dname, traj.duration + OVERHEAD, tm, touch, crash)
    print(row, "(%.0fs)" % (time.time() - t0), flush=True); lines.append(row)
open(os.path.join(HERE, "..", "docs", "strategy_results_aiding.md"), "w").write("\n".join(lines) + "\n")
