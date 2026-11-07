"""Offline test of the distance-field re-localiser along the planned route.

Synthetic forward scans (80 deg FOV, like the OAK-D mono HFOV) are ray-cast
from true poses on the planned trajectory; the prior is perturbed; we report
the corrected error and how often the health check accepts the fix.
"""
import math, os
import numpy as np
import _path  # noqa: F401
from _path import COURSE_OBJ, HERE
from dh_core.course import Course
from dh_core.route import RouteParams, plan_mission
from dh_core.localize import DistanceFieldMatcher, raycast_scan

course = Course.from_obj(COURSE_OBJ)
plan = plan_mission(course, RouteParams(route="window"))
tr = plan.legs[0].traj
import time
RES = float(os.environ.get("DH_LOC_RES", "0.025"))
t0 = time.time()
grid = course.occupancy(RES, 0.58, 1.12)   # un-inflated occupancy at cruise band
m = DistanceFieldMatcher(grid)
print("matcher build %.1fs at res %.3f" % (time.time() - t0, RES))
rng = np.random.default_rng(3)
lines = ["# Re-localiser test (synthetic 80-degree forward scans along the window route; map resolution from DH_LOC_RES)\n",
         "| prior error std | samples | accepted | median pos err before (m) | median pos err after, accepted (m) | 95th pct after (m) | median yaw err after (deg) |",
         "|---|---|---|---|---|---|---|"]
for sig_p, sig_y in ((0.10, 3.0), (0.20, 6.0), (0.30, 10.0)):
    before, after, yaw_after, acc = [], [], [], 0
    idx = np.linspace(5, len(tr.p) - 5, 80).astype(int)
    for k in idx:
        true = np.array([tr.p[k, 0], tr.p[k, 1], tr.yaw[k]])
        scan = raycast_scan(grid, true, rng=rng)
        prior = true + np.array([rng.normal(0, sig_p), rng.normal(0, sig_p), math.radians(rng.normal(0, sig_y))])
        res = m.match(scan, prior)
        before.append(np.linalg.norm(prior[:2] - true[:2]))
        if res.ok:
            acc += 1
            after.append(np.linalg.norm(res.pose[:2] - true[:2]))
            yaw_after.append(abs(math.degrees((res.pose[2] - true[2] + math.pi) % (2 * math.pi) - math.pi)))
    row = "| %.2f m / %.0f deg | %d | %d%% | %.3f | %.3f | %.3f | %.2f |" % (
        sig_p, sig_y, len(idx), 100 * acc / len(idx), np.median(before),
        np.median(after) if after else float("nan"), np.percentile(after, 95) if after else float("nan"),
        np.median(yaw_after) if yaw_after else float("nan"))
    print(row); lines.append(row)
open(os.path.join(HERE, "..", "docs", "localizer_test.md"), "w").write("\n".join(lines) + "\n")
