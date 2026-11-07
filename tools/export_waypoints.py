"""Export the auto-generated route as human-readable waypoints (reference data, NOT used by the flight code).

The flight code re-derives everything from the course file at start-up; this table exists for the
presentation, for sanity-checking on site with a tape measure, and as a documented last-resort fallback.
"""
import math, os
import numpy as np
import _path  # noqa: F401
from _path import COURSE_OBJ, HERE
from dh_core.course import Course
from dh_core.route import RouteParams, plan_mission

course = Course.from_obj(COURSE_OBJ)
IN = 0.0254
out = ["# Auto-generated waypoints for course v3 (reference only)\n",
       "Frame: official model frame (metres; origin = floor centre; +X east, +Y north, +Z up). "
       "`map_in` = the overhead-map frame used in the research handoff (inches from the NW corner, x east, y south). "
       "Waypoints are the corners of the shortcut path *before* corner rounding; t is the time at which the "
       "planned trajectory passes nearest to the waypoint at v_max 0.85 m/s.\n"]
for route in ("window", "gate"):
    plan = plan_mission(course, RouteParams(route=route))
    tr = plan.legs[0].traj
    pl = plan.planner
    start, goal = course.launch[:2], course.pad_center[:2]
    anchors = [np.array(a) for a in plan.info["anchors"]]
    dense = pl.plan_via([start] + anchors + [goal])
    keep = [int(np.argmin(np.linalg.norm(dense - a, axis=1))) for a in anchors]
    wps = pl.shortcut(dense, min_clearance=0.37, keep=keep)
    out += ["\n## Route: %s  (length %.2f m, planned %.1f s)\n" % (route, tr.length, tr.duration),
            "| # | X (m) | Y (m) | z (m) | map_in (x, y) | t (s) | planned speed there (m/s) | clearance (m) |",
            "|---|---|---|---|---|---|---|---|"]
    for i, w in enumerate(wps):
        k = tr.nearest_index(np.array([w[0], w[1], tr.p[0, 2]]))
        mi = ((w[0] + 3.048) / IN, (3.048 - w[1]) / IN)
        out.append("| %d | %.3f | %.3f | %.2f | (%.1f, %.1f) | %.1f | %.2f | %.3f |" % (
            i, w[0], w[1], tr.p[k, 2], mi[0], mi[1], tr.t[k], tr.v[k], pl.clearance_at(w)))
open(os.path.join(HERE, "..", "docs", "waypoints_v3.md"), "w").write("\n".join(out) + "\n")
print("\n".join(out))
