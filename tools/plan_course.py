"""Plan the official course (gate and window variants) and plot them.

Usage: python plan_course.py [out.png]
"""
import sys, time
import numpy as np
import _path  # noqa: F401
from _path import COURSE_OBJ
from dh_core.course import Course
from dh_core.route import RouteParams, plan_mission

out_png = sys.argv[1] if len(sys.argv) > 1 else "course_plan.png"
course = Course.from_obj(COURSE_OBJ)
plans = {}
for route in ("gate", "window"):
    t0 = time.time()
    plans[route] = plan_mission(course, RouteParams(route=route), with_return=True)
    p = plans[route]
    main = p.legs[0].traj
    print("%-6s plan %.1fs | length %.2f m | time %.1f s | min clearance %.3f m | return %.1f s | lengths %s"
          % (route, time.time() - t0, main.length, main.duration, p.info["min_clearance"],
             p.legs[1].traj.duration, {k: round(v, 2) for k, v in p.info["route_lengths"].items()}))

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    g = plans["gate"].grid
    fig, ax = plt.subplots(figsize=(8, 8))
    ext = [g.origin[0], g.origin[0] + g.shape[1] * g.res, g.origin[1], g.origin[1] + g.shape[0] * g.res]
    ax.imshow(g.occ, origin="lower", extent=ext, cmap="Greys", alpha=0.8)
    infl = plans["gate"].planner.free
    ax.imshow(~infl, origin="lower", extent=ext, cmap="Reds", alpha=0.15)
    for route, col in (("gate", "tab:blue"), ("window", "tab:green")):
        tr = plans[route].legs[0].traj
        sc = ax.scatter(tr.p[:, 0], tr.p[:, 1], c=tr.v, cmap="viridis", s=4, vmin=0, vmax=1.0)
        ax.plot(tr.p[:, 0], tr.p[:, 1], color=col, lw=0.6, label="%s: %.1f m, %.1f s" % (route, tr.length, tr.duration))
    if course.guideline is not None:
        ax.plot(course.guideline[:, 0], course.guideline[:, 1], "r--", lw=1, label="floor guideline")
    ax.plot(*course.launch[:2], "r^", ms=10, label="launch")
    ax.add_patch(plt.Circle(course.pad_center[:2], course.pad_radius, fill=False, color="k"))
    for a in course.apertures:
        ax.plot(*a.center[:2], "gs", ms=8, label="aperture (window)")
    plt.colorbar(sc, ax=ax, label="planned speed (m/s)")
    ax.set_aspect("equal"); ax.legend(loc="upper right", fontsize=8)
    ax.set_title("Auto-planned routes from the official course mesh (cruise z = 0.85 m)")
    ax.set_xlabel("X east (m)"); ax.set_ylabel("Y north (m)")
    fig.savefig(out_png, dpi=130, bbox_inches="tight")
    print("saved", out_png)
except ImportError:
    pass
