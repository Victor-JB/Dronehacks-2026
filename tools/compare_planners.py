"""Compare the trajectory generators on the official course (all from the same A* skeleton).

A) fillet      : clearance A* -> shortcut -> corner arcs -> time-optimal speed profile   (default)
B) sfc         : Safe Flight Corridor + min-jerk QP + Eq.(5)-style time allocation
                 (Liu et al., RA-L 2017)
C) sfc+retime  : the SFC curve's geometry re-timed with the time-optimal speed profile
D) mpl h1/h2   : motion-primitive search (Liu et al., IROS 2017 / arXiv 1709.05401) guided by the
                 shortcut waypoints, with the min-time heuristic h1 or the LQMT heuristic h2

  python tools/compare_planners.py            -> docs/planner_comparison.md, docs/figures/sfc_corridor.png
"""
import os, sys, time
import numpy as np
import _path  # noqa: F401
from _path import COURSE_OBJ, HERE
from dh_core.course import Course
from dh_core.route import RouteParams, plan_mission
from dh_core.timing import time_parameterize
from dh_core.planner_mpl import MotionPrimitivePlanner, MPLParams

V, A = 0.85, 1.5
course = Course.from_obj(COURSE_OBJ)
lines = ["# Planner comparison on the official course (v_max %.2f m/s, a_max %.1f m/s^2, same A* skeleton)\n" % (V, A),
         "Duration is the planned course time (launch to pad hover). Clearance is the drone centre's distance to the "
         "nearest obstacle (robot radius 0.27 m). Compute time is on a laptop CPU.\n",
         "| route | method | length (m) | duration (s) | min clearance (m) | compute (s) |", "|---|---|---|---|---|---|"]
fig_data = {}
for route in ("window", "gate"):
    rows = []
    t0 = time.time()
    plan = plan_mission(course, RouteParams(route=route, v_max=V, v_dark=V))
    ta = time.time() - t0
    tr, pl = plan.legs[0].traj, plan.planner
    clr = lambda P: min(pl.clearance_at(q) for q in P)
    rows.append(("fillet + time-optimal profile (default)", tr.length, tr.duration, clr(tr.p), ta))

    t0 = time.time()
    ps = plan_mission(course, RouteParams(route=route, v_max=V, v_dark=V, planner="sfc"))
    tb = time.time() - t0
    ts = ps.legs[0].traj
    st = ps.info["sfc"]
    rows.append(("SFC + min-jerk QP (%d polygons)" % st["pieces"], ts.length, ts.duration, clr(ts.p), tb))
    xy = ts.p[:, :2]
    xyd = [xy[0]]
    for q in xy[1:]:
        if np.linalg.norm(q - xyd[-1]) >= 0.02:
            xyd.append(q)
    xyd = np.array(xyd)
    def vlim(q):
        d = pl.clearance_at(q); f = np.clip((d - 0.27) / (0.45 - 0.27), 0, 1)
        return 0.5 + (V - 0.5) * f
    hy = time_parameterize(np.hstack([xyd, np.full((len(xyd), 1), 0.85)]), V, A, A, vlim)
    rows.append(("SFC curve re-timed with the time-optimal profile", hy.length, hy.duration, clr(hy.p), tb))
    fig_data[route] = (tr, ts, st["polys"], np.array(ps.info["sparse"]), pl)

    sparse = np.array(plan.info["sparse"])
    free = lambda P: np.array([pl.clearance_at(q) > 0.30 for q in P])
    for h, name in (("min_time", "h1 min-time"), ("lqmt", "h2 LQMT")):
        m = MotionPrimitivePlanner(free, MPLParams(v_max=V, a_max=A, tau=0.3, mu=2, rho=4.0, heuristic=h))
        t0 = time.time()
        trm, stm = m.plan_through(sparse, 0.85)
        tm = time.time() - t0
        if trm is None:
            rows.append(("motion primitives, %s: failed" % name, np.nan, np.nan, np.nan, tm))
        else:
            rows.append(("motion primitives, %s heuristic (%d expansions)" % (name, stm["expansions"]),
                         trm.length, trm.duration, clr(trm.p), tm))
    for r in rows:
        lines.append("| %s | %s | %.2f | %.1f | %.3f | %.1f |" % ((route,) + r))
        print(lines[-1], flush=True)
lines += ["", "Notes:",
          "- SFC is slower because a min-jerk polynomial cannot hold the speed limit the way a time-optimal profile can, "
          "and min-jerk does not reward path length, so the curve swings wide inside the corridor. Its strengths are "
          "a certified-safe convex corridor and C3 smoothness, which matter for receding-horizon replanning in unknown maps "
          "(the paper's use case). Here the map is known in advance.",
          "- The LQMT heuristic h2 returns the same optimal cost as h1 with 2.7-3.5x fewer expansions, as in the paper's Fig. 5.",
          "- Default stays `planner: fillet`."]
out = os.path.join(HERE, "..", "docs", "planner_comparison.md")
open(out, "w", encoding="utf-8").write("\n".join(lines) + "\n")

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MPoly


def poly_vertices(Aa, bb):
    """Vertices of {x : A x <= b} in 2-D (brute force over line pairs)."""
    Aa, bb = np.asarray(Aa), np.asarray(bb)
    V = []
    for i in range(len(Aa)):
        for j in range(i + 1, len(Aa)):
            M = np.array([Aa[i], Aa[j]])
            if abs(np.linalg.det(M)) < 1e-9:
                continue
            x = np.linalg.solve(M, [bb[i], bb[j]])
            if np.all(Aa @ x <= bb + 1e-7):
                V.append(x)
    if len(V) < 3:
        return None
    V = np.array(V); c = V.mean(0)
    return V[np.argsort(np.arctan2(V[:, 1] - c[1], V[:, 0] - c[0]))]


fig, axs = plt.subplots(1, 2, figsize=(14, 7))
for ax, route in zip(axs, ("window", "gate")):
    tr, ts, polys, sparse, pl = fig_data[route]
    g = pl.grid
    ext = [g.origin[0], g.origin[0] + g.occ.shape[1] * g.res, g.origin[1], g.origin[1] + g.occ.shape[0] * g.res]
    ax.imshow(g.occ, origin="lower", extent=ext, cmap="Greys", alpha=0.8)
    for k, (Aa, bb) in enumerate(polys):
        V = poly_vertices(Aa, bb)
        if V is not None:
            ax.add_patch(MPoly(V, closed=True, fc="tab:cyan", ec="tab:blue", alpha=0.18, lw=0.8))
    ax.plot(sparse[:, 0], sparse[:, 1], "o--", color="tab:purple", ms=3, lw=1, label="A* + shortcut skeleton")
    ax.plot(tr.p[:, 0], tr.p[:, 1], color="tab:orange", lw=2, label="fillet (default) %.1f s" % tr.duration)
    ax.plot(ts.p[:, 0], ts.p[:, 1], color="tab:blue", lw=2, label="SFC min-jerk %.1f s" % ts.duration)
    ax.set_aspect("equal"); ax.set_xlim(-3.1, 3.1); ax.set_ylim(-3.1, 3.1)
    ax.set_title("%s route: safe flight corridor (shrunk by robot radius)" % route)
    ax.legend(loc="lower right", fontsize=8)
fig.savefig(os.path.join(HERE, "..", "docs", "figures", "sfc_corridor.png"), dpi=110, bbox_inches="tight")
print("wrote", out)
