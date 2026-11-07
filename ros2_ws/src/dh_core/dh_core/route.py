"""Mission route generation from a course mesh.

Given only the course file, this module plans launch -> landing pad with the
drone kept below the wall tops, so the maze, the tunnel and the chicane are
discovered by the planner rather than typed in. Optional variants:
  route="gate"   : shortest path that does not use wall apertures
  route="window" : path forced through the best wall aperture (the window)
  route="auto"   : the cheaper of the two after a score-weighted comparison
and an optional return leg flown above the walls back to the launch mark.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from .course import Course, Grid2D
from .planner_astar import ClearancePlanner
from .timing import Trajectory, fillet_path, time_parameterize


@dataclass
class RouteParams:
    res: float = 0.05
    cruise_z: float = 0.85          # m; inside window (0.305-1.219), tube widest ~0.94
    drone_half_height: float = 0.12
    z_margin: float = 0.15
    robot_radius: float = 0.27      # m; ~half diagonal of a 7" quad incl. props + margin
    preferred_clearance: float = 0.45
    clearance_weight: float = 4.0
    v_max: float = 0.85             # planned speed; PX4 MPC_XY_VEL_MAX (bench drone) is 1.0
    a_max: float = 1.5              # m/s^2 along-track
    a_lat: float = 1.5              # m/s^2 lateral (corner speed)
    v_narrow: float = 0.5           # speed cap where clearance is tight
    v_dark: float = 0.85            # speed cap inside "dark" zones; fast = less dead-reckoning time
    fillet_r: float = 0.6
    aperture_standoff: float = 0.9  # m before/after the window plane (forced waypoints)
    window_straight: float = 0.6    # m of STRAIGHT path required on each side of the window plane
                                    # (>= v * PX4 lag + settling margin: the turn is finished before the hole)
    corner: str = "clothoid"        # clothoid (continuous curvature) | arc (classic circular fillet)
    a_total: float = 1.5            # m/s^2 combined along+lateral acceleration limit (0 = separate limits)
    route: str = "gate"             # gate | window | auto
    window_bonus_m: float = 3.0     # 'auto': prefer window unless it costs > this many metres
    return_z: float = 2.05          # m; above 1.727 m walls, below 3.0 m interface ceiling
    dark_zone_hints: tuple = ("tube", "tunnel")
    dark_axis_margin: float = 0.35  # m beyond each end of a dark zone where the axis alignment starts
    land_approach_len: float = 1.2  # m of path before the pad over which to descend
    land_approach_z: float = 0.30   # m height at the pad before handing over to PX4 landing
    planner: str = "fillet"         # fillet (A* + arcs + time-optimal profile) | sfc (safe flight corridor + min-jerk QP) | mpl (motion primitives)
    sfc_margin: float = 0.03        # sfc: corridor shrink = robot_radius + this
    j_max: float = 5.0              # sfc: jerk limit for the time allocation (m/s^3)


@dataclass
class Leg:
    name: str
    traj: Trajectory


@dataclass
class MissionPlan:
    legs: List[Leg]
    route_used: str
    grid: Grid2D
    planner: ClearancePlanner
    info: Dict = field(default_factory=dict)

    @property
    def total_time(self) -> float:
        return sum(l.traj.duration for l in self.legs)


def _block_apertures(grid: Grid2D, course: Course) -> Grid2D:
    g = Grid2D(grid.origin.copy(), grid.res, grid.occ.copy())
    for ap in course.apertures:
        a = ap.center[:2] - 0.5 * ap.width * ap.along
        b = ap.center[:2] + 0.5 * ap.width * ap.along
        g.fill_polygon(np.vstack([a, b]))
    return g


def dark_zones(course: Course, hints) -> List[np.ndarray]:
    zones = []
    for n in course.obstacles:
        if any(h in n.lower() for h in hints):
            bb = course.groups[n].bbox
            ext = bb[1] - bb[0]
            if ext[0] * ext[1] >= 0.3:          # ignore small parts such as the tube cradles
                zones.append(bb)
    return zones


def plan_mission(course: Course, prm: RouteParams = RouteParams(), with_return: bool = False,
                 start_xy: Optional[np.ndarray] = None, goal_xy: Optional[np.ndarray] = None) -> MissionPlan:
    start = np.asarray(start_xy if start_xy is not None else course.launch[:2], float)
    goal = np.asarray(goal_xy if goal_xy is not None else course.pad_center[:2], float)
    z_lo = prm.cruise_z - prm.drone_half_height - prm.z_margin
    z_hi = prm.cruise_z + prm.drone_half_height + prm.z_margin
    if z_hi >= course.wall_height:
        raise ValueError("cruise band reaches the wall tops; lower cruise_z")
    grid = course.occupancy(prm.res, z_lo, z_hi)

    usable = [ap for ap in course.apertures if ap.z_min <= z_lo and ap.z_max >= z_hi
              and ap.width > 2 * prm.robot_radius]
    gate_grid = _block_apertures(grid, course)
    gate_pl = ClearancePlanner(gate_grid, prm.robot_radius, prm.preferred_clearance, prm.clearance_weight)
    candidates = {}
    gp = gate_pl.astar(start, goal)
    if gp is not None:
        candidates["gate"] = (gp, gate_pl, [])
    if usable:
        win_pl = ClearancePlanner(grid, prm.robot_radius, prm.preferred_clearance, prm.clearance_weight)
        best = None
        for ap in usable:
            p1, p2 = ap.waypoints(prm.aperture_standoff)
            for a, b in ((p1, p2), (p2, p1)):
                path = win_pl.plan_via([start, a[:2], b[:2], goal])
                if path is not None:
                    L = np.linalg.norm(np.diff(path, axis=0), axis=1).sum()
                    if best is None or L < best[0]:
                        best = (L, path, [a[:2], b[:2]])
        if best is not None:
            candidates["window"] = (best[1], win_pl, best[2])
    if not candidates:
        raise RuntimeError("no route found from launch to pad")

    def length(k):
        return np.linalg.norm(np.diff(candidates[k][0], axis=0), axis=1).sum()

    route = prm.route
    if route == "auto":
        if "window" in candidates and "gate" in candidates:
            route = "window" if length("window") <= length("gate") + prm.window_bonus_m else "gate"
        else:
            route = next(iter(candidates))
    if route not in candidates:
        route = next(iter(candidates))
    dense, pl, anchors = candidates[route]
    window_anchors = [np.asarray(a, float) for a in anchors] if route == "window" else []

    # Fly straight down the axis of every dark zone the route crosses: inside the
    # zone visual odometry is blind, so the path should be centred and aligned
    # *before* entry (maximum lateral margin for dead-reckoning drift).
    zones = dark_zones(course, prm.dark_zone_hints)
    tagged = [(int(np.argmin(np.linalg.norm(dense - a, axis=1))), np.asarray(a, float)) for a in anchors]
    for lo, hi in zones:
        inside = np.all((dense >= lo[:2]) & (dense <= hi[:2]), axis=1)
        if not inside.any():
            continue
        ext, c = hi[:2] - lo[:2], 0.5 * (hi[:2] + lo[:2])
        axis = np.array([1.0, 0.0]) if ext[0] >= ext[1] else np.array([0.0, 1.0])
        half = 0.5 * float(max(ext)) + prm.dark_axis_margin
        ends = [c - half * axis, c + half * axis]
        idx = [int(np.argmin(np.linalg.norm(dense - e, axis=1))) for e in ends]
        tagged += sorted(zip(idx, ends), key=lambda x: x[0])
    if len(tagged) > len(anchors):
        tagged.sort(key=lambda x: x[0])
        via = pl.plan_via([start] + [a for _, a in tagged] + [goal])
        if via is not None:
            dense = via
            anchors = [a for _, a in tagged]

    # keep anchor points during shortcutting (window crossed square-on, tube along its axis)
    keep_idx = [int(np.argmin(np.linalg.norm(dense - a, axis=1))) for a in anchors]
    sparse = pl.shortcut(dense, min_clearance=prm.robot_radius + 0.10, keep=keep_idx)
    clear_ok = lambda q: pl.clearance_at(q) >= prm.robot_radius + 0.06
    # keep a straight run through the window: cap the curved part around each window waypoint
    max_tangent = [None] * len(sparse)
    for wa in window_anchors:
        k = int(np.argmin(np.linalg.norm(sparse - wa, axis=1)))
        if np.linalg.norm(sparse[k] - wa) < prm.res:          # A* snaps waypoints to cell centres
            max_tangent[k] = max(0.05, prm.aperture_standoff - prm.window_straight)
    smooth, kappa = fillet_path(sparse, prm.fillet_r, clear_ok, transition=prm.corner,
                                max_tangent=max_tangent, return_kappa=True)
    path3 = np.hstack([smooth, np.full((len(smooth), 1), prm.cruise_z)])
    _descent_ramp(course, path3, prm)

    def vlim(q):
        d = pl.clearance_at(q)
        frac = np.clip((d - prm.robot_radius) / max(prm.preferred_clearance - prm.robot_radius, 1e-6), 0, 1)
        v = prm.v_narrow + (prm.v_max - prm.v_narrow) * frac
        for lo, hi in zones:
            if lo[0] - 0.3 <= q[0] <= hi[0] + 0.3 and lo[1] - 0.3 <= q[1] <= hi[1] + 0.3:
                v = min(v, prm.v_dark)
        return v

    main, sfc_stats = None, None
    if prm.planner == "sfc":
        main, sfc_stats = _sfc_trajectory(course, pl, sparse, prm, vlim)
    elif prm.planner == "mpl":
        main, sfc_stats = _mpl_trajectory(course, pl, sparse, prm)
    if main is None:
        main = time_parameterize(path3, prm.v_max, prm.a_max, prm.a_lat, vlim, kappa=kappa,
                                 a_total=prm.a_total or None)
    legs = [Leg("course", main)]
    info = {"route_lengths": {k: float(length(k)) for k in candidates},
            "min_clearance": float(min(pl.clearance_at(q) for q in smooth)),
            "anchors": [a.tolist() for a in anchors], "planner": (prm.planner if sfc_stats and sfc_stats.get("used") else "fillet"),
            "sfc": sfc_stats, "sparse": sparse.tolist()}

    if with_return:
        legs.append(Leg("return", return_leg(np.array([goal[0], goal[1], 0.0]), start, prm)))
    return MissionPlan(legs=legs, route_used=route, grid=grid, planner=pl, info=info)


def _mpl_trajectory(course: Course, pl: ClearancePlanner, sparse: np.ndarray, prm: RouteParams):
    """Motion-primitive search (planner_mpl, LQMT heuristic) guided through the shortcut waypoints."""
    from .planner_mpl import MotionPrimitivePlanner, MPLParams
    r_free = prm.robot_radius + prm.sfc_margin
    free = lambda xy: np.array([pl.clearance_at(q) > r_free for q in xy])
    mpl = MotionPrimitivePlanner(free, MPLParams(v_max=prm.v_max, a_max=prm.a_max, tau=0.3, mu=2, rho=4.0))
    tr, st = mpl.plan_through(sparse, prm.cruise_z)
    st = dict(st) if isinstance(st, dict) else {"status": str(st)}
    if tr is None:
        st["used"] = False
        return None, st
    _descent_ramp(course, tr.p, prm)
    tr.s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(tr.p, axis=0), axis=1))])
    st["used"] = True
    return tr, st


def _sfc_trajectory(course: Course, pl: ClearancePlanner, sparse: np.ndarray, prm: RouteParams, vlim):
    """Safe-flight-corridor + min-jerk QP (planner_sfc) through the shortcut polyline, sampled into a
    Trajectory with the same descent ramp. Returns (None, stats) if the result is not clearly safe."""
    from .planner_sfc import SFCParams, plan_sfc
    sp = SFCParams(robot_r=prm.robot_radius + prm.sfc_margin, v_max=prm.v_max, a_max=prm.a_max, j_max=prm.j_max)
    vseg = []
    for a, b in zip(sparse[:-1], sparse[1:]):
        qs = [a + (b - a) * f for f in np.linspace(0, 1, 9)]
        vseg.append(min(vlim(np.array([q[0], q[1], prm.cruise_z])) for q in qs))
    traj2, polys, st = plan_sfc(sparse, pl.grid.occ, pl.grid.origin, pl.grid.res, sp, vseg)
    t = np.arange(0.0, traj2.duration + 1e-9, 0.02)
    if t[-1] < traj2.duration:
        t = np.append(t, traj2.duration)
    xy = traj2.eval(t); vxy = traj2.eval(t, 1)
    p3 = np.hstack([xy, np.full((len(xy), 1), prm.cruise_z)])
    _descent_ramp(course, p3, prm)
    clr = np.array([pl.clearance_at(q) for q in xy])
    st.update(min_clearance=float(clr.min()), polys=[(A.tolist(), b.tolist()) for A, b in polys])
    if not st["corridor_ok"] or st["corridor_violation_m"] > 0.03 or clr.min() < prm.robot_radius:
        st["used"] = False
        return None, st
    spd = np.linalg.norm(vxy, axis=1)
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(p3, axis=0), axis=1))])
    yaw = np.arctan2(vxy[:, 1], vxy[:, 0])
    for k in range(len(yaw)):                       # hold heading while (almost) stopped
        if spd[k] < 0.05:
            yaw[k] = yaw[k - 1] if k > 0 else math.atan2(*(sparse[1] - sparse[0])[::-1])
    st["used"] = True
    return Trajectory(p3, s, spd, t, yaw), st


def _descent_ramp(course: Course, path3: np.ndarray, prm: RouteParams) -> None:
    """Start descending over the last metre(s) before the pad so the final landing
    (PX4 crawls at MPC_LAND_CRWL below MPC_LAND_ALT3) starts from low height.

    The ramp is shortened until the column it sweeps is free of low obstacles
    (boulders are 0.53 m tall)."""
    if prm.land_approach_len <= 0:
        return
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(path3[:, :2], axis=0), axis=1))])
    total = s[-1]
    L = min(prm.land_approach_len, 0.8 * total)
    z_end = prm.land_approach_z
    low = course.occupancy(prm.res, max(0.02, z_end - prm.drone_half_height - 0.05), prm.cruise_z)
    dist_low = low.distance()
    while L > 0.2:
        idx = np.where(s >= total - L)[0]
        ok = all(dist_low[low.to_cell(path3[i])] > prm.robot_radius for i in idx)
        if ok:
            frac = (s[idx] - (total - L)) / L
            path3[idx, 2] = prm.cruise_z + (z_end - prm.cruise_z) * frac
            return
        L *= 0.7


def return_leg(from_xyz, to_xy, prm: RouteParams, land_z: float = 0.0) -> Trajectory:
    """Climb above the walls, fly straight over them, descend onto the target (rule 7.10 return)."""
    a = np.asarray(from_xyz, float)
    up = [a, [a[0], a[1], prm.return_z]]
    over = [[to_xy[0], to_xy[1], prm.return_z]]
    down = [[to_xy[0], to_xy[1], land_z]]
    ret = fillet_path(np.array(up + over + down, float), 0.0)  # densify only
    return time_parameterize(ret, min(prm.v_max, 0.8), min(prm.a_max, 1.0), prm.a_lat, None)
