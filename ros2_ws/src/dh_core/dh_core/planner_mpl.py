"""Search-based motion planning with motion primitives (simplified MPL, 2-D).

Re-implementation (numpy only) of the core idea of
  S. Liu et al., "Search-based Motion Planning for Quadrotors using Linear
  Quadratic Minimum Time Control", IROS 2017, and the motion_primitive_library
  (github.com/sikang/motion_primitive_library, Apache-2.0):
  * state s = (p, v); control u = acceleration held constant for tau seconds
  * control set U = {-a_max .. a_max} discretised with (2*mu+1)^2 samples
  * edge cost (|u|^2 + rho) * tau  -> minimises J = int |u|^2 dt + rho T
  * heuristic (default 'lqmt'): the paper's Linear Quadratic Minimum Time bound (Prop. 3), here for a
    position goal with FREE arrival velocity: C(T) = 3|dp - v0 T|^2 / T^3 + rho T minimised over
    T >= T_bar = |dp|_inf / v_max (roots of rho T^4 - 3|v0|^2 T^2 + 12 (dp.v0) T - 9|dp|^2). It is a
    relaxation of the same cost, hence admissible, and much tighter than 'min_time'
    (rho * T_bar, the paper's h1).
  * every primitive is collision-checked against the inflated occupancy grid

To keep Python run-time small the search is run segment-by-segment between the
waypoints of the geometric (A*) path (a "guided" MPL), chaining end states, so
each search covers 1-3 m. The output is a dynamically feasible piecewise
constant-acceleration trajectory (C1 velocity), directly usable as a
Trajectory for the follower.
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .timing import Trajectory


@dataclass
class MPLParams:
    tau: float = 0.3
    a_max: float = 1.5
    v_max: float = 0.85
    mu: int = 2                  # (2mu+1)^2 = 25 controls
    rho: float = 4.0             # time weight
    pos_res: float = 0.05
    goal_tol: float = 0.20
    max_expansions: int = 20000
    heuristic: str = "lqmt"     # lqmt (h2, Prop. 3) | min_time (h1)
    norm_limit: bool = True     # |u| <= a_max (same total-acceleration budget as the other planners)


def lqmt_free_end_cost(d: np.ndarray, v0: np.ndarray, rho: float, t_bar: float) -> float:
    """min over T >= t_bar of  3|d - v0 T|^2 / T^3 + rho T  (double integrator, free final velocity)."""
    dd, dv, vv = float(d @ d), float(d @ v0), float(v0 @ v0)
    if dd < 1e-12:
        return rho * t_bar
    C = lambda T: 3.0 * (dd - 2 * dv * T + vv * T * T) / T ** 3 + rho * T
    cands = [max(t_bar, 1e-3)]
    for r in np.roots([rho, 0.0, -3.0 * vv, 12.0 * dv, -9.0 * dd]):
        if abs(r.imag) < 1e-9 and r.real > cands[0]:
            cands.append(float(r.real))
    return min(C(T) for T in cands)


class MotionPrimitivePlanner:
    def __init__(self, free_fn, prm: MPLParams = MPLParams()):
        """free_fn(xy (N,2)) -> bool array, True where the drone centre is collision-free."""
        self.free = free_fn
        self.prm = prm
        g = np.linspace(-prm.a_max, prm.a_max, 2 * prm.mu + 1)
        self.U = np.array([(ax, ay) for ax in g for ay in g
                           if not prm.norm_limit or math.hypot(ax, ay) <= prm.a_max * (1 + 1e-9)])
        self.v_res = prm.a_max * prm.tau / prm.mu

    def _key(self, p, v):
        r = self.prm.pos_res
        return (int(round(p[0] / r)), int(round(p[1] / r)),
                int(round(v[0] / self.v_res)), int(round(v[1] / self.v_res)))

    def _h(self, p, goal, v=None):
        prm = self.prm
        d = goal - p
        nd = float(np.linalg.norm(d))
        if nd > 1e-9:                                   # goal is a disc of radius goal_tol
            d = d * max(0.0, 1.0 - prm.goal_tol / nd)
        t_bar = float(np.max(np.abs(d))) / prm.v_max
        if prm.heuristic != "lqmt" or v is None:
            return prm.rho * t_bar
        return lqmt_free_end_cost(d, np.asarray(v, float), prm.rho, t_bar)

    def plan_segment(self, p0, v0, goal, v_goal_max: float = None):
        prm = self.prm
        tau = prm.tau
        ts = np.linspace(0, tau, 6)[1:]
        start = (np.asarray(p0, float), np.asarray(v0, float))
        heap = [(self._h(start[0], goal, start[1]), 0.0, 0, start, None)]
        parents = {}
        best_g = {self._key(*start): 0.0}
        cnt = 0
        uid = 1
        nodes = {0: (start, None, None)}
        while heap and cnt < prm.max_expansions:
            f, gc, nid, (p, v), _ = heapq.heappop(heap)
            cnt += 1
            if np.linalg.norm(p - goal) <= prm.goal_tol and (v_goal_max is None or np.linalg.norm(v) <= v_goal_max):
                # reconstruct
                seq = []
                k = nid
                while k is not None:
                    st, parent, u = nodes[k]
                    seq.append((st, u))
                    k = parent
                seq.reverse()
                return seq, cnt
            for u in self.U:
                v1 = v + u * tau
                if np.max(np.abs(v1)) > prm.v_max + 1e-9 or np.linalg.norm(v1) > prm.v_max * 1.0001:
                    continue
                pts = p + np.outer(ts, v) + 0.5 * np.outer(ts ** 2, u)
                if not np.all(self.free(pts)):
                    continue
                p1 = pts[-1]
                ng = gc + (float(u @ u) + prm.rho) * tau
                key = self._key(p1, v1)
                if ng >= best_g.get(key, np.inf):
                    continue
                best_g[key] = ng
                nodes[uid] = ((p1, v1), nid, u)
                heapq.heappush(heap, (ng + self._h(p1, goal, v1), ng, uid, (p1, v1), None))
                uid += 1
        return None, cnt

    def _has_exit(self, state, nxt) -> bool:
        """Cheap look-ahead: can the end state brake to (near) zero without colliding?"""
        p, v = state
        sp = np.linalg.norm(v)
        if sp < 1e-6:
            return True
        tstop = sp / self.prm.a_max
        a = -v / sp * self.prm.a_max
        ts = np.linspace(0, tstop, 6)[1:]
        pts = p + np.outer(ts, v) + 0.5 * np.outer(ts ** 2, a)
        return bool(np.all(self.free(pts)))

    def plan_through(self, waypoints: Sequence[np.ndarray], z: float, stop_at_end: bool = True):
        """Chain segment searches through waypoints; returns (Trajectory, stats) or (None, stats)."""
        p = np.asarray(waypoints[0], float)[:2]
        v = np.zeros(2)
        samples: List[Tuple[float, np.ndarray, np.ndarray]] = [(0.0, p.copy(), v.copy())]
        t = 0.0
        total = 0
        for k, wp in enumerate(waypoints[1:], start=1):
            last = k == len(waypoints) - 1
            # Arriving at a corner waypoint too fast can leave no collision-free
            # primitive for the next segment, so intermediate arrival speed is
            # bounded and relaxed step by step.
            caps = [0.25] if (last and stop_at_end) else [0.6, 0.4, 0.2]
            seq = None
            for cap in caps:
                seq, n = self.plan_segment(p, v, np.asarray(wp, float)[:2], cap)
                total += n
                if seq is not None:
                    nxt = waypoints[k + 1] if not last else None
                    if nxt is None or self._has_exit(seq[-1][0], np.asarray(nxt, float)[:2]):
                        break
                    seq = None
            if seq is None:
                return None, {"expansions": total, "failed_segment": k}
            for (st, u) in seq[1:]:
                p_prev, v_prev = samples[-1][1], samples[-1][2]
                for s in np.linspace(0, self.prm.tau, 4)[1:]:
                    samples.append((t + s, p_prev + v_prev * s + 0.5 * u * s * s, v_prev + u * s))
                t += self.prm.tau
            p, v = samples[-1][1].copy(), samples[-1][2].copy()
        T = np.array([s[0] for s in samples])
        P = np.array([[s[1][0], s[1][1], z] for s in samples])
        V = np.array([np.linalg.norm(s[2]) for s in samples])
        S = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
        yaw = np.array([math.atan2(s[2][1], s[2][0]) if np.linalg.norm(s[2]) > 0.05 else np.nan for s in samples])
        # fill undefined headings (hover) with the neighbouring travel heading
        idx = np.where(~np.isnan(yaw))[0]
        if len(idx):
            yaw = np.interp(np.arange(len(yaw)), idx, yaw[idx])
        else:
            yaw = np.zeros(len(yaw))
        return Trajectory(p=P, s=S, v=V, t=T, yaw=yaw), {"expansions": total}
