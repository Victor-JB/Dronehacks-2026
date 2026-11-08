"""Exploration policy: unknown course -> online map -> frontiers -> A* + clothoid -> TrajectoryFollower ->
Intent. Sees ONLY sensor-shaped inputs (depth/RGB images, K, extrinsic, odometry). Never imports the sim.

States
  SCAN         rotate on the spot (forward camera: the start surroundings are unseen)
  EXPLORE      fly to the best frontier (A* on the band grid, clothoid smoothing, time-optimal profile)
  LOOK         at a frontier: face the unknown, pause, then pick the next frontier
  INSPECT      a narrow opening is on the route but not yet verified: go to a standoff and look at it
  TO_TARGET    a verified landing square: fly over its tracked estimate
  VIEW_TARGET  a detected but not yet verified square: approach a viewpoint to re-observe it
  OVER_TARGET  arrived over the estimate. DESCENT IS DISABLED: hover and report (no validated landing)
  HOLD         no safe progress possible (no reachable frontier, lost localization, target too uncertain)
Replans every `replan_s`, when the current trajectory meets newly blocked space, or on arrival.
Dead-end memory: reached and unreachable frontier goals are blacklisted; visited cells are penalised.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from ..planner_astar import ClearancePlanner
from ..timing import fillet_path, time_parameterize
from ..tracker import FollowerParams, TrajectoryFollower
from .landing import LandingParams, TargetTracker, TrackParams, detect, label
from .online_map import FREE, UNKNOWN, MapParams, OnlineMap
from .openings import OpeningParams, evaluate_opening


@dataclass
class ExplorerParams:
    v_explore: float = 0.45
    a_max: float = 0.8
    scan_rate: float = 0.7              # rad/s
    look_s: float = 1.5
    replan_s: float = 1.0
    goal_tol: float = 0.20
    preferred_clearance: float = 0.45
    narrow: float = 0.42                # path clearance below this triggers the opening check
    blacklist_r: float = 0.5
    w_size: float = 0.02                # score bonus per frontier cell in a cluster
    w_turn: float = 0.6                 # penalty for frontiers behind (avoid back-tracking)
    w_visit: float = 1.5
    min_cluster: int = 6
    view_dist: float = 1.8              # target viewpoint distance (forward camera sees the floor beyond ~1.5 m)
    target_tol: float = 0.15
    max_inspect: int = 2
    standoff: float = 0.6               # straight run before/after a verified opening's centre
    max_target_views: int = 3
    switch_ratio: float = 0.5           # switch frontier only if the best new one costs < this x the current


@dataclass
class Intent:
    velocity: np.ndarray
    yaw_rate: float
    state: str
    note: str = ""


class Explorer:
    def __init__(self, mprm: MapParams = MapParams(), prm: ExplorerParams = ExplorerParams(),
                 lprm: LandingParams = LandingParams(), tprm: TrackParams = TrackParams(),
                 oprm: OpeningParams = OpeningParams()):
        self.mprm, self.prm, self.lprm, self.oprm = mprm, prm, lprm, oprm
        self.map = OnlineMap(mprm)
        self.tracker = TargetTracker(tprm)
        self.state = "SCAN"
        self.scan_left = 2 * math.pi
        self.follower: Optional[TrajectoryFollower] = None
        self.goal: Optional[np.ndarray] = None
        self.blacklist: List[np.ndarray] = []
        self.vetoes: List[dict] = []           # unverified openings: {"p", "axis", "tries", "permanent"}
        self.inspecting: Optional[dict] = None
        self.verified_openings: List = []
        self.t_plan = -1e9
        self.t_state = 0.0
        self.look_yaw = None
        self.pl: Optional[ClearancePlanner] = None
        self.cls = None
        self.log: List[str] = []
        self.plan_times: List[float] = []
        self.replans = 0
        self.target_views = 0

    # ------------------------------------------------------------------ perception
    def on_frame(self, t, depth, rgb, K, R_wc, o_w, loc_ok: bool):
        if not loc_ok:
            return                                    # never smear the map with an unreliable pose
        self.map.integrate(depth, K, R_wc, o_w, t)
        if rgb is not None:
            for d in detect(rgb, depth, K, R_wc, o_w, self.lprm):
                self.tracker.update(d)

    # ------------------------------------------------------------------ policy
    def step(self, t, pos, yaw, vel, loc_ok: bool) -> Intent:
        p = self.prm
        pos = np.asarray(pos, float)
        self.map.mark_visited(pos)
        self.tracker.predict(t, pos)
        if not loc_ok:
            return self._set("HOLD", t, "localization not OK")
        if self.state == "OVER_TARGET":
            return Intent(np.zeros(3), 0.0, "OVER_TARGET", "descent disabled: hovering over the target estimate")
        if self.state == "SCAN":
            if self.scan_left > 0:
                self.scan_left -= p.scan_rate * 0.1
                return Intent(self._hold_z(pos), p.scan_rate, "SCAN")
            self.state = "EXPLORE"
        # target logic has priority once a square is tracked
        if self.tracker.xy is not None and self.state not in ("TO_TARGET",):
            if self.tracker.verified:
                self._goto(t, pos, yaw, vel, self.tracker.xy, "TO_TARGET")
            elif self.tracker.usable and self.state not in ("VIEW_TARGET", "LOOK"):
                if self.target_views >= p.max_target_views:
                    return self._set("HOLD", t, "target seen but never verified: hold for recovery")
                self.target_views += 1
                self._goto(t, pos, yaw, vel, self._viewpoint(pos), "VIEW_TARGET")
        if self.state == "TO_TARGET":
            if not self.tracker.usable:
                return self._set("HOLD", t, "target estimate too uncertain: hold for recovery")
            self.goal = self.tracker.xy.copy()
            if np.linalg.norm(pos[:2] - self.goal) < p.target_tol:
                self.state = "OVER_TARGET"
                self.log.append("%.1f over target (sigma %.2f m); descent disabled" % (t, self.tracker.sigma))
                return Intent(np.zeros(3), 0.0, "OVER_TARGET")
        if self.state == "LOOK":
            if t - self.t_state < p.look_s:
                return Intent(self._hold_z(pos), self._yaw_to(yaw, self.look_yaw), "LOOK")
            self.state = "EXPLORE"
            self.follower = None
        if self.state == "HOLD":
            if t - self.t_state < 2.0:
                return Intent(self._hold_z(pos), 0.0, "HOLD")
            self.state = "EXPLORE"                  # retry after a pause (new evidence may unblock)
        # (re)plan
        need = (self.follower is None or self.follower.done or t - self.t_plan >= p.replan_s
                or not self._traj_still_clear())
        if need:
            self._replan(t, pos, yaw, vel)
            if self.follower is None:
                return Intent(self._hold_z(pos), 0.0, self.state, "no plan")
        if self.goal is not None and np.linalg.norm(pos[:2] - self.goal) < p.goal_tol:
            return self._arrived(t, pos, yaw)
        v, yr, _, _ = self.follower.step(pos, yaw, 0.1)
        return Intent(v, yr, self.state)

    # ------------------------------------------------------------------ helpers
    def _set(self, s, t, note=""):
        if self.state != s:
            self.log.append("%.1f %s: %s" % (t, s, note))
        self.state, self.t_state = s, t
        self.follower = None
        return Intent(np.zeros(3), 0.0, s, note)

    def _hold_z(self, pos):
        return np.array([0.0, 0.0, float(np.clip(1.0 * (self.mprm.cruise_z - pos[2]), -0.3, 0.3))])

    @staticmethod
    def _yaw_to(yaw, target, k=1.5, lim=0.8):
        if target is None:
            return 0.0
        e = (target - yaw + math.pi) % (2 * math.pi) - math.pi
        return float(np.clip(k * e, -lim, lim))

    def _viewpoint(self, pos):
        tgt = self.tracker.xy
        d = pos[:2] - tgt
        n = np.linalg.norm(d)
        return tgt + (d / n if n > 1e-6 else np.array([1.0, 0.0])) * self.prm.view_dist

    def _goto(self, t, pos, yaw, vel, goal, state):
        self.state, self.t_state = state, t
        self.goal = np.asarray(goal, float)
        self.follower = None
        self.log.append("%.1f %s -> (%.2f, %.2f)" % (t, state, goal[0], goal[1]))

    def _arrived(self, t, pos, yaw):
        p = self.prm
        if self.state == "VIEW_TARGET":
            self.state, self.t_state = "LOOK", t
            self.look_yaw = math.atan2(self.tracker.xy[1] - pos[1], self.tracker.xy[0] - pos[0])
            return Intent(self._hold_z(pos), 0.0, "LOOK", "viewing target")
        if self.state == "INSPECT" and self.inspecting is not None:
            self.state, self.t_state = "LOOK", t
            ax = self.inspecting["axis"]
            self.look_yaw = math.atan2(ax[1], ax[0])
            self.inspecting["tries"] += 1
            self.inspecting["recheck"] = True          # re-evaluated after the look (new evidence)
            self.inspecting = None
            return Intent(self._hold_z(pos), 0.0, "LOOK", "inspecting opening")
        # frontier reached: blacklist it (dead-end memory) and look at the unknown beyond it
        self.blacklist.append(self.goal.copy())
        self.look_yaw = self._unknown_bearing(pos)
        self.state, self.t_state = "LOOK", t
        self.follower = None
        return Intent(self._hold_z(pos), 0.0, "LOOK")

    def _unknown_bearing(self, pos):
        cls = self.cls if self.cls is not None else self.map.classify()
        X, Y = self.map.grid_coords()
        m = (cls == UNKNOWN) & ((X - pos[0]) ** 2 + (Y - pos[1]) ** 2 < 1.5 ** 2)
        if not m.any():
            return None
        return math.atan2(Y[m].mean() - pos[1], X[m].mean() - pos[0])

    def _traj_still_clear(self) -> bool:
        if self.follower is None or self.pl is None:
            return True
        cls = self.map.classify()
        g = self.map.planning_grid(cls)
        self._apply_vetoes(g)
        dist = g.distance()
        tr = self.follower.traj
        k0 = max(0, int(np.searchsorted(tr.t, self.follower.tau)) - 1)
        p = tr.p[k0:, :2]
        if len(p) < 2:
            return True
        # the WHOLE remaining path, densified to half a voxel, must keep the body clear of blocked cells
        s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))])
        q = np.arange(0.0, s[-1] + 1e-9, 0.5 * g.res)
        pts = np.stack([np.interp(q, s, p[:, 0]), np.interp(q, s, p[:, 1])], 1)
        ij = np.floor((pts - g.origin) / g.res).astype(int)
        ok = (ij[:, 0] >= 0) & (ij[:, 1] >= 0) & (ij[:, 0] < dist.shape[1]) & (ij[:, 1] < dist.shape[0])
        if not ok.all():
            return False
        return bool((dist[ij[:, 1], ij[:, 0]] >= self.mprm.robot_radius - 0.02).all())

    def _recheck_vetoes(self, t, force=False):
        for v in self.vetoes:
            if v.get("cleared") or v.get("permanent") or not (force or v.pop("recheck", False)):
                continue
            op = evaluate_opening(self.map, v["p"], v["axis"], True, self.oprm)
            if op.verified or op.kind == "open":
                v["cleared"] = True
                self.log.append("%.1f opening re-checked: %s" % (t, "verified" if op.verified else "open space"))
            elif v["tries"] >= self.prm.max_inspect:
                v["permanent"] = True
                self.log.append("%.1f opening rejected for good: %s" % (t, op.reason))

    def _apply_vetoes(self, grid):
        for v in self.vetoes:
            if v.get("cleared"):
                continue
            r, c = grid.to_cell(v["p"])
            rad = int(math.ceil(0.12 / grid.res))
            grid.occ[max(0, r - rad):r + rad + 1, max(0, c - rad):c + rad + 1] = True

    def _replan(self, t, pos, yaw, vel):
        import time as _t
        t0 = _t.perf_counter()
        self.t_plan = t
        self.replans += 1
        self.cls = cls = self.map.classify()
        grid = self.map.planning_grid(cls)
        self._recheck_vetoes(t)
        self._apply_vetoes(grid)
        self.pl = pl = ClearancePlanner(grid, self.mprm.robot_radius, self.prm.preferred_clearance, 4.0)
        if self.state in ("TO_TARGET", "VIEW_TARGET", "INSPECT") and self.goal is not None:
            goals = [self.goal]
        else:
            self.state = "EXPLORE"
            goals = self._frontier_goals(pos, yaw, cls, pl)
            if not goals and any(not (v.get("cleared") or v.get("permanent")) for v in self.vetoes):
                self._recheck_vetoes(t, force=True)            # stuck: maybe a vetoed opening is now verified
                grid = self.map.planning_grid(cls)
                self._apply_vetoes(grid)
                self.pl = pl = ClearancePlanner(grid, self.mprm.robot_radius, self.prm.preferred_clearance, 4.0)
                goals = self._frontier_goals(pos, yaw, cls, pl)
            if not goals:
                self.follower = None
                self._set("HOLD", t, "no reachable frontier")
                self.plan_times.append(_t.perf_counter() - t0)
                return
        for g in goals[:6]:
            path = pl.astar(pos[:2], g)
            if path is None or len(path) < 2:
                if self.state == "EXPLORE":
                    self.blacklist.append(np.asarray(g, float))
                continue
            path = np.vstack([pos[:2], path])
            ok, path, anchors = self._openings_ok(t, pos, path, pl, g)
            if not ok:
                self.plan_times.append(_t.perf_counter() - t0)
                return
            self.goal = np.asarray(g, float)
            self.follower = self._trajectory(path, pl, vel, anchors)
            self.plan_times.append(_t.perf_counter() - t0)
            return
        self.follower = None
        if self.state == "EXPLORE":
            self._set("HOLD", t, "frontiers unreachable")
        self.plan_times.append(_t.perf_counter() - t0)

    def _frontier_goals(self, pos, yaw, cls, pl) -> List[np.ndarray]:
        p = self.prm
        cells = self.map.frontiers(cls, pl.free, self.mprm.robot_radius + 3 * self.mprm.voxel)
        if len(cells) == 0:
            return []
        mask = np.zeros(cls.shape, bool)
        mask[cells[:, 0], cells[:, 1]] = True
        cand = []
        hd = np.array([math.cos(yaw), math.sin(yaw)])
        for rows, cols in label(mask):
            if len(rows) < p.min_cluster:
                continue
            pts = np.array([pl.grid.to_world(r, c) for r, c in zip(rows, cols)])
            if self.blacklist:                       # dead-end memory excludes cells, not whole clusters
                bl = np.array(self.blacklist)
                far = np.min(np.linalg.norm(pts[:, None, :] - bl[None, :, :], axis=2), axis=1) >= p.blacklist_r
                pts = pts[far]
                if len(pts) < p.min_cluster:
                    continue
            g = pts[np.argmin(np.linalg.norm(pts - pts.mean(0), axis=1))]
            d = g - pos[:2]
            dist = float(np.linalg.norm(d))
            turn = 0.0 if dist < 1e-6 else 1.0 - float(np.dot(d / dist, hd))
            r, c = pl.grid.to_cell(g)
            rad = 6
            vis = self.map.visited[max(0, r - rad):r + rad, max(0, c - rad):c + rad].mean()
            cand.append((dist + p.w_turn * turn + p.w_visit * vis - p.w_size * len(rows), g))
        cand.sort(key=lambda x: x[0])
        goals = [g for _, g in cand]
        # goal persistence: keep chasing the current frontier while it is still one and not clearly beaten,
        # otherwise replanning every second flips between frontiers and the drone wanders (back-tracking)
        cur = self.goal if self.state == "EXPLORE" else None
        if cur is not None and len(cells):
            fr = np.array([pl.grid.to_world(r, c) for r, c in cells[:: max(1, len(cells) // 2000)]])
            still = np.min(np.linalg.norm(fr - cur, axis=1)) < 0.3
            cur_cost = float(np.linalg.norm(cur - pos[:2]))
            best_cost = cand[0][0] if cand else math.inf
            if still and not any(np.linalg.norm(cur - b) < p.blacklist_r for b in self.blacklist)                     and best_cost > p.switch_ratio * cur_cost:
                goals = [cur] + [g for g in goals if np.linalg.norm(g - cur) > 0.3]
        return goals

    def _openings_ok(self, t, pos, path, pl, goal):
        """A path squeezing through a narrow, ENCLOSED opening is only kept if the opening is verified;
        otherwise the opening is vetoed and inspected from a standoff."""
        clr = np.array([pl.clearance_at(q) for q in path])
        k = int(np.argmin(clr))
        if clr[k] >= self.prm.narrow:
            return True, path, []
        a, b = path[max(0, k - 4)], path[min(len(path) - 1, k + 4)]
        n = b - a
        if np.linalg.norm(n) < 1e-6:
            return True, path, []
        op = evaluate_opening(self.map, path[k], n, True, self.oprm)
        if op.kind in ("open", "none"):
            return True, path, []
        if op.verified:
            if not any(np.linalg.norm(op.centre[:2] - o.centre[:2]) < 0.3 for o in self.verified_openings):
                self.verified_openings.append(op)
                self.log.append("%.1f opening verified: %s %.2fx%.2f m at (%.2f, %.2f), axis (%.2f, %.2f), run %.1f m"
                                % (t, op.kind, op.width, op.height, op.centre[0], op.centre[1], op.normal[0],
                                   op.normal[1], op.length))
            # straight, aligned traversal: standoff -> centre -> standoff along the opening axis
            ax = op.normal[:2] * (1.0 if np.dot(op.normal[:2], n) >= 0 else -1.0)
            s = self.prm.standoff
            ea, eb = op.centre[:2] - ax * s, op.centre[:2] + ax * s
            via = pl.plan_via([pos[:2], ea, eb, goal])
            if via is not None:
                via = np.vstack([pos[:2], via])
                ia = int(np.argmin(np.linalg.norm(via - ea, axis=1)))
                ib = int(np.argmin(np.linalg.norm(via - eb, axis=1)))
                return True, via, [ia, ib]
            return True, path, []
        # unverified: veto it and go and look, unless already looked at enough
        near = [v for v in self.vetoes if np.linalg.norm(v["p"] - path[k]) < 0.3]
        v = near[0] if near else {"p": np.asarray(path[k], float), "axis": op.normal[:2], "tries": 0}
        if not near:
            self.vetoes.append(v)
            self.log.append("%.1f opening NOT verified (%s): vetoed, inspecting" % (t, op.reason))
        self.follower = None
        if v["tries"] < self.prm.max_inspect:
            stand = op.centre[:2] - op.normal[:2] * 0.9
            if np.dot(stand - pos[:2], op.normal[:2]) > 0:              # approach from our side
                stand = op.centre[:2] + op.normal[:2] * 0.9
            self.inspecting = v
            self._goto(t, pos, None, None, stand, "INSPECT")
        else:
            v["permanent"] = True
        return False, path, []

    def _trajectory(self, path, pl, vel, anchors=()):
        r = self.mprm.robot_radius
        sparse = pl.shortcut(path, min_clearance=r + 0.02, keep=anchors)
        keep_pts = [path[i] for i in anchors]
        max_tan = [None] * len(sparse)
        for kp in keep_pts:                                  # stay straight through the opening
            j = int(np.argmin(np.linalg.norm(sparse - kp, axis=1)))
            max_tan[j] = 0.08
        z = self.mprm.cruise_z
        clear_ok = lambda q: pl.clearance_at(q) >= r + 0.03
        smooth, kappa = fillet_path(sparse, 0.4, clear_ok, transition="clothoid", max_tangent=max_tan,
                                    return_kappa=True)
        p3 = np.hstack([smooth, np.full((len(smooth), 1), z)])
        p = self.prm

        def vlim(q):
            d = pl.clearance_at(q)
            return 0.25 + (p.v_explore - 0.25) * float(np.clip((d - r) / max(p.preferred_clearance - r, 1e-6), 0, 1))
        v0 = min(float(np.linalg.norm(np.asarray(vel)[:2])), p.v_explore) if vel is not None else 0.0
        tr = time_parameterize(p3, p.v_explore, p.a_max, p.a_max, vlim, v_start=v0, kappa=kappa)
        return TrajectoryFollower(tr, FollowerParams(v_cap=p.v_explore + 0.1, v_lead=0.3))
