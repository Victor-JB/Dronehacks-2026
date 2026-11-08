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
    scan_timeout: float = 25.0          # s
    look_s: float = 1.5
    replan_s: float = 1.0
    goal_tol: float = 0.20
    preferred_clearance: float = 0.55
    v_narrow: float = 0.15              # speed where clearance is at its minimum (openings, tight corridors)
    narrow: float = 0.45                # path clearance below this triggers the opening check
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
    approach_max: float = 0.8           # a frontier needs a traversable approach cell within this distance
    tile: float = 0.8                   # frontier cells are grouped into tiles of this size (m)
    reactivate_ratio: float = 2.0       # a blacklisted frontier comes back if its cluster grows this much
    persist_r: float = 1.0              # the current goal is "the same" frontier within this distance (>= tile: its
                                        # approach point moves as the map grows; 0.4 let goals flip every replan)
    switch_gain: float = 1.0            # switch only if another frontier scores this much better (m)
    unreachable_retry_s: float = 20.0   # an unreachable frontier is skipped this long, then retried
    progress_m: float = 0.15            # watchdog: at least this much motion ...
    progress_s: float = 8.0             # ... within this long while following a plan


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
        self._scan_yaw = None
        self._scan_t0 = None
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
        self.frontier_sizes = {}
        self._prog = None
        self.looked = set()
        self.n_frontier_cells = 0

    # ------------------------------------------------------------------ perception
    def on_frame(self, t, depth, rgb, K, R_wc, o_w, loc_ok: bool):
        if not loc_ok:
            return                                    # never smear the map with an unreliable pose
        self.map.integrate(depth, K, R_wc, o_w, t)
        if rgb is not None:
            for d in detect(rgb, depth, K, R_wc, o_w, self.lprm):
                self.tracker.update(d)

    def on_rgb(self, t, rgb, K, R_wc, o_w, loc_ok: bool, depth=None):
        """A colour frame from its OWN camera (intrinsics K, pose R_wc/o_w at capture). Without an aligned
        depth image the detector projects onto the floor plane (z = floor_z) using the odometry height."""
        if not loc_ok or rgb is None:
            return
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
            # count the rotation actually measured (a vetoed yaw command must not look like a finished scan)
            if self._scan_yaw is not None:
                self.scan_left -= abs((yaw - self._scan_yaw + math.pi) % (2 * math.pi) - math.pi)
            self._scan_yaw = yaw
            self._scan_t0 = t if self._scan_t0 is None else self._scan_t0
            if self.scan_left > 0 and t - self._scan_t0 < p.scan_timeout:
                return Intent(self._hold_z(pos), p.scan_rate, "SCAN")
            if self.scan_left > 0:
                self.log.append("%.1f scan incomplete (%.0f deg left): exploring anyway" % (t, math.degrees(self.scan_left)))
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
        # no-progress watchdog: a plan the final-command gate keeps refusing must not deadlock the mission
        if self._prog is None or np.linalg.norm(pos[:2] - self._prog[1]) > p.progress_m:
            self._prog = (t, pos[:2].copy())
        elif t - self._prog[0] > p.progress_s:
            self.log.append("%.1f no progress for %.0f s towards (%.2f, %.2f): goal retired for %.0f s, replanning"
                            % (t, t - self._prog[0], self.goal[0], self.goal[1], p.unreachable_retry_s))
            self.blacklist.append({"p": self.goal.copy(), "size": 10 ** 6, "until": t + p.unreachable_retry_s})
            self._prog = (t, pos[:2].copy())
            if self.state in ("TO_TARGET", "VIEW_TARGET", "INSPECT"):
                self.state = "EXPLORE"
            self.follower = None
            return Intent(self._hold_z(pos), 0.0, self.state, "no progress")
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
        if self.state == "TO_TARGET":
            # reached the tracked target estimate: hover (the generic frontier "arrival" used to fire first,
            # blacklist the pad and drop out of TO_TARGET)
            self.state = "OVER_TARGET"
            self.log.append("%.1f over target (sigma %.2f m); descent disabled" % (t, self.tracker.sigma))
            return Intent(np.zeros(3), 0.0, "OVER_TARGET")
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
        # frontier reached: remember it (dead-end memory, with its size for re-activation) and look beyond
        self.blacklist.append({"p": self.goal.copy(), "size": self.frontier_sizes.get(tuple(np.round(self.goal, 3)), 10 ** 6)})
        self.look_yaw = self._unknown_bearing(pos)
        self.state, self.t_state = "LOOK", t
        self.follower = None
        return Intent(self._hold_z(pos), 0.0, "LOOK")

    def _unknown_bearing(self, pos):
        cls = self.cls if self.cls is not None else self.map.classify()
        X, Y = self.map.grid_coords()
        m = self.map.frontiers(cls) & ((X - pos[0]) ** 2 + (Y - pos[1]) ** 2 < 1.5 ** 2)   # not wall backsides
        if not m.any():
            return None
        return math.atan2(Y[m].mean() - pos[1], X[m].mean() - pos[0])

    def _traj_still_clear(self) -> bool:
        if self.follower is None or self.pl is None:
            return True
        cls = self.map.classify()
        g = self.map.planning_grid(cls)
        self._apply_verified(g)
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

    def _nearest_reachable(self, pos, target, pl):
        """The cell reachable from pos (same connected traversable region) nearest to target, if within 2.5 m."""
        try:
            from scipy.ndimage import label as _lab
            lab, _ = _lab(pl.free)
        except ImportError:
            lab = np.zeros(pl.free.shape, int)
            for i, (rr, cc) in enumerate(label(pl.free)):
                lab[rr, cc] = i + 1
        s0 = pl._nearest_free(pl.grid.to_cell(pos[:2]))
        if s0 is None or lab[s0] == 0:
            return None
        cells = np.argwhere(lab == lab[s0])
        W = pl.grid.origin + (cells[:, ::-1] + 0.5) * pl.grid.res
        d = np.linalg.norm(W - np.asarray(target)[:2], axis=1)
        k = int(np.argmin(d))
        return W[k] if d[k] <= 2.5 else None

    def _apply_verified(self, grid):
        """A 3-D-verified opening (true envelope fits in observed-free voxels slice by slice) is trusted by the
        2-D planner along its axis: cells within the body envelope of the verified centreline are cleared.
        Real walls lie farther than robot_radius from the axis and stay. Re-checked on every use."""
        r = self.mprm.robot_radius - grid.res
        for o in self.verified_openings:
            a = o.centre[:2] - o.normal[:2] * self.prm.standoff
            b = o.centre[:2] + o.normal[:2] * self.prm.standoff
            n = max(2, int(np.linalg.norm(b - a) / (0.5 * grid.res)) + 1)
            for q in a + np.linspace(0, 1, n)[:, None] * (b - a):
                rr, cc = grid.to_cell(q)
                k = int(math.ceil(r / grid.res))
                ys, xs = np.mgrid[max(0, rr - k):rr + k + 1, max(0, cc - k):cc + k + 1]
                cen = grid.origin + (np.stack([xs, ys], -1) + 0.5) * grid.res
                inside = np.linalg.norm(cen - q, axis=-1) <= r
                grid.occ[ys[inside], xs[inside]] = False

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
        self._apply_verified(grid)
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
                self._apply_verified(grid)
                self._apply_vetoes(grid)
                self.pl = pl = ClearancePlanner(grid, self.mprm.robot_radius, self.prm.preferred_clearance, 4.0)
                goals = self._frontier_goals(pos, yaw, cls, pl)
            if not goals:
                self.follower = None
                self._set("HOLD", t, "exploration exhausted: no safely reachable frontier")
                self.plan_times.append(_t.perf_counter() - t0)
                return
        for g in goals[:6]:
            path = pl.astar(pos[:2], g)
            if path is None or len(path) < 2:
                if self.state == "EXPLORE":
                    key = tuple(np.round(np.asarray(g) / 0.3).astype(int))
                    if key not in self.looked:
                        # unreachable only because the space between is unobserved? go as close as we can
                        # and LOOK at it (e.g. the unseen half of a tunnel) before giving up on it
                        self.looked.add(key)
                        near = self._nearest_reachable(pos, g, pl)
                        if near is not None:
                            self.inspecting = {"p": np.asarray(g, float), "axis": (np.asarray(g) - near), "tries": 0}
                            self.log.append("%.1f frontier (%.2f, %.2f) unreachable: approaching (%.2f, %.2f) to look"
                                            % (t, g[0], g[1], near[0], near[1]))
                            self._goto(t, pos, yaw, vel, near, "INSPECT")
                            path = pl.astar(pos[:2], near)
                            if path is not None and len(path) >= 2:
                                self.follower = self._trajectory(np.vstack([pos[:2], path]), pl, vel)
                                self.plan_times.append(_t.perf_counter() - t0)
                                return
                    # still unreachable after looking: retry later (the map keeps changing)
                    self.blacklist.append({"p": np.asarray(g, float), "size": self.frontier_sizes.get(tuple(np.round(g, 3)), 10 ** 6),
                                           "until": t + self.prm.unreachable_retry_s})
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
        """Frontier clusters -> approach goals (the traversable cell nearest each cluster), scored by
        distance, heading change, how visited the area is, and cluster size (information). Dead-end memory:
        a blacklisted goal is skipped unless its cluster has grown (new information re-activates it).
        Persistence: the current goal is kept while its cluster still exists and nothing is clearly better."""
        p = self.prm
        fmask = self.map.frontiers(cls)
        trav = np.argwhere(pl.free)
        self.n_frontier_cells = int(fmask.sum())
        if not fmask.any() or len(trav) == 0:
            return []
        T = pl.grid.origin + (trav[:, ::-1] + 0.5) * pl.grid.res          # traversable cell centres (x, y)
        hd = np.array([math.cos(yaw), math.sin(yaw)])
        cand = []
        fr = np.argwhere(fmask)
        P = pl.grid.origin + (fr[:, ::-1] + 0.5) * pl.grid.res
        # tiles, not connected components: a frontier boundary is often ONE long connected cluster, and
        # retiring it on the first visit removed every frontier at once
        keys = np.floor(P / p.tile).astype(int)
        uniq, inv = np.unique(keys, axis=0, return_inverse=True)
        for ti in range(len(uniq)):
            pts = P[inv.ravel() == ti]
            n = len(pts)
            if n < p.min_cluster:
                continue
            c = pts[np.argmin(np.linalg.norm(pts - pts.mean(0), axis=1))]
            d = np.linalg.norm(T - c, axis=1)
            k = int(np.argmin(d))
            if d[k] > p.approach_max:
                continue                                    # no safe spot near this frontier (yet)
            g = T[k]
            bl = [b for b in self.blacklist if np.linalg.norm(g - b["p"]) < p.blacklist_r
                  and b.get("until", math.inf) > self.t_plan]
            if bl and n < p.reactivate_ratio * max(b["size"] for b in bl):
                continue
            dv = g - pos[:2]
            dist = float(np.linalg.norm(dv))
            turn = 0.0 if dist < 1e-6 else 1.0 - float(np.dot(dv / dist, hd))
            r, cc = pl.grid.to_cell(g)
            vis = float(self.map.visited[max(0, r - 6):r + 6, max(0, cc - 6):cc + 6].mean())
            cand.append((dist + p.w_turn * turn + p.w_visit * vis - p.w_size * n, g, n))
        cand.sort(key=lambda x: x[0])
        self.frontier_sizes = {tuple(np.round(g, 3)): n for _, g, n in cand}
        goals = [g for _, g, _ in cand]
        cur = self.goal if self.state == "EXPLORE" else None
        if cur is not None and cand:
            same = [(sc, g) for sc, g, _ in cand if np.linalg.norm(g - cur) < p.persist_r]
            if same and cand[0][0] > same[0][0] - p.switch_gain:        # not clearly beaten: keep it
                goals = [same[0][1]] + [g for g in goals if np.linalg.norm(g - same[0][1]) > 1e-6]
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
        known = [o for o in self.verified_openings if np.linalg.norm(o.centre[:2] - path[k]) < 0.4]
        if known:
            # already verified: re-use its centre/axis unless the map now shows something inside the envelope
            o = known[0]
            recheck = evaluate_opening(self.map, o.centre[:2], o.normal[:2], True, self.oprm)
            if "occupied" in recheck.reason:
                self.verified_openings.remove(o)          # new evidence: no longer trusted
                self.log.append("%.1f opening at (%.2f, %.2f) no longer verified: %s" % (t, o.centre[0], o.centre[1], recheck.reason))
            op = recheck if (recheck.verified or "occupied" in recheck.reason) else o
        else:
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
            if not pl.segment_clear(ea, eb, self.mprm.robot_radius):
                # the estimated axis would not fit as a straight line (e.g. skewed in a tube): keep A*'s own
                # path, which is collision-free by construction, rather than forcing a straight line
                return True, path, []
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
            return p.v_narrow + (p.v_explore - p.v_narrow) * float(np.clip((d - r) / max(p.preferred_clearance - r, 1e-6), 0, 1))
        v0 = min(float(np.linalg.norm(np.asarray(vel)[:2])), p.v_explore) if vel is not None else 0.0
        tr = time_parameterize(p3, p.v_explore, p.a_max, p.a_max, vlim, v_start=v0, kappa=kappa)
        return TrajectoryFollower(tr, FollowerParams(v_cap=p.v_explore + 0.1, v_lead=0.3))
