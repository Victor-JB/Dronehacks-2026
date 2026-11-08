"""refnav.plan - occupancy grid + A* route planning. PURE (numpy + heapq): no ROS, no clock, no files.

Coordinates are NED metres (n, e) in the mission's own frame (the frame PX4 reports position in).

The grid has two layers:
* a PRIOR map (known walls/pillars, rasterised once at arming), and
* OBSERVED cells: depth points the camera has seen in at least `min_hits` different frames. A cell
  that lies within `novel_m` of the prior map is NOT added: the prior already covers known
  structure, and estimator drift or placement error would otherwise paint "ghost walls" a few cm
  away from the real ones and close tight corridors. Observation therefore only reports things the
  prior did not know about (or that are clearly somewhere else).

Planning inflates both layers by the vehicle radius plus margin (a hard limit), and prefers
clearance beyond it: a cell's step cost grows as its distance to the nearest obstacle falls below
`prefer_m`, so the path keeps to the middle of corridors and swings wide of corners instead of
hugging them at the minimum margin. That matters: a tracking error of a few cm beside a wall is
otherwise a collision, and a forward camera cannot see a wall end it is already passing. If the start or goal sits inside
the inflated zone (tight corridors, estimator error) the inflation is relaxed in steps rather than
failing, so the planner degrades to "closer than we would like" before it gives up. A returned
path is string-pulled: the fewest straight, collision-free (on the inflated grid) segments.
"""
from __future__ import annotations

import heapq
import math

import numpy as np

_SQRT2 = math.sqrt(2.0)
_NEIGHBOURS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
               (-1, -1, _SQRT2), (-1, 1, _SQRT2), (1, -1, _SQRT2), (1, 1, _SQRT2)]


def _disc_offsets(radius_cells):
    r = int(math.ceil(radius_cells))
    return [(dy, dx) for dy in range(-r, r + 1) for dx in range(-r, r + 1)
            if dy * dy + dx * dx <= radius_cells * radius_cells + 1e-9]


def _distance(occ, res):
    """Distance (m) from every cell to the nearest True cell; the grid edge counts as an obstacle.
    Two-pass 8-neighbour chamfer transform (within ~8% of Euclidean), vectorised per row."""
    occ = np.pad(occ, 1, constant_values=True)
    d = np.where(occ, 0.0, 1e9)
    rows, cols = d.shape
    idx = np.arange(cols, dtype=float)

    def scan(row):                                   # propagate along the row, both directions
        a = np.minimum.accumulate(row - idx) + idx
        b = np.minimum.accumulate((row + idx)[::-1])[::-1] - idx
        return np.minimum(a, b)

    for rr in (range(rows), range(rows - 1, -1, -1)):
        step = 1 if rr.step > 0 else -1
        for r in rr:
            q = r - step
            if 0 <= q < rows:
                prev = d[q]
                cand = prev + 1.0
                cand[1:] = np.minimum(cand[1:], prev[:-1] + _SQRT2)
                cand[:-1] = np.minimum(cand[:-1], prev[1:] + _SQRT2)
                d[r] = np.minimum(d[r], cand)
            d[r] = scan(d[r])
    return d[1:-1, 1:-1] * res


def _dilate(mask, radius_cells):
    """Binary dilation by a disc, in cells."""
    if radius_cells <= 0:
        return mask.copy()
    out = np.zeros_like(mask)
    h, w = mask.shape
    for dy, dx in _disc_offsets(radius_cells):
        ys0, ys1 = max(0, dy), min(h, h + dy)
        xs0, xs1 = max(0, dx), min(w, w + dx)
        out[ys0:ys1, xs0:xs1] |= mask[ys0 - dy:ys1 - dy, xs0 - dx:xs1 - dx]
    return out


class Grid:
    def __init__(self, bounds, res=0.05, min_hits=4, novel_m=0.2, prefer_m=0.6, beta=10.0):
        """bounds = (n_min, n_max, e_min, e_max): the flyable area (e.g. inside the safety net).
        Everything outside it is blocked."""
        self.n0, self.n1, self.e0, self.e1 = (float(b) for b in bounds)
        self.res = float(res)
        self.rows = int(math.ceil((self.n1 - self.n0) / self.res))
        self.cols = int(math.ceil((self.e1 - self.e0) / self.res))
        if self.rows < 2 or self.cols < 2:
            raise ValueError("planning bounds are too small")
        self.prior = np.zeros((self.rows, self.cols), bool)
        self.hits = np.zeros((self.rows, self.cols), np.int16)
        self.min_hits = int(min_hits)
        self.novel_m = float(novel_m)
        self.prefer_m = float(prefer_m)
        self.beta = float(beta)
        self.last_inflate = None
        self._prior_near = None
        self._cache = {}
        self._version = 0

    # ---------------------------------------------------------------- coordinates
    def cell(self, n, e):
        return int((n - self.n0) / self.res), int((e - self.e0) / self.res)

    def centre(self, r, c):
        return self.n0 + (r + 0.5) * self.res, self.e0 + (c + 0.5) * self.res

    def inside(self, r, c):
        return 0 <= r < self.rows and 0 <= c < self.cols

    def _cell_centres(self):
        rr, cc = np.mgrid[0:self.rows, 0:self.cols]
        return self.n0 + (rr + 0.5) * self.res, self.e0 + (cc + 0.5) * self.res

    # ---------------------------------------------------------------- prior map
    def add_polygon(self, pts):
        """Rasterise a CONVEX polygon given as [(n, e), ...] (either winding)."""
        pts = [(float(a), float(b)) for a, b in pts]
        nn, ee = self._cell_centres()
        sign = 0.0
        for i in range(len(pts)):
            (n1, e1), (n2, e2) = pts[i], pts[(i + 1) % len(pts)]
            sign += n1 * e2 - n2 * e1
        sign = 1.0 if sign >= 0 else -1.0
        inside = np.ones_like(self.prior)
        for i in range(len(pts)):
            (n1, e1), (n2, e2) = pts[i], pts[(i + 1) % len(pts)]
            cross = (n2 - n1) * (ee - e1) - (e2 - e1) * (nn - n1)
            inside &= sign * cross >= -1e-9
        self.prior |= inside
        self._touch(prior=True)

    def add_circle(self, n, e, radius):
        nn, ee = self._cell_centres()
        self.prior |= (nn - n) ** 2 + (ee - e) ** 2 <= float(radius) ** 2
        self._touch(prior=True)

    def _touch(self, prior=False):
        self._version += 1
        self._cache.clear()
        if prior:
            self._prior_near = None

    # ---------------------------------------------------------------- observation
    def observe(self, points_ne, force=False):
        """Count depth points (N,2) of NED (n, e). Each cell counts once per call (one frame), so
        `min_hits` means "seen in that many frames". force=True: mark the cells occupied now (an
        obstacle the mission has just confirmed it must stop for). Returns how many cells were
        newly counted."""
        if points_ne is None or len(points_ne) == 0:
            return 0
        pts = np.asarray(points_ne, float).reshape(-1, 2)
        r = np.floor((pts[:, 0] - self.n0) / self.res).astype(int)
        c = np.floor((pts[:, 1] - self.e0) / self.res).astype(int)
        ok = (r >= 0) & (r < self.rows) & (c >= 0) & (c < self.cols)
        if not ok.any():
            return 0
        flat = np.unique(r[ok] * self.cols + c[ok])
        rr, cc = flat // self.cols, flat % self.cols
        if self._prior_near is None:
            self._prior_near = _dilate(self.prior, self.novel_m / self.res)
        if not force:                                 # a forced cell is trusted over the map
            keep = ~self._prior_near[rr, cc]
            rr, cc = rr[keep], cc[keep]
        if len(rr) == 0:
            return 0
        if force:
            self.hits[rr, cc] = np.maximum(self.hits[rr, cc], self.min_hits)
        else:
            self.hits[rr, cc] = np.minimum(self.hits[rr, cc] + 1, 100)
        self._touch()
        return int(len(rr))

    def occupied(self):
        return self.prior | (self.hits >= self.min_hits)

    # ---------------------------------------------------------------- planning
    def blocked(self, inflate_m):
        """Boolean map of cells the vehicle centre may not occupy."""
        key = round(float(inflate_m), 4)
        if key in self._cache:
            return self._cache[key]
        k = float(inflate_m) / self.res
        b = _dilate(self.occupied(), k)
        m = int(math.ceil(k))
        if m > 0:                                   # the edge of the flyable area is a wall
            b[:m, :] = True
            b[-m:, :] = True
            b[:, :m] = True
            b[:, -m:] = True
        self._cache[key] = b
        return b

    def clearance(self):
        """Distance (m) from each cell to the nearest obstacle (prior or observed) or the area edge."""
        if "clr" not in self._cache:
            self._cache["clr"] = _distance(self.occupied(), self.res)
        return self._cache["clr"]

    def step_cost(self):
        """Per-cell cost multiplier: 1 with `prefer_m` or more of clearance, rising smoothly below it."""
        if "cost" not in self._cache:
            short = np.clip((self.prefer_m - self.clearance()) / max(self.prefer_m, 1e-6), 0.0, 1.0)
            self._cache["cost"] = 1.0 + self.beta * short ** 2
        return self._cache["cost"]

    def observed_blocked(self, inflate_m):
        """Inflated map of OBSERVED obstacles only (not the prior map or the area edge)."""
        key = ("obs", round(float(inflate_m), 4))
        if key not in self._cache:
            self._cache[key] = _dilate(self.hits >= self.min_hits, float(inflate_m) / self.res)
        return self._cache[key]

    def segment_free(self, a, b, inflate_m, blocked=None):
        """Is the straight segment a->b (NED) clear of inflated obstacles?"""
        blocked = self.blocked(inflate_m) if blocked is None else blocked
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        steps = max(2, int(length / (self.res * 0.5)) + 1)
        for i in range(steps + 1):
            t = i / steps
            r, c = self.cell(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
            if not self.inside(r, c) or blocked[r, c]:
                return False
        return True

    def _nearest_free(self, r, c, blocked, max_cells):
        """Closest unblocked cell to (r, c) within max_cells (ring search), or None."""
        if self.inside(r, c) and not blocked[r, c]:
            return r, c
        best = None
        for d in range(1, max_cells + 1):
            for dr in range(-d, d + 1):
                for dc in range(-d, d + 1):
                    if max(abs(dr), abs(dc)) != d:
                        continue
                    rr, cc = r + dr, c + dc
                    if self.inside(rr, cc) and not blocked[rr, cc]:
                        dist = dr * dr + dc * dc
                        if best is None or dist < best[0]:
                            best = (dist, rr, cc)
            if best is not None:
                return best[1], best[2]
        return None

    def _astar(self, start, goal, blocked):
        mult = self.step_cost()
        h = lambda r, c: math.hypot(r - goal[0], c - goal[1])  # noqa: E731
        openq = [(h(*start), 0.0, start)]
        g = {start: 0.0}
        parent = {}
        done = set()
        while openq:
            _, cost, cur = heapq.heappop(openq)
            if cur in done:
                continue
            done.add(cur)
            if cur == goal:
                path = [cur]
                while cur in parent:
                    cur = parent[cur]
                    path.append(cur)
                return path[::-1]
            r, c = cur
            for dr, dc, w in _NEIGHBOURS:
                rr, cc = r + dr, c + dc
                if not self.inside(rr, cc) or blocked[rr, cc]:
                    continue
                if dr and dc and (blocked[r + dr, c] or blocked[r, c + dc]):
                    continue                          # no squeezing diagonally between two blocked cells
                ng = cost + w * mult[rr, cc]
                if ng < g.get((rr, cc), math.inf):
                    g[(rr, cc)] = ng
                    parent[(rr, cc)] = cur
                    heapq.heappush(openq, (ng + h(rr, cc), ng, (rr, cc)))
        return None

    def plan(self, start, goal, inflate_m, relax=(1.0, 0.8, 0.6), simplify_m=0.06):
        """Path [(n, e), ...] from start to goal, or None. The first element is `start` and the
        last is `goal` exactly (they may lie in the relaxed zone). Also returns via .last_inflate
        the inflation that succeeded (smaller than inflate_m = a tighter path than wanted)."""
        self.last_inflate = None
        for f in relax:
            infl = inflate_m * f
            blocked = self.blocked(infl)
            # the start/goal may be inside the inflated zone only through estimator error: let the
            # search begin at the nearest free cell, but only within a limited distance
            reach = int(math.ceil(0.25 / self.res))
            s = self._nearest_free(*self.cell(*start), blocked, reach)
            g = self._nearest_free(*self.cell(*goal), blocked, reach)
            if s is None or g is None:
                continue
            cells = self._astar(s, g, blocked)
            if cells is None:
                continue
            pulled = self._simplify([self.centre(r, c) for r, c in cells], blocked, infl, simplify_m)
            self.last_inflate = infl
            return [tuple(start)] + pulled[1:-1] + [tuple(goal)]
        return None

    def _simplify(self, pts, blocked, infl, tol):
        """Douglas-Peucker: keep the fewest points such that no cell-centre of the A* path is more than
        `tol` from the simplified path AND every kept segment is collision-free. Unlike string-pulling
        this does not cut corners to the minimum margin, so the clearance the search paid for stays."""
        if len(pts) <= 2:
            return list(pts)
        keep = {0, len(pts) - 1}
        stack = [(0, len(pts) - 1)]
        while stack:
            i, j = stack.pop()
            if j <= i + 1:
                continue
            a, b = np.array(pts[i]), np.array(pts[j])
            ab = b - a
            norm = float(np.hypot(*ab))
            seg = np.array(pts[i + 1:j]) - a
            dev = np.abs(seg[:, 0] * ab[1] - seg[:, 1] * ab[0]) / norm if norm > 1e-9 else np.hypot(seg[:, 0], seg[:, 1])
            k = int(np.argmax(dev))
            if float(dev[k]) > tol or not self.segment_free(pts[i], pts[j], infl, blocked):
                m = i + 1 + k
                keep.add(m)
                stack += [(i, m), (m, j)]
        return [pts[i] for i in sorted(keep)]
