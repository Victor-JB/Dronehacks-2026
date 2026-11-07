"""Safe Flight Corridor + min-jerk QP trajectory planner (numpy only).

Implements, in 2-D (the course is flown at a fixed cruise height), the pipeline of
  S. Liu, M. Watterson, K. Mohta, K. Sun, S. Bhattacharya, C. J. Taylor, V. Kumar,
  "Planning Dynamically Feasible Trajectories for Quadrotors Using Safe Flight Corridors
  in 3-D Complex Environments", IEEE RA-L 2(3), 2017  (code: github.com/sikang/DecompUtil)

  1. piece-wise linear collision-free path P = <p0 .. pn>       (our clearance A* + shortcut)
  2. for every segment L_i: "Find Ellipsoid" (shrink a circle to the largest ellipse around L_i
     containing no obstacle point) and "Find Polyhedron" (Alg. 1: dilate the ellipse until it touches
     an obstacle, add the tangent half-plane, drop the obstacles behind it, repeat), restricted to a
     bounding box of radius r_s around the segment (paper Sec. II-B.3)
  3. "Shrink" every half-plane by the robot radius r (Sec. II-B.4). If a half-plane would cut the
     segment, it is rotated about its touching point until the segment keeps distance r (Fig. 9e).
     Safety: each obstacle point is removed by some half-plane (o is on its far side), so after
     shrinking every point of the corridor is >= r from every obstacle point.
  4. min-jerk piece-wise polynomial, one 5th-order piece per polygon, as a QP (Eq. 3) with the
     corridor imposed at sample points (the paper's "sample-based method"), continuity up to jerk,
     rest-to-rest boundary conditions;
  5. time allocation re-scaled with Eq. (5): dT' = max{1, v/vmax, sqrt(a/amax), cbrt(j/jmax)} dT.
     Because we optimise for lap time we also *shorten* pieces that stay below all limits, iterating
     until each piece touches a limit, then finish with the paper's stretch-only rule.

The QP is solved with a small dense ADMM (OSQP-style: Stellato et al., Math. Prog. Comp. 2020), so
there is no solver dependency on the drone.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

DEG = 5                    # polynomial degree (min jerk)
NC = DEG + 1               # coefficients per axis per piece


@dataclass
class SFCParams:
    robot_r: float = 0.30          # half-plane shrink (robot radius + margin), m
    bbox_r: float = 1.0            # bounding box half-width around each segment, m
    v_max: float = 0.85
    a_max: float = 1.5
    j_max: float = 5.0
    samples: int = 12              # corridor constraint samples per piece
    time_iters: int = 14
    shrink_floor: float = 0.80     # per-iteration lower bound on the time scale factor
    admm_iters: int = 6000
    admm_eps: float = 2e-4


# ------------------------------------------------------------------ corridor construction
def obstacle_points(occ: np.ndarray, origin: np.ndarray, res: float) -> np.ndarray:
    """Centres of occupied cells that touch free space (the obstacle surface), as (N,2) world xy."""
    occ = occ.astype(bool)
    pad = np.pad(occ, 1, constant_values=True)
    interior = pad[:-2, 1:-1] & pad[2:, 1:-1] & pad[1:-1, :-2] & pad[1:-1, 2:]
    r, c = np.nonzero(occ & ~interior)
    return origin + (np.stack([c, r], 1) + 0.5) * res


def _frame(p1, p2):
    d = p2 - p1
    L = float(np.linalg.norm(d))
    e1 = d / L if L > 1e-9 else np.array([1.0, 0.0])
    return e1, np.array([-e1[1], e1[0]]), L


def find_ellipse(p1: np.ndarray, p2: np.ndarray, obs: np.ndarray):
    """Largest ellipse with major axis = segment p1p2 (semi-axis L/2) containing no obstacle point.
    Returns (centre, R (2x2, columns = axes), semi-axes (a, b))."""
    e1, e2, L = _frame(p1, p2)
    c = 0.5 * (p1 + p2)
    a = max(L / 2, 1e-3)
    b = a
    if len(obs):
        rel = obs - c
        u, w = rel @ e1, rel @ e2
        for _ in range(len(obs)):
            inside = (u / a) ** 2 + (w / b) ** 2 < 1.0
            if not inside.any():
                break
            idx = np.nonzero(inside)[0]
            j = idx[np.argmin(np.hypot(u[idx], w[idx]))]            # closest obstacle to the centre
            b = max(abs(w[j]) / math.sqrt(max(1.0 - (u[j] / a) ** 2, 1e-12)), 1e-4)
    return c, np.stack([e1, e2], 1), np.array([a, b])


def _keep_segment(n0: np.ndarray, pc: np.ndarray, ends: Sequence[np.ndarray], r: float) -> Optional[np.ndarray]:
    """Unit normal n of a line through pc with n.(q - pc) <= -r for both segment ends, closest to n0."""
    if all(float(n0 @ (q - pc)) <= -r for q in ends):
        return n0
    th = np.linspace(-math.pi, math.pi, 2880, endpoint=False)
    N = np.stack([np.cos(th), np.sin(th)], 1)
    ok = np.ones(len(th), bool)
    for q in ends:
        ok &= (N @ (q - pc)) <= -r
    if not ok.any():
        return None
    k = np.nonzero(ok)[0]
    return N[k[np.argmax(N[k] @ n0)]]


def find_polygon(p1: np.ndarray, p2: np.ndarray, obs: np.ndarray, prm: SFCParams):
    """Alg. 1 (Find Polyhedron) with bounding box and robot-radius shrink.
    Returns (A (k,2) unit normals, b (k,)) of the SHRUNK polygon {x : A x <= b}, and a flag that is
    False if some half-plane could not keep the segment r away (segment too close to an obstacle)."""
    e1, e2, L = _frame(p1, p2)
    r = prm.robot_r
    # bounding box (paper Fig. 8): segment-aligned, r_s beyond the segment on every side
    box_A = np.array([e1, -e1, e2, -e2])
    box_b = np.array([e1 @ p2 + prm.bbox_r, -(e1 @ p1) + prm.bbox_r, e2 @ p1 + prm.bbox_r, -(e2 @ p1) + prm.bbox_r])
    if len(obs):
        inb = np.all(obs @ box_A.T <= box_b + 1e-9, axis=1)
        rem = obs[inb]
    else:
        rem = obs
    c, R, ax = find_ellipse(p1, p2, rem)
    Einv = R @ np.diag(1.0 / ax) @ R.T                 # maps world offsets to the unit circle
    A, b, ok_all = [], [], True
    while len(rem):
        m = np.linalg.norm((rem - c) @ Einv.T, axis=1)
        j = int(np.argmin(m))
        pc = rem[j]
        g = Einv.T @ (Einv @ (pc - c))                 # gradient of the ellipse function at pc (Eq. 2)
        n0 = g / (np.linalg.norm(g) + 1e-12)
        n = _keep_segment(n0, pc, (p1, p2), r)
        if n is None:                                  # obstacle within r of the segment
            ok_all = False
            n = n0
        bj = float(n @ pc)
        A.append(n); b.append(bj)
        rem = rem[(rem @ n) < bj - 1e-9]               # keep only obstacles strictly on the inner side
    A = np.vstack(A + list(box_A)) if A else box_A.copy()
    b = np.concatenate([b, box_b]) if b else box_b.copy()
    return A, b - r, ok_all


def build_corridor(path: np.ndarray, obs: np.ndarray, prm: SFCParams):
    polys, ok = [], True
    for p1, p2 in zip(path[:-1], path[1:]):
        A, b, f = find_polygon(np.asarray(p1, float), np.asarray(p2, float), obs, prm)
        polys.append((A, b))
        ok &= f
    return polys, ok


# ------------------------------------------------------------------ polynomial helpers
def _basis(tau: float, m: int) -> np.ndarray:
    """d^m/dtau^m of [1, tau, ..., tau^DEG]."""
    out = np.zeros(NC)
    for k in range(m, NC):
        out[k] = math.factorial(k) / math.factorial(k - m) * tau ** (k - m)
    return out


def _jerk_hessian() -> np.ndarray:
    H = np.zeros((NC, NC))
    for k in range(3, NC):
        for l in range(3, NC):
            ck = k * (k - 1) * (k - 2); cl = l * (l - 1) * (l - 2)
            H[k, l] = ck * cl / (k + l - 5)
    return H


H_JERK = _jerk_hessian()


class PolyTraj2D:
    """Piece-wise 5th-order polynomial in normalised time tau = t / T_i on each piece."""

    def __init__(self, coeffs: np.ndarray, T: np.ndarray):
        self.c = coeffs          # (n, 2, NC)
        self.T = np.asarray(T, float)
        self.t0 = np.concatenate([[0.0], np.cumsum(self.T)])

    @property
    def duration(self):
        return float(self.t0[-1])

    def eval(self, t: np.ndarray, m: int = 0) -> np.ndarray:
        t = np.clip(np.atleast_1d(np.asarray(t, float)), 0.0, self.duration)
        i = np.clip(np.searchsorted(self.t0, t, side="right") - 1, 0, len(self.T) - 1)
        tau = (t - self.t0[i]) / self.T[i]
        P = np.zeros((len(t), NC))
        for k in range(m, NC):
            P[:, k] = math.factorial(k) / math.factorial(k - m) * tau ** (k - m)
        out = np.einsum("nk,ndk->nd", P, self.c[i])
        return out / self.T[i][:, None] ** m

    def piece_max(self, m: int, n_s: int = 40) -> np.ndarray:
        """max |d^m p/dt^m| on each piece."""
        out = np.zeros(len(self.T))
        tau = np.linspace(0, 1, n_s)
        B = np.stack([_basis(x, m) for x in tau])
        for i in range(len(self.T)):
            v = B @ self.c[i].T / self.T[i] ** m
            out[i] = float(np.max(np.linalg.norm(v, axis=1)))
        return out


# ------------------------------------------------------------------ QP (dense ADMM, OSQP-style)
def solve_qp(P: np.ndarray, q: np.ndarray, A: np.ndarray, l: np.ndarray, u: np.ndarray,
             iters: int = 6000, eps: float = 2e-4, x0=None, y0=None):
    """min 1/2 x'Px + q'x  s.t.  l <= Ax <= u.  Returns (x, y, info)."""
    n, m = P.shape[0], A.shape[0]
    eq = np.isclose(l, u)
    sigma, alpha = 1e-6, 1.6
    rho0 = 0.1
    rho = np.where(eq, 1e3 * rho0, rho0)
    x = np.zeros(n) if x0 is None else x0.copy()
    z = np.clip(A @ x, l, u)
    y = np.zeros(m) if y0 is None else y0.copy()

    def factor(rho):
        return np.linalg.inv(P + sigma * np.eye(n) + A.T @ (rho[:, None] * A))

    Kinv = factor(rho)
    info = {"iters": iters, "converged": False}
    for k in range(iters):
        xt = Kinv @ (sigma * x - q + A.T @ (rho * z - y))
        zt = A @ xt
        x = alpha * xt + (1 - alpha) * x
        zr = alpha * zt + (1 - alpha) * z
        z_new = np.clip(zr + y / rho, l, u)
        y = y + rho * (zr - z_new)
        z = z_new
        if k % 25 == 0 or k == iters - 1:
            Ax = A @ x
            r_p = float(np.max(np.abs(Ax - z)))
            Px = P @ x
            Aty = A.T @ y
            r_d = float(np.max(np.abs(Px + q + Aty)))
            sp = max(np.max(np.abs(Ax)), np.max(np.abs(z)), 1e-9)
            sd = max(np.max(np.abs(Px)), np.max(np.abs(Aty)), np.max(np.abs(q)), 1e-9)
            if r_p < eps * (1 + sp) and r_d < eps * (1 + sd):
                info.update(iters=k + 1, converged=True)
                break
            if k % 200 == 0 and k > 0:            # OSQP rho adaptation
                ratio = math.sqrt((r_p / sp) / max(r_d / sd, 1e-12))
                if ratio > 5 or ratio < 0.2:
                    rho = np.clip(rho * ratio, 1e-6, 1e6)
                    Kinv = factor(rho)
    info["r_prim"] = float(np.max(np.abs(A @ x - z)))
    return x, y, info


def min_jerk_in_corridor(path: np.ndarray, polys, T: np.ndarray, prm: SFCParams, margin: float = 0.01,
                         warm=None):
    """Eq. (3): min sum int |jerk|^2 subject to rest-to-rest ends, C3 continuity and the corridor."""
    n = len(T)
    nv = n * 2 * NC
    idx = lambda i, d: slice((i * 2 + d) * NC, (i * 2 + d + 1) * NC)
    P = np.zeros((nv, nv))
    for i in range(n):
        for d in range(2):
            P[idx(i, d), idx(i, d)] = H_JERK / T[i] ** 5
    rows, lo, hi = [], [], []

    def eq_row(entries, val):
        r = np.zeros(nv)
        for (i, d, vec) in entries:
            r[idx(i, d)] += vec
        rows.append(r); lo.append(val); hi.append(val)

    p0, pn = np.asarray(path[0], float), np.asarray(path[-1], float)
    for d in range(2):
        eq_row([(0, d, _basis(0.0, 0))], p0[d])
        for m in (1, 2):
            eq_row([(0, d, _basis(0.0, m) / T[0] ** m)], 0.0)
        eq_row([(n - 1, d, _basis(1.0, 0))], pn[d])
        for m in (1, 2):
            eq_row([(n - 1, d, _basis(1.0, m) / T[n - 1] ** m)], 0.0)
        for i in range(n - 1):
            for m in range(4):
                eq_row([(i, d, _basis(1.0, m) / T[i] ** m), (i + 1, d, -_basis(0.0, m) / T[i + 1] ** m)], 0.0)
    taus = np.linspace(0.0, 1.0, prm.samples)
    for i, (Ah, bh) in enumerate(polys):
        for tau in taus:
            B = _basis(tau, 0)
            for a, bb in zip(Ah, bh):
                r = np.zeros(nv)
                r[idx(i, 0)] = a[0] * B
                r[idx(i, 1)] = a[1] * B
                rows.append(r); lo.append(-1e9); hi.append(bb - margin)
    A = np.array(rows); l = np.array(lo); u = np.array(hi)
    s = np.linalg.norm(A, axis=1); s[s < 1e-12] = 1.0           # row scaling
    A, l, u = A / s[:, None], l / s, u / s
    l[l < -1e8] = -1e9
    x0, y0 = (warm if warm is not None else (None, None))
    x, y, info = solve_qp(P, np.zeros(nv), A, l, u, prm.admm_iters, prm.admm_eps, x0, y0)
    return PolyTraj2D(x.reshape(n, 2, NC), T), (x, y), info


def corridor_violation(traj: PolyTraj2D, polys, n_s: int = 60) -> float:
    worst = -np.inf
    tau = np.linspace(0, 1, n_s)
    B = np.stack([_basis(x, 0) for x in tau])
    for i, (Ah, bh) in enumerate(polys):
        pts = B @ traj.c[i].T
        worst = max(worst, float(np.max(pts @ Ah.T - bh)))
    return worst


def plan_sfc(path: np.ndarray, occ: np.ndarray, origin: np.ndarray, res: float, prm: SFCParams = SFCParams(),
             v_limits: Optional[Sequence[float]] = None, log=None):
    """Full pipeline. path: (n+1, 2) collision-free polyline. Returns (PolyTraj2D, polys, stats)."""
    path = np.asarray(path, float)[:, :2]
    keep = [0] + [k for k in range(1, len(path)) if np.linalg.norm(path[k] - path[k - 1]) > 1e-3]
    path = path[keep]
    obs = obstacle_points(occ, origin, res)
    polys, ok = build_corridor(path, obs, prm)
    L = np.linalg.norm(np.diff(path, axis=0), axis=1)
    vl = np.full(len(L), prm.v_max) if v_limits is None else np.minimum(np.asarray(v_limits, float), prm.v_max)
    # initial allocation: trapezoid-ish (accelerate/decelerate at the ends, cruise elsewhere)
    T = np.maximum(L / vl, np.sqrt(4 * L / prm.a_max) * 0.5)
    T[0] += vl[0] / prm.a_max; T[-1] += vl[-1] / prm.a_max
    # Time allocation. With rest-to-rest ends and position-only corridor constraints, the QP solution
    # in normalised time is invariant to a common scale k of all T_i (v ~ 1/k, a ~ 1/k^2, j ~ 1/k^3).
    # So each round: solve, compute every piece's Eq.(5) factor f_i, scale ALL pieces by max f (exactly
    # feasible, no re-solve), keep the best total, then re-balance the RELATIVE allocation (pieces at
    # their limit get more time, pieces with slack less) and repeat.
    hist, best = [], None
    for it in range(prm.time_iters):
        traj, _, info = min_jerk_in_corridor(path, polys, T, prm)
        vm, am, jm = traj.piece_max(1), traj.piece_max(2), traj.piece_max(3)
        f = np.maximum.reduce([vm / vl, np.sqrt(am / prm.a_max), np.cbrt(jm / prm.j_max)])
        k = float(f.max())
        dur = k * float(T.sum())
        hist.append((round(dur, 3), info["iters"]))
        if log:
            log("  iter %d: duration after exact rescale %.2f s (admm %d%s)" % (
                it, dur, info["iters"], "" if info["converged"] else ", not converged"))
        if best is None or dur < best[0] - 1e-6:
            best = (dur, T * k, traj.c.copy())
        if np.all(f > 0.97 * k):
            break                                    # every piece is at its limit
        T = T * np.clip((f / k) ** 0.6, prm.shrink_floor, 1.0 / prm.shrink_floor)
    traj = PolyTraj2D(best[2], best[1])
    viol = corridor_violation(traj, polys)
    stats = {"corridor_ok": bool(ok), "pieces": len(polys), "duration": traj.duration,
             "corridor_violation_m": viol, "time_iters": hist, "obstacle_pts": len(obs)}
    return traj, polys, stats
