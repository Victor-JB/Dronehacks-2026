# The professor's papers: what each one gives us, and what was built from it (2026-11-06)

All three were read in full from the PDFs the team supplied. Results come from `tools/compare_planners.py` (`docs/planner_comparison.md`), the SIL, and unit tests 10–11.

## 1. Liu, Watterson, Mohta, Sun, Bhattacharya, Taylor, Kumar — "Planning Dynamically Feasible Trajectories for Quadrotors Using Safe Flight Corridors in 3-D Complex Environments", IEEE RA-L 2(3):1688–1695, 2017

**Idea.** JPS/A* gives a piecewise-linear path. Around each segment the method builds a convex polyhedron of free space:
- **Find Ellipsoid:** shrink a sphere to the largest obstacle-free ellipsoid around the segment.
- **Find Polyhedron (Alg. 1):** dilate that ellipsoid until it touches an obstacle point, add the tangent half-space, remove the points behind it, and repeat.
- A **bounding box** of radius r_s limits the work.
- Every half-space is then **shrunk by the robot radius**. If shrinking would cut the path segment, the plane is rotated about its touch point (Fig. 9e).

The union of the polyhedra is the Safe Flight Corridor. A min-snap or min-jerk piecewise polynomial is solved as a **QP** (Eq. 3), with each piece constrained to its polyhedron at sample points. The time allocation is stretched with **Eq. 5**: ΔT' = max{1, v/vmax, √(a/amax), ∛(j/jmax)}·ΔT. The whole pipeline runs at 50–300 ms per replan, inside receding-horizon planning.

**Built:** `dh_core/planner_sfc.py` (2-D, numpy only):
- Alg. 1 with the bounding box and the robot-radius shrink, including the Fig. 9e rotation.
- A dense ADMM QP solver (OSQP-style), so there is no solver dependency on the Jetson.
- The min-jerk QP with C3 continuity and sampled corridor constraints.
- Time allocation that uses an exact property of this QP: scaling all Tᵢ by k scales v, a and j by 1/k, 1/k² and 1/k³ and leaves the shape unchanged. So the global scale is set exactly, and only the relative allocation is iterated. The paper's stretch-only Eq. 5 did not converge on our 16-piece paths.
- Selected with `planner: sfc` (mission parameter) or `--planner sfc` (SIL).
- **Safety argument (unit-tested):** each obstacle point is removed by some half-plane that it lies beyond, so after shrinking, every corridor point is at least r from every obstacle point.

**Result on our course:**

| | fillet (default) | SFC min-jerk | SFC curve re-timed |
|---|---|---|---|
| window | 20.2 s | 27.2 s | 22.7 s |
| gate | 21.5 s | 32.3 s | 25.3 s |

- SIL, window route, 3 seeds: all clean, pad error 1.0–1.3 cm (fillet: about 2 cm), course 29.2 s (fillet: 22.2 s).
- **Why slower:** a min-jerk polynomial cannot hold the speed limit the way a time-optimal profile does, and its cost does not reward path length, so the curve swings wide inside the corridor (+1.2 m on the window route, +2.8 m on the gate route).
- **Second drawback here:** the curve bulges inside the dark tube instead of staying on the tube axis (`figures/sfc_corridor.png`), which wastes the drift margin.
- **Verdict:** keep `fillet` for scored runs. The SFC is good presentation material (certified-safe corridor, the professor's method). It is the right tool if we ever have to replan mid-flight around something unexpected, since the paper's 50–300 ms replanning is its real strength.

## 2. Liu, Atanasov, Mohta, Kumar — "Search-based Motion Planning for Quadrotors using Linear Quadratic Minimum Time Control", arXiv 1709.05401 / IROS 2017

**Idea.**
- The state is (p, v, …). Motion primitives are constant controls held for τ, and together they form a lattice.
- A* searches that lattice to minimize J + ρT = Σ(‖u‖² + ρ)τ.
- **Heuristics:**
  - h₁ = ρ·‖Δp‖∞/v_max, the minimum time.
  - h₂ = the optimal cost of the *unconstrained* Linear Quadratic Minimum Time problem (Prop. 3), found from the roots of a polynomial in T. The paper's Fig. 5 shows h₂ cutting expansions from 1282 to 376.
- §IV refines the primitive path with an unconstrained min-jerk QP through its waypoints.

**Built:** `dh_core/planner_mpl.py` already implemented the search with h₁. It now has **h₂** as the default heuristic (`MPLParams.heuristic = "lqmt"`). Our goal is a position with *free* arrival velocity, so I used the free-end form of Prop. 3:
- The transversality condition gives u(T) = 0, so J* = 3‖Δp − v₀T‖²/T³.
- C(T) = J* + ρT is minimized over T ≥ T̄ using the real roots of ρT⁴ − 3|v₀|²T² + 12(Δp·v₀)T − 9|Δp|² = 0.
- Δp is shrunk by the goal tolerance so the heuristic stays admissible for the goal disc.
- Unit test 11 checks it against a brute-force minimum.

**Result:** the same optimal trajectory durations, with **2.7–3.5× fewer expansions**:

| route | h₁ | h₂ |
|---|---|---|
| window | 29,076 expansions / 35.9 s | 10,787 / 14.6 s |
| gate | 14,316 / 17.1 s | 4,052 / 5.4 s |

This reproduces the paper's Fig. 5 on our course. MPL remains slower to fly (24.6 s and 27.3 s) than the fillet planner, because its velocity lattice is coarse.

## 3. Kaufmann, Bauersfeld, Loquercio, Müller, Koltun, Scaramuzza — "Champion-level drone racing using deep reinforcement learning" (Swift), Nature 620:982–987, 2023

**Idea.**
- **Perception:** VIO plus a CNN gate detector, fused in a Kalman filter.
- **Control:** a 2×128 MLP policy outputs collective thrust and body rates at 100 Hz. It is trained with PPO in simulation, with a reward for progress toward the next gate plus a term for keeping the next gate in the camera's view.
- **Sim-to-real:** the gap is closed with *residual models fitted to real flights*. Perception residuals are modelled with Gaussian processes (stochastic); dynamics residuals with k-nearest-neighbour regression (largely deterministic).
- The paper notes that approaches relying on precomputed paths are sensitive to noisy perception and dynamics.

**What transfers to DroneHacks (and what doesn't):**
- **Not feasible:** training an RL racing policy. It needs a high-fidelity simulator, motion-capture data and days of training. The Sky Beta interface also accepts velocity/position setpoints, not body rates.
- **Transferable idea 1, residuals from real data:** after the first real flights, fit the *actual* velocity-tracking lag and bias from the bag. Compare `/dh/pose_map` against the planned velocity; `tools/analyze_bag.py` already extracts both. Then set the SIL's `SimParams` (lag τ, velocity noise) and the follower's `v_lead` to the measured values, instead of the current guess τ ≈ 0.3 s. This is the same "close the sim-to-real gap with measured residuals" step Swift uses, in its simplest form.
- **Transferable idea 2, perception-aware heading:** Swift is rewarded for keeping the next gate in view, because seeing it improves pose estimates. Our yaw already follows the direction of travel, which keeps the forward stereo pointed at upcoming walls (good for the localizer). The one place this matters is the tube exit: look along the tube axis so the camera re-acquires texture as early as possible. The axis anchors already make the heading match the axis there.
- **Supports our design choice:** Swift's analysis says precomputed-path methods break when the state estimate is noisy. Our answer is the map-based localizer, which corrects the estimate rather than the plan. The final SIL shows it is the difference between 6/10 and 10/10 clean runs on the window route, and between 1/10 and 10/10 on the gate route.
