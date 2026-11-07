# Planner comparison on the official course (v_max 0.85 m/s, a_max 1.5 m/s^2, same A* skeleton)

Duration is the planned course time (launch to pad hover). Clearance is the drone centre's distance to the nearest obstacle (robot radius 0.27 m). Compute time is on a laptop CPU.

| route | method | length (m) | duration (s) | min clearance (m) | compute (s) |
|---|---|---|---|---|---|
| window | fillet + time-optimal profile (default) | 16.20 | 20.2 | 0.354 | 1.3 |
| window | SFC + min-jerk QP (16 polygons) | 17.40 | 27.2 | 0.292 | 5.6 |
| window | SFC curve re-timed with the time-optimal profile | 17.26 | 22.7 | 0.300 | 5.6 |
| window | motion primitives, h1 min-time heuristic (29076 expansions) | 16.17 | 24.6 | 0.316 | 35.9 |
| window | motion primitives, h2 LQMT heuristic (10787 expansions) | 16.05 | 24.6 | 0.316 | 14.6 |
| gate | fillet + time-optimal profile (default) | 16.75 | 21.5 | 0.354 | 1.4 |
| gate | SFC + min-jerk QP (24 polygons) | 19.57 | 32.3 | 0.300 | 12.4 |
| gate | SFC curve re-timed with the time-optimal profile | 19.44 | 25.3 | 0.300 | 12.4 |
| gate | motion primitives, h1 min-time heuristic (14316 expansions) | 17.00 | 27.3 | 0.316 | 17.1 |
| gate | motion primitives, h2 LQMT heuristic (4052 expansions) | 16.68 | 27.3 | 0.316 | 5.4 |

Notes:
- SFC is slower because a min-jerk polynomial cannot hold the speed limit the way a time-optimal profile can, and min-jerk does not reward path length, so the curve swings wide inside the corridor. Its strengths are a certified-safe convex corridor and C3 smoothness, which matter for receding-horizon replanning in unknown maps (the paper's use case). Here the map is known in advance.
- The LQMT heuristic h2 returns the same optimal cost as h1 with 2.7-3.5x fewer expansions, as in the paper's Fig. 5.
- Default stays `planner: fillet`.
