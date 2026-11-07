# Planner comparison suite (software-in-the-loop, real node code)

Same seeds (101, 102, 103) for every planner, localizer on, tube flown at full speed, PX4 cap 1.0 m/s unless noted. Simulator: PX4-like velocity loop with a 3 m/s^2 horizontal acceleration limit and 0.12 s attitude lag, altitude drift, ray-cast forward clearance driving the stereo slowdown, and 3-D wall contact at the true height. All planners share one 1.5 m/s^2 total-acceleration budget (fillet: combined limit; SFC: |a| limit; MPL: |u| <= a_max). 'fillet, old (arcs)' is the previous fillet planner, kept for comparison. Points use the official finished-run formula with +5 s per touch; T* = fastest finish in that test (all teams = these runs). 'Clean' = finished on the pad with no wall touch; 'crashed' = wall contact deeper than 10 cm (the sim has no collision physics). Plan time is laptop CPU; expect ~2-3x on the Orin Nano.

## Test A: nominal conditions

| planner | route | clean / finished / crashed / runs | course time mean (min-max) s | touches/run | worst clearance m | pad error m | worst estimate error m | max speed m/s | plan time s | **mean points** |
|---|---|---|---|---|---|---|---|---|---|---|
| fillet, old (arcs) | window | 0 / 3 / 0 / 3 | 23.9 (23.8-24.0) | 2.67 | -0.028 | 0.014 | 0.345 | 0.99 | 6.6 | **53.0** |
| fillet, old (arcs) | gate | 0 / 3 / 0 / 3 | 26.2 (26.1-26.4) | 3.00 | -0.028 | 0.041 | 0.345 | 0.99 | 8.8 | **46.8** |
| fillet, new (clothoids) | window | 3 / 3 / 0 / 3 | 24.3 (24.1-24.4) | 0.00 | +0.010 | 0.018 | 0.230 | 0.99 | 7.2 | **59.9** |
| fillet, new (clothoids) | gate | 3 / 3 / 0 / 3 | 26.6 (26.5-26.7) | 0.00 | +0.010 | 0.034 | 0.231 | 0.98 | 8.1 | **53.1** |
| sfc | window | 3 / 3 / 0 / 3 | 32.5 (32.4-32.6) | 0.00 | +0.052 | 0.023 | 0.322 | 0.99 | 38.2 | **54.8** |
| sfc | gate | 3 / 3 / 0 / 3 | 37.4 (37.3-37.4) | 0.00 | +0.052 | 0.021 | 0.227 | 0.93 | 70.3 | **47.9** |
| mpl | window | 3 / 3 / 0 / 3 | 29.3 (29.2-29.4) | 0.00 | +0.010 | 0.007 | 0.254 | 0.92 | 39.2 | **56.5** |
| mpl | gate | 3 / 3 / 0 / 3 | 32.8 (32.8-32.9) | 0.00 | +0.010 | 0.020 | 0.254 | 0.92 | 43.2 | **49.7** |

## Test B: harsh conditions (2.5x drift, 2x tube blackout error)

| planner | route | clean / finished / crashed / runs | course time mean (min-max) s | touches/run | worst clearance m | pad error m | worst estimate error m | max speed m/s | plan time s | **mean points** |
|---|---|---|---|---|---|---|---|---|---|---|
| fillet, old (arcs) | window | 0 / 1 / 2 / 3 | 24.0 (23.9-24.1) | 3.33 | -0.240 | 1.764 | 1.266 | 0.99 | 7.1 | **18.2** |
| fillet, old (arcs) | gate | 0 / 2 / 1 / 3 | 26.3 (26.2-26.4) | 3.33 | -0.240 | 1.771 | 1.466 | 0.95 | 7.5 | **31.5** |
| fillet, new (clothoids) | window | 1 / 1 / 2 / 3 | 24.9 (24.9-24.9) | 0.67 | -0.240 | 2.267 | 0.980 | 0.99 | 6.9 | **20.0** |
| fillet, new (clothoids) | gate | 1 / 1 / 2 / 3 | 27.3 (27.3-27.3) | 0.67 | -0.240 | 2.348 | 1.708 | 0.99 | 7.4 | **17.8** |
| sfc | window | 1 / 1 / 2 / 3 | 32.8 (32.8-32.8) | 0.67 | -0.240 | 3.618 | 1.479 | 0.95 | 36.4 | **18.4** |
| sfc | gate | 1 / 1 / 2 / 3 | 38.1 (37.6-38.7) | 2.00 | -0.240 | 0.057 | 0.655 | 0.94 | 73.8 | **16.1** |
| mpl | window | 0 / 0 / 3 / 3 | 31.6 (29.5-32.8) | 1.67 | -0.240 | 0.016 | 0.936 | 0.94 | 39.5 | **0.0** |
| mpl | gate | 1 / 1 / 2 / 3 | 34.8 (33.0-36.5) | 1.33 | -0.240 | 1.233 | 0.930 | 0.92 | 41.4 | **16.7** |

## Test C: changed layout (3 perturbed courses, map updated)

| planner | route | clean / finished / crashed / runs | course time mean (min-max) s | touches/run | worst clearance m | pad error m | worst estimate error m | max speed m/s | plan time s | **mean points** |
|---|---|---|---|---|---|---|---|---|---|---|
| fillet, old (arcs) | window | 0 / 3 / 0 / 3 | 24.8 (23.2-25.7) | 1.33 | -0.016 | 0.014 | 0.321 | 1.00 | 8.8 | **55.7** |
| fillet, old (arcs) | gate | 0 / 3 / 0 / 3 | 28.1 (26.4-30.6) | 2.00 | -0.040 | 0.032 | 0.321 | 0.99 | 9.3 | **47.9** |
| fillet, new (clothoids) | window | 2 / 3 / 0 / 3 | 26.2 (24.4-27.3) | 0.33 | -0.016 | 0.024 | 0.091 | 0.98 | 8.8 | **57.7** |
| fillet, new (clothoids) | gate | 1 / 3 / 0 / 3 | 31.8 (29.1-36.6) | 0.67 | -0.016 | 0.040 | 0.091 | 0.98 | 10.1 | **48.9** |
| sfc | window | 1 / 3 / 0 / 3 | 30.8 (26.9-34.1) | 2.00 | -0.040 | 0.026 | 0.322 | 0.97 | 94.2 | **53.0** |
| sfc | gate | 1 / 3 / 0 / 3 | 34.2 (29.1-37.4) | 1.33 | -0.040 | 0.042 | 0.172 | 0.93 | 215.8 | **47.4** |
| mpl | window | 1 / 2 / 1 / 3 | 29.8 (26.9-31.3) | 0.67 | -0.128 | 0.016 | 0.563 | 0.97 | 47.5 | **37.0** |
| mpl | gate | 1 / 3 / 0 / 3 | 32.9 (29.1-39.8) | 0.67 | -0.016 | 0.038 | 0.174 | 0.96 | 228.2 | **48.6** |

## Test S: speed cap (fillet planner, window route, planned speed = 85 % of the cap)

| PX4 speed cap m/s | planned speed m/s | clean / finished / crashed / runs | course time mean (min-max) s | touches/run | worst clearance m | worst estimate error m | max speed flown m/s | **mean points** |
|---|---|---|---|---|---|---|---|---|
| 1.0 | 0.85 | 3 / 3 / 0 / 3 | 24.3 (24.1-24.4) | 0.00 | +0.010 | 0.230 | 0.99 | **59.9** |
| 1.5 | 1.27 | 0 / 3 / 0 / 3 | 20.7 (20.5-21.0) | 1.00 | -0.016 | 0.288 | 1.40 | **58.8** |
| 2.0 | 1.70 | 0 / 3 / 0 / 3 | 19.9 (19.7-20.0) | 1.00 | -0.016 | 0.178 | 1.77 | **59.4** |

## What this run shows (stricter simulator, equal 1.5 m/s² budget; 3 runs per cell)

- **The new fillet beats the old one everywhere it matters:**
  - Nominal: 6/6 clean against 0/6, 59.9 against 53.0 points on the window route.
  - The old circular arcs touch walls 2–3 times per run at the maze corners, where curvature jumps meet the acceleration-limited drone.
  - The clothoids cost about 0.4 s and remove every nominal touch.
- **New fillet against SFC and MPL:** new fillet is best in nominal and changed-layout conditions on both routes. SFC and MPL are now 5–11 s slower, and their plan times (38–228 s on a laptop) would be impractical on the Orin.
- **Harsh drift now breaks every planner** (1/3 clean at best). Once altitude drift and 3-D contact are modelled, the limit is position error, not the planner. Next step: drift-aware margins, re-planning, or a LiDAR.
- **Speed cap:** with acceleration limits modelled, raising the cap to 1.5–2.0 m/s saves 3.6–4.4 s but adds a touch per run, so points don't improve (58.8 / 59.4 against 59.9). This reverses the earlier idealised result; stay at 1.0 m/s.
