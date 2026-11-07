# Monte-Carlo strategy results (N=40 per row)

Times include 6 s takeoff/landing overhead; touched runs add 5 s per touch. clr_p10 = 10th-percentile minimum clearance between the drone's physical radius (0.24 m) and the nearest obstacle.

| route | speed cap (m/s) | estimator drift | re-anchor after tunnel | planned time (s) | mean sim time (s) | P(touch) | P(crash) | clr_p10 (m) |
|---|---|---|---|---|---|---|---|---|
| gate | 0.50 | nominal | False | 38.6 | 43.6 | 0.88 | 0.82 | -0.090 |
| gate | 0.50 | nominal | True | 38.6 | 39.9 | 0.15 | 0.10 | -0.045 |
| gate | 0.50 | pessimistic | False | 38.6 | 49.1 | 1.00 | 0.95 | -0.090 |
| gate | 0.50 | pessimistic | True | 38.6 | 51.7 | 0.93 | 0.68 | -0.090 |
| gate | 0.75 | nominal | False | 29.9 | 34.2 | 0.50 | 0.42 | -0.090 |
| gate | 0.75 | nominal | True | 29.9 | 32.6 | 0.20 | 0.07 | -0.040 |
| gate | 0.75 | pessimistic | False | 29.9 | 45.4 | 1.00 | 0.95 | -0.090 |
| gate | 0.75 | pessimistic | True | 29.9 | 36.3 | 0.65 | 0.53 | -0.090 |
| gate | 1.00 | nominal | False | 25.8 | 30.9 | 0.70 | 0.55 | -0.090 |
| gate | 1.00 | nominal | True | 25.8 | 36.2 | 0.65 | 0.23 | -0.060 |
| gate | 1.00 | pessimistic | False | 25.8 | 31.9 | 0.93 | 0.90 | -0.090 |
| gate | 1.00 | pessimistic | True | 25.8 | 31.9 | 0.75 | 0.55 | -0.090 |
| gate | 1.25 | nominal | False | 24.1 | 37.1 | 0.95 | 0.72 | -0.090 |
| gate | 1.25 | nominal | True | 24.1 | 45.3 | 1.00 | 0.53 | -0.060 |
| gate | 1.25 | pessimistic | False | 24.1 | 44.6 | 0.97 | 0.95 | -0.090 |
| gate | 1.25 | pessimistic | True | 24.1 | 40.7 | 0.97 | 0.82 | -0.090 |
| gate | 1.50 | nominal | False | 23.5 | 39.2 | 1.00 | 0.65 | -0.062 |
| gate | 1.50 | nominal | True | 23.5 | 46.5 | 1.00 | 0.75 | -0.060 |
| gate | 1.50 | pessimistic | False | 23.5 | 40.0 | 1.00 | 0.95 | -0.090 |
| gate | 1.50 | pessimistic | True | 23.5 | 38.7 | 0.95 | 0.88 | -0.090 |
| window | 0.50 | nominal | False | 37.4 | 45.2 | 0.60 | 0.45 | -0.090 |
| window | 0.50 | nominal | True | 37.4 | 38.6 | 0.10 | 0.07 | +0.005 |
| window | 0.50 | pessimistic | False | 37.4 | 46.1 | 0.85 | 0.80 | -0.090 |
| window | 0.50 | pessimistic | True | 37.4 | 41.8 | 0.53 | 0.42 | -0.090 |
| window | 0.75 | nominal | False | 28.6 | 33.6 | 0.45 | 0.25 | -0.062 |
| window | 0.75 | nominal | True | 28.6 | 29.7 | 0.10 | 0.05 | +0.007 |
| window | 0.75 | pessimistic | False | 28.6 | 33.8 | 0.88 | 0.82 | -0.090 |
| window | 0.75 | pessimistic | True | 28.6 | 32.8 | 0.60 | 0.40 | -0.090 |
| window | 1.00 | nominal | False | 24.6 | 29.8 | 0.62 | 0.50 | -0.083 |
| window | 1.00 | nominal | True | 24.6 | 27.8 | 0.40 | 0.10 | -0.042 |
| window | 1.00 | pessimistic | False | 24.6 | 33.3 | 0.90 | 0.82 | -0.090 |
| window | 1.00 | pessimistic | True | 24.6 | 29.9 | 0.57 | 0.40 | -0.090 |
| window | 1.25 | nominal | False | 22.7 | 35.7 | 0.97 | 0.65 | -0.082 |
| window | 1.25 | nominal | True | 22.7 | 36.6 | 1.00 | 0.35 | -0.060 |
| window | 1.25 | pessimistic | False | 22.7 | 34.5 | 1.00 | 0.90 | -0.090 |
| window | 1.25 | pessimistic | True | 22.7 | 32.2 | 0.93 | 0.78 | -0.090 |
| window | 1.50 | nominal | False | 22.1 | 34.1 | 1.00 | 0.75 | -0.090 |
| window | 1.50 | nominal | True | 22.1 | 39.1 | 1.00 | 0.72 | -0.060 |
| window | 1.50 | pessimistic | False | 22.1 | 40.1 | 1.00 | 0.95 | -0.090 |
| window | 1.50 | pessimistic | True | 22.1 | 32.5 | 0.97 | 0.78 | -0.090 |
