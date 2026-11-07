# Experiment 3 - closed-loop map-relative aiding (partial; N=30 per row, PX4 cap 1.0 m/s, planned 0.85 m/s)

Model: simple Monte-Carlo simulator (`dh_core/sim.py`, built-in P+feed-forward controller, no velocity lead),
synthetic scans of the official course fed to the real `DistanceFieldMatcher`.
**Caveats:** this run used the *earlier* unsigned-distance matcher (later found to be biased and replaced by
the signed-distance version), and it timed out after five rows, so the Gate B rows and the LiDAR
"pessimistic" row are missing. Treat it as directional evidence only.

| route | aiding | drift | planned time (s) | mean time incl. penalties (s) | P(touch) | P(crash) |
|---|---|---|---|---|---|---|
| window | none | nominal | 26.7 | 35.3 | 0.63 | 0.47 |
| window | none | pessimistic | 26.7 | 34.7 | 0.90 | 0.77 |
| window | stereo80 (forward stereo-derived scan, blind in the tube) | nominal | 26.7 | 29.7 | 0.17 | 0.10 |
| window | stereo80 | pessimistic | 26.7 | 30.1 | 0.47 | 0.37 |
| window | lidar360 (2D LiDAR, works in the dark) | nominal | 26.7 | 28.3 | 0.07 | 0.00 |

Reading: map-relative scan matching is the single biggest reliability lever in this model, and a sensor that
keeps working inside the dark tube (an auxiliary 2D LiDAR such as the YDLIDAR T-mini Plus, $79-95, 45 g;
see the hardware survey in the research handoff) removes most of the remaining tunnel risk.
