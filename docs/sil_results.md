# Software-in-the-loop results (real mission/localizer node code, simulated drone)

Odom frame rotated 0.7 rad and offset from the course frame; noise 1 cm; drift 0.01 m/sqrt(m); tube blackout velocity error std 0.08 m/s; PX4-like 1.0 m/s velocity cap with 0.3 s lag; synthetic 80-degree forward scans (blind in the tube) when the localizer is on. Contact = drone centre within 0.24 m of an obstacle.

| route | localizer | tube speed (m/s) | seeds | clean runs | mean course time (s) | mean touches | worst clearance (m) | mean pad error (m) |
|---|---|---|---|---|---|---|---|---|
| window | on | 0.85 | 10 | 9 | 22.2 | 0.20 | -0.060 | 0.011 |
| window | on | 0.60 | 10 | 8 | 23.2 | 0.60 | -0.190 | 0.014 |
| window | off | 0.85 | 10 | 7 | 22.2 | 0.70 | -0.240 | 0.194 |
| window | off | 0.60 | 10 | 6 | 23.0 | 1.10 | -0.240 | 0.236 |
| gate | on | 0.85 | 10 | 9 | 23.4 | 0.20 | -0.060 | 0.030 |
| gate | on | 0.60 | 10 | 7 | 24.5 | 1.00 | -0.190 | 0.025 |
| gate | off | 0.85 | 10 | 1 | 25.4 | 3.10 | -0.240 | 0.235 |
| gate | off | 0.60 | 10 | 1 | 26.2 | 2.70 | -0.240 | 0.258 |
