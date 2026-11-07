# Software-in-the-loop results (real mission/localizer node code, simulated drone)

Odom frame rotated 0.7 rad and offset from the course frame; noise 1 cm; drift 0.01 m/sqrt(m); tube blackout velocity error std 0.08 m/s; PX4-like 1.0 m/s velocity cap with 0.3 s lag; synthetic 80-degree forward scans (blind in the tube) when the localizer is on. Contact = drone centre within 0.24 m of an obstacle.

| route | localizer | tube speed (m/s) | seeds | clean runs | mean course time (s) | mean touches | worst clearance (m) | mean pad error (m) |
|---|---|---|---|---|---|---|---|---|
| window | on | 0.85 | 10 | 10 | 22.2 | 0.00 | +0.029 | 0.019 |
| window | off | 0.85 | 10 | 6 | 24.1 | 0.50 | -0.190 | 0.228 |
| gate | on | 0.85 | 10 | 10 | 23.5 | 0.00 | +0.029 | 0.035 |
| gate | off | 0.85 | 10 | 1 | 25.3 | 2.00 | -0.240 | 0.227 |
