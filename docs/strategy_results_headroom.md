# Experiment 2 - planned-speed headroom below the PX4 cap (cap = 1.00 m/s, N=50)

dark_vel_err = velocity error while visual odometry is lost in the tube (0.08 = no aiding, 0.03 = e.g. IMU fusion / ToF aiding).

| route | plan fraction | dark vel err | re-anchor | planned time (s) | mean time incl. penalties (s) | P(touch) | P(crash) |
|---|---|---|---|---|---|---|---|
| gate | 1.00 | 0.08 | False | 25.8 | 30.5 | 0.68 | 0.60 |
| gate | 1.00 | 0.08 | True | 25.8 | 37.8 | 0.72 | 0.18 |
| gate | 1.00 | 0.03 | False | 25.8 | 31.9 | 0.56 | 0.36 |
| gate | 1.00 | 0.03 | True | 25.8 | 35.1 | 0.62 | 0.12 |
| gate | 0.85 | 0.08 | False | 27.9 | 37.0 | 0.82 | 0.56 |
| gate | 0.85 | 0.08 | True | 27.9 | 37.0 | 0.64 | 0.10 |
| gate | 0.85 | 0.03 | False | 27.9 | 35.0 | 0.64 | 0.38 |
| gate | 0.85 | 0.03 | True | 27.9 | 35.6 | 0.64 | 0.16 |
| gate | 0.70 | 0.08 | False | 31.0 | 41.3 | 0.82 | 0.62 |
| gate | 0.70 | 0.08 | True | 31.0 | 36.3 | 0.48 | 0.18 |
| gate | 0.70 | 0.03 | False | 31.0 | 37.3 | 0.52 | 0.26 |
| gate | 0.70 | 0.03 | True | 31.0 | 36.4 | 0.34 | 0.04 |
| window | 1.00 | 0.08 | False | 24.6 | 29.7 | 0.62 | 0.50 |
| window | 1.00 | 0.08 | True | 24.6 | 26.7 | 0.30 | 0.10 |
| window | 1.00 | 0.03 | False | 24.6 | 27.8 | 0.26 | 0.10 |
| window | 1.00 | 0.03 | True | 24.6 | 27.1 | 0.24 | 0.02 |
| window | 0.85 | 0.08 | False | 26.7 | 32.3 | 0.74 | 0.64 |
| window | 0.85 | 0.08 | True | 26.7 | 31.9 | 0.58 | 0.10 |
| window | 0.85 | 0.03 | False | 26.7 | 31.2 | 0.56 | 0.32 |
| window | 0.85 | 0.03 | True | 26.7 | 31.4 | 0.54 | 0.18 |
| window | 0.70 | 0.08 | False | 29.8 | 35.5 | 0.66 | 0.54 |
| window | 0.70 | 0.08 | True | 29.8 | 33.3 | 0.30 | 0.06 |
| window | 0.70 | 0.03 | False | 29.8 | 33.8 | 0.34 | 0.14 |
| window | 0.70 | 0.03 | True | 29.8 | 32.0 | 0.10 | 0.00 |
