# Sim controllers

Each folder here is one algorithm to fly in the sim:

```sh
./fly-sim.sh controllers/waypoints
```

A folder needs `controller.py` defining:

```python
class Controller:
    def __init__(self, course):      # course: dict from sim/mission/course-window-x500.json
        ...
    def step(self, obs) -> dict:     # called on every new state estimate (~20-50 Hz sim time)
        return {"velocity": (vx, vy, vz), "yaw": yaw_or_None, "done": False}
```

`obs` (ENU world frame: x east, y north, z up; metres, seconds, radians):

| key | value |
|---|---|
| `t` | sim time of the state estimate |
| `position`, `velocity` | PX4's estimate (x, y, z), not ground truth |
| `yaw` | heading, 0 = east, counter-clockwise |
| `depth` | latest forward depth image, H x W metres (z-depth, 0 = no return), or None |
| `depth_t` | sim time of that depth image |
| `tof_range_m` | downward range sensor |

Return `velocity` in m/s (capped at the course's `max_speed`), `yaw` to turn toward
(rate-limited by the runner) or None to keep heading, and `done: True` to land.
Optional: `phase` (a label for the logs), `target` (for the logs), `fault` (a string aborts and lands).

The runner does takeoff, a 3 s hover check, landing on the pad and all safety checks before
and after `step()`. An exception in `step()` aborts and lands. Code here can import `refnav`
(the repository root is on the path) and the sim's `mission.*` helpers.

The depth camera: 384 x 256, 90 deg horizontal FOV, 20 Hz, 0.34 m ahead of the body, facing forward.
