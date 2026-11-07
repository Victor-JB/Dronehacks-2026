# dronehacks_nav — the `jetson` branch

This branch is the team's mission pruned to run on the skybeta drone
(`skybeta-02`, Jetson Orin Nano, PX4 v1.16.2). The simulator project lives on
`main`; keep it there.

| Path | What it is |
|---|---|
| `refnav/core.py`, `refnav/depth.py` | The mission's flight logic and depth processing. **Kept byte-identical to `main`**, so mission work merges either way |
| `dronehacks_nav/skybeta_node.py` | The ROS shell for the drone: cuVSLAM odometry + `/skybeta/state` in, `/skybeta/setpoint/pose` out |
| `dronehacks_nav/frames.py` | ENU/FLU (skybeta) ↔ NED/FRD (refnav.core) conversions |
| `launch/mission.launch.py` | `skybeta_flight_interface` + the mission node |
| `config/drone.yaml`, `route.json` | Mission parameters and route |
| `tests/` | `python3 -m pytest -q tests`: mission logic and frame conversions, no ROS needed |

## How it flies on skybeta

The drone's stack (`~/skybeta`, owned by the organisers) already runs the
OAK-D, cuVSLAM, the vision bridge into PX4, and `skybeta_flight_interface`,
which owns everything on `/fmu/in/*`. Read `~/skybeta/docs/competitor_guide.md`
on the drone. In short:

- **We never arm, never change mode, never land.** The safety pilot arms on
  Altitude, then switches to Position; if this node is streaming setpoints the
  flight interface enters Offboard and the mission takes off.
- The node streams only once the mission is ready (`READY` in the log), so
  arming before that leaves the pilot in Position.
- When the mission ends or wants to land it **holds position and asks the pilot
  to take over and land**: skybeta has no land command, and altitude is floored
  at `min_altitude_m` (0.3 m).
- Altitude is clamped to 0.3–3.0 m above the startup pose. **There is no
  horizontal geofence**: size the route to the space.

**Depth:** skybeta publishes no depth image, and `refnav.core` refuses to fly
without one (it never translates without observed free space). With the default
`depth_topic: ""` the mission waits, never streams, and gives up after
`wait_s`. A depth source (stereo from `/oak/{left,right}/image_mono`, or OAK-D
on-device depth) is the next piece of work.

## Deploying

The drone holds a clone at `~/team_ws/src/dronehacks`, checked out on `jetson`.
From the laptop:

```sh
git push drone jetson        # updates the drone's working tree in place
```

Then, on the drone, the stack rebuilds the team workspace when it starts, or by
hand inside the container:

```sh
ssh dronehacks@skybeta-02.local
docker exec -it skybeta-stack bash
team_build.sh --packages-select dronehacks_nav
```

Python edits need no rebuild (`--symlink-install`); new files, launch files and
config do. To have the stack's `competitor` window run this mission, set in
`/etc/skybeta/drone.env` (sudo) and restart the stack on the ground, drone still:

```sh
COMPETITOR_PKG=dronehacks_nav
COMPETITOR_LAUNCH=mission.launch.py
```

First runs: props off, then low and tethered. Nothing on this stack has flown yet.

## Keeping branches in step

Mission changes made on `main` come over with `git merge main` (or
`git cherry-pick`). Expect conflicts only on files this branch deleted
(`refnav/node.py`, `onboard/`, …): those deletions are deliberate, so keep them
deleted and port any interface change into `dronehacks_nav/skybeta_node.py`.
