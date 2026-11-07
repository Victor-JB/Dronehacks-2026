# dronehacks_nav — `main`, the code the drone flies

`main` is the team's mission as it runs on the skybeta drone (`skybeta-02`, Jetson Orin Nano,
PX4 v1.16.2). The simulator lives on `sim`; new policies come from it through `promote.sh` (`SYSTEM.md` §10). **`SYSTEM.md` explains how the whole system works**; this
file is the quick start.

> **Before flying, read `PICKUP.md`:** with depth on, VIO reaches PX4 ~290 ms late and PX4 has no
> position. That is the next thing to fix.

| Path | What it is |
|---|---|
| `refnav/core.py`, `refnav/depth.py` | The mission's flight logic and depth processing. The mission logic; the sim flies this exact file (below) |
| `dronehacks_nav/skybeta_node.py` | The ROS shell for the drone: cuVSLAM odometry + `/skybeta/state` in, `/skybeta/setpoint/pose` out |
| `dronehacks_nav/frames.py` | ENU/FLU (skybeta) ↔ NED/FRD (refnav.core) conversions |
| `dronehacks_nav/motion_check.py` | Freeze if commanded motion does not show up in odometry, or VIO height disagrees with the rangefinder |
| `dronehacks_nav/mapping.py`, `map_node.py` | Voxel occupancy map from depth + pose: `/map/occupied`, `/map/image` |
| `dronehacks_nav/camera_web.py` | Camera, depth and map as a web page on port 8080 |
| `launch/mission.launch.py` | `skybeta_flight_interface` + the mission node |
| `config/drone.yaml` | ROS parameters of the mission node |
| `modes.json`, `apply_mode.py`, `route.json` | What to fly: pick a mode in `modes.json`, run `python3 apply_mode.py`, which writes `route.json` |
| `scripts/fly.sh` | Start/stop a flight session on the drone |
| `skybeta_overrides/` | Team image that turns on the OAK's stereo depth |
| `tests/` | `python3 -m pytest -q tests`: mission, frames, motion check, map; no ROS needed |

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

**Depth:** `refnav.core` refuses to fly without depth (it never translates without
observed free space). skybeta ships no depth image, so the team image below turns
on the OAK's own stereo depth, `/oak/stereo/image_raw`.

## Deploying

The drone holds a clone at `~/team_ws/src/dronehacks`, checked out on `main`. From the laptop:

```sh
git push drone main          # updates the drone's working tree in place
```

Then, on the drone, the stack rebuilds the team workspace when it starts, or by
hand inside the container:

```sh
ssh dronehacks@skybeta-02.local
docker exec -it skybeta-stack bash
team_build.sh --packages-select dronehacks_nav
```

Python edits need no rebuild (`--symlink-install`); new files, launch files and
config do (`fly.sh start` rebuilds anyway).

## Flying: `scripts/fly.sh`

The drone boots with `COMPETITOR_PKG=manual` in `/etc/skybeta/drone.env`: the stack
(VIO, PX4 link) comes up by itself, and the `competitor` window only builds and
idles. Nothing of ours runs until, on the drone:

```sh
~/team_ws/src/dronehacks/scripts/fly.sh start    # rebuild, then mission + bag + foxglove + map + camera page
~/team_ws/src/dronehacks/scripts/fly.sh attach   # watch the mission (Ctrl-b d leaves, Ctrl-b n next window)
~/team_ws/src/dronehacks/scripts/fly.sh stop     # on the ground: stops them all, closes the bag
```

- Camera, depth and map: `http://skybeta-02.local:8080/` in a browser (`dronehacks_nav/camera_web.py`).
- Foxglove (pose, state, `/map/occupied`; no images): `ssh -N -L 8765:localhost:8765 dronehacks@skybeta-02.local`
  on the laptop, then `ws://localhost:8765`.
- Bags: `~/team_ws/bags/<date-time>/` (mcap; depth, odometry, state, setpoints, PX4).

After `start`, wait for `READY` in the mission window; then the safety pilot arms on
Altitude and switches to Position. `stop` while airborne kills the flight interface
and PX4 falls back to Position under the pilot.

First runs: props off, then low and tethered. Nothing on this stack has flown yet.

## Depth (team image)

skybeta ships the OAK with depth off. `skybeta_overrides/` holds a team image that
swaps in a camera config with on-device stereo depth (`/oak/stereo/image_raw`,
320x200, 30 Hz). On the drone `~/team_ws/Dockerfile` links to it; rebuild with
`~/skybeta/host/run.sh build` after every skybeta update. To go back to the stock
camera: `rm ~/team_ws/Dockerfile && docker rm -f skybeta-stack` (on the ground).

## Keeping branches in step

Mission changes made on `main` come over with `git merge main` (or
`git cherry-pick`). Expect conflicts only on files this branch deleted
(`refnav/node.py`, `onboard/`, …): those deletions are deliberate, so keep them
deleted and port any interface change into `dronehacks_nav/skybeta_node.py`.
