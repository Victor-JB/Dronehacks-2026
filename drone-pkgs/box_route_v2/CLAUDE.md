# box_route_v2 — v2 of box_route (v1 = `~/team_ws/src/box_route`, untouched: the fallback)

**Switch the stack to v2 / back to v1** (ask first; then restart the stack on the ground, still):
```bash
sudo sed -i 's/^COMPETITOR_PKG=.*/COMPETITOR_PKG=box_route_v2/' /etc/skybeta/drone.env   # v2
sudo sed -i 's/^COMPETITOR_PKG=.*/COMPETITOR_PKG=box_route/'    /etc/skybeta/drone.env   # back to v1
```
Both use `COMPETITOR_LAUNCH=box_mission.launch.py`. Edit v2's route in
`~/team_ws/src/box_route_v2/config/route.yaml`. Live view / bag: `~/team_ws/src/box_route_v2/scripts/fly.sh start`.
Both publish `/box_route/status` (only one ever runs).

**What v2 changes (2026-11-08, from the morning bags 055027 / 055648 / 065240 / 065751):**
- `rotate_to_heading: false`: the route is fixed to the course (the stack-start heading), not rotated to
  the nose at Offboard start. Run 1 started 4 deg turned: v1 swung the route 0.3 m and it hit the
  tunnel-exit wall. Place the drone on the mark facing down the course BEFORE the stack starts.
- `straight: true` per waypoint: the leg into it has no early turn and moves only lined up within 5 deg
  (the window: run 2 was still yawing as it clipped it).
- Waypoints 4-14 re-derived from today's walkthroughs (window crossed at y -4.855 instead of -5.00;
  back half follows the forward walk; ends at pad 1). Every leg is within 9 cm of a flown/walked track.
- Window 2: 1.2 m on waypoint 10 (climbs at 9, crosses on the leg 9 -> 10). Window 1 at y -4.80.
- **Sections after pad 1, each `enabled: true/false` in route.yaml** (`box_route_v2/mission_plan.py`):
  `pad1_hover` (hover 0.35 m over the pad, climb back, settle) -> `wall` (3-D waypoints over the wall at 2.2 m,
  right of the ladder; wall rules: `ignore_px4_height`, `vio_height_hold`, `on_vio_fault: hold|land`) ->
  `pad2_hover` -> `reverse` (wall back, pad 1 again, course backwards, land at take-off). All off = land on pad 1.
  It lands only at the last step of what is enabled. Bad values / typos reject the file (old settings kept).
- PX4 flies on the RANGEFINDER height (EKF2_HGT_REF=2, QGC params). Over the wall it may climb (sim worst
  case 3.2 m); it never went below 2.1 m over the 1.9 m wall. Hovers can't go under 0.3 m (flight_interface).
- Offline check without ROS (Mac or drone): `python3 test/sim_offline.py <route.yaml> [nose_deg] [--wall-fx=0.5] [--vio-low] [-v]`
  (`test/route_all_on.yaml` = everything enabled). Visual check on the Mac: `drone-logs/analysis/plot_check.py`.

The rest of this file is v1's, still accurate for v2 except paths (`box_route` -> `box_route_v2`).

# box_route — setup (for a human, or for Claude after `ssh` into the drone)

> **Claude:** do these steps in order on the drone's host shell. Ask the user before step 4
> (editing `/etc/skybeta/drone.env`) and step 5 (restarting the stack), and only restart with the
> drone **on the ground, disarmed and still**. Never run a second `flight_interface`.

## What it flies

> **FULL COURSE right now (2026-11-08): full-size route at 1.1 m, 0.25 m/s (0.15 m/s on wall legs).**
> **All route settings live in `config/route.yaml`** (height, scale, nominal + wall speed, how many
> waypoints to fly, every waypoint's fwd/left/wall). Edit and save on the host
> (`nano ~/team_ws/src/box_route/config/route.yaml`): the mission re-reads it within ~1 s **while
> disarmed** (never in flight; a change made while armed waits for the disarm). No rebuild, no
> restart. The log says `config loaded` or `config REJECTED: <why>` (old settings kept); the live
> view redraws the route and lists every waypoint's (x, y).

```
climb to 1.1 m -> 13 waypoints from the 2026-11-07 odom map (walk_1107) -> land at the last one
```

Waypoints (metres from the take-off point: x forward, y LEFT), copied from
`testFiles/test_files/my_mission.py` `ROUTE`. All at `HEIGHT_M` = 1.1 m (inside the window opening, ~0.88-1.52 m). **Tunnel overhang:** dronehacks'
notes put it ~0.15 m above 1.08 m, i.e. ~1.23 m: ~0.12 m of clearance at 1.1 m (1.2 m left ~3 cm).

| # | (fwd, left) | | # | (fwd, left) |
|---|---|---|---|---|
| 0 | (0, 0) climb | | 7 | (-3.035, -4.680) |
| 1 | (0.875, -0.260) | | 8 | (-3.265, -3.520) |
| 2 | (1.145, -4.100) | | 9 | (-3.465, -3.290) |
| 3 | (0.885, -4.810) | | 10 | (-3.545, -1.980) |
| 4 | (0.555, -5.040) | | 11 | (-3.175, -1.530) |
| 5 | (-0.215, -4.820) | | 12 | (-3.335, -0.930) |
| 6 | (-1.645, -5.090) | | 13 | (-3.355, -0.450) then land |

**Take off 1 ft ahead of the walkthrough's start, facing the same way as the walkthrough.**
The route is rotated to the nose heading when Offboard starts. It lands about 3.4 m behind and
0.45 m right of the take-off point. Then: hover 2 s, descend 0.3 m/s by twist, PX4 disarms ~2 s
after touchdown. No touchdown after 20 s: hovers, logs `Pilot, take over and land`.

A waypoint counts as reached within 0.3 m horizontally; height must be within 0.15 m too, but only
for the first 3 s there (after that it moves on and logs `height off by ...`), and only once
PX4 says airborne and VIO has it at least half way up.

Safety rules (from the 2026-11-07/08 flight data). **All the limits below, and `turn_early` (on/off) / `turn_ahead_m`, are set in `config/route.yaml`** (live-reloaded while disarmed, like the route); the numbers here are the defaults:

- **Won't start** if VIO's height differs from PX4's by more than 0.4 m (a drifted VIO made it climb
  to the 3 m ceiling). The live view shows the difference; the banner says REFUSED. Fix: land, then
  `docker rm -f skybeta-stack` on the ground.
- **Hovers 5 s at the climb point** before leg 1 (every take-off slid 0.2-0.4 m, then came back).
- **Heights come from PX4** (rangefinder): waypoint height checks and every hold. VIO reads 0.1-0.4 m low.
- **VIO watchdog -> stop the route and land** (logs `route stopped, LANDING`): no odometry for 1 s,
  a jump > 0.4 m in one frame, VIO vs PX4 > 0.5 m horizontally or > 0.6 m in height, or a waypoint
  not reached 15 s after the setpoint got there. All three crashes so far were VIO failing at a wall.
- **Turns while moving:** faces along each leg and blends toward the next leg's heading over the last
  0.5 m (at most half the leg), so the camera turns away from the wall ahead; at sharp corners the
  setpoint waits while it finishes the turn (45 deg/s).

It starts **only** when the drone arms first and Offboard comes on after (the pilot's switch to
Position). If Offboard is already on at arming (leftover PX4 state, e.g. after a kill-switch stop),
it logs `armed with Offboard already on (leftover state): NOT starting` and does nothing: disarm and
power-cycle the Pixhawk.
Route, height, speeds: `config/route.yaml` (see the top). Safety/turn constants: top of
`box_route/box_mission.py` (those need a stack restart, still no rebuild). `ROUTE` there is only the
fallback if route.yaml is missing or invalid at start-up.

## Files

```
box_route/
  package.xml, setup.py, setup.cfg, resource/box_route    ament_python package "box_route"
  box_route/box_mission.py                                the node (entry point box_mission)
  box_route/live_view.py                                  web live view on :8080 (entry point live_view)
  scripts/fly.sh                                          bag + foxglove + live view
  launch/box_mission.launch.py                            flight_interface (auto_offboard_on_arm, anchor_setpoints) + box_mission
```

## 1. Check the drone

```bash
cat ~/skybeta/VERSION                         # must be 0.3.3 or later
docker ps --format '{{.Names}} {{.Status}}'   # skybeta-stack must be Up
ls ~/team_ws/src/box_route                    # this package
```

## 2. Build (inside the container)

```bash
docker exec -it skybeta-stack bash
team_build.sh --packages-select box_route
source /root/team_ws/install/setup.bash
ros2 launch box_route box_mission.launch.py --show-args    # parses = good
exit
```

The **whole** of `~/team_ws/src` must build, or the stack launches nothing at power-up.

## 3. Sanity check the route (in the container: the host has no rclpy)

```bash
docker exec -w /root/team_ws/src/box_route skybeta-stack bash -lc \
  'source /root/team_ws/install/setup.bash && python3 -c "from box_route.box_mission import route_body; print(route_body())"'
# test: [(0.0, 0.0, 1.0), (0.21875, -0.065, 1.0), ... 14 points ..., (-0.83875, -0.1125, 1.0)]
```

## 4. Point the stack at it (ask first)

```bash
grep COMPETITOR /etc/skybeta/drone.env
sudo cp /etc/skybeta/drone.env /etc/skybeta/drone.env.bak
sudo sed -i 's/^COMPETITOR_PKG=.*/COMPETITOR_PKG=box_route/; s/^COMPETITOR_LAUNCH=.*/COMPETITOR_LAUNCH=box_mission.launch.py/' /etc/skybeta/drone.env
grep COMPETITOR /etc/skybeta/drone.env
```

**Short mode (waypoint 2 only):** `COMPETITOR_LAUNCH=box_wp2.launch.py` flies climb -> waypoint 1 ->
waypoint 2, then settles and lands at waypoint 2 (`last_waypoint: 2`; every other rule is the same).
The competitor log says `(MODE: stop and land at waypoint 2)`. Back to the full route:
`COMPETITOR_LAUNCH=box_mission.launch.py`.

To go back to the old mission: `COMPETITOR_PKG=test_files`, `COMPETITOR_LAUNCH=my_mission.launch.py`
(or restore `drone.env.bak`).

## 5. Restart the stack (ask first; on the ground, disarmed, still)

**PX4 must be connected first** (battery in, `"px4_link": true` in `/skybeta/state`). If the stack
starts before PX4 has a valid position, flight_interface logs `anchor_setpoints is true but EKF2 had
no valid local position` and VIO and PX4 heights disagree (flight 2026-11-08: ~0.24 m low).
If you see that warning, restart again once PX4 is up.

```bash
docker rm -f skybeta-stack        # comes back in ~10 s, re-zeroes the position origin
sleep 20
docker exec skybeta-stack tmux capture-pane -p -J -t skybeta:competitor | tail -20
```

Look for `course route: 1.1 m up, scale 1.0, 13 waypoints from the odom map, 0.25 m/s ...` (full) or `scale 0.25` (test) and no `team_ws build failed`.

## Live view, bag, Foxglove (`scripts/fly.sh`, ported from dronehacks main)

```bash
~/team_ws/src/box_route/scripts/fly.sh start          # bag + foxglove + live view (no depth: no extra load)
~/team_ws/src/box_route/scripts/fly.sh start --depth  # also depth + occupancy map (dronehacks_nav map_node)
~/team_ws/src/box_route/scripts/fly.sh stop           # on the ground, closes the bag cleanly
```

Live view: `http://skybeta-02.local:8080/`, view only (no buttons, publishes nothing). Top-down
odometry: green = track, yellow = drone + heading, blue dashed = route it wants to fly (dim =
preview from where it stands, before Offboard), orange = next waypoint, pink = current setpoint.
Plus the camera, PX4 armed/Offboard, PX4-vs-VIO height. The route and setpoint come from
`/box_route/status`, a read-only JSON the mission publishes at 5 Hz.

## 6. Fly

1. Place the drone 1 ft ahead of the walkthrough's start, facing the walkthrough's direction. Keep it still ~1 min.
2. Pilot arms on **Altitude**, then within a few seconds switches to **Position** → mission runs.
3. Take over any time: throttle to centre / move a stick. One run per arming.
4. Live view + bag: `~/team_ws/src/box_route/scripts/fly.sh start`, open http://skybeta-02.local:8080/,
   `... stop` after landing.

Watch the log: `docker exec -it skybeta-stack tmux attach -t skybeta` → `Ctrl-b 3`, leave with
`Ctrl-b d` (never Ctrl-C there).
