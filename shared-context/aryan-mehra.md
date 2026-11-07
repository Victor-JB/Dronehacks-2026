# Aryan Mehra — Agent Context

**Role:** Team Lead, Dead Reckoning

_Update this file every time your agent context changes. See `shared-context/README.md` for the rules._

---

## Hardcoded route (strategy)

We're hardcoding the maze route (team decision: costs up to 15 Generality points per rule 9.5,
but the alternative risks the 60-point Run score, so we're trading the smaller bucket). The route
was derived from the published course CAD (`dronehacks-course-v3.obj`, in `diagrams/` -- download
it yourself from the Connect challenge page's "Download the course" if you need the raw file) via
an occupancy grid + A*, launch mark to landing pad through Gate B (not the window bonus yet).

Current files, repo root of `Dronehacks-2026`, branch `hardcoded-route-draft`:
- `route.json` -- the live 26-leg route `refnav` actually loads, plus the tuned `Params` it needs
  to fly this maze (see "Known issue" below -- now solved, kept the name for history).
- `diagrams/route-plan.html` -- open it in a browser: scaled floor plan with every wall/boulder/
  tunnel/landing-pad, the planned path numbered waypoint-by-waypoint, and the leg table.
- `local_sim/` -- see below.

Open question before trusting the route against the real/cloud sim: does the FlyX Lab "Indoor
maze" world's geometry actually match this CAD file? Nothing in the repo confirms that; check by
eye (Follow/Depth view) before relying on it there.

## Local sim (FlyX Lab is flaky -- use this instead for fast iteration)

`local_sim/` is a pure-Python, no-ROS/no-PX4/no-GPU simulator that runs the REAL `refnav.core.Mission`
decision code against the real course geometry, so you can test a route or a Params change in under
a second without waiting on the cloud sim. It extends the exact pattern `tests/test_refnav_core.py`
already uses (kinematic fake PX4 + ray-cast depth camera), just pointed at the real maze instead of
a test room.

**Run it:**
```bash
cd local_sim
python3 sim.py ../route.json
```
Needs only `numpy` (already a repo dependency) plus the repo's own `refnav` package -- no ROS
install, no simulator session, no login. Runs in under a second for the whole 26-leg route.

**Read the output:** `result` is the mission's own final state (`"complete"`, `"landed: route
blocked: ..."`, etc. -- same strings `refnav.core.Mission` produces for real). `worst_clearance_m`
is an INDEPENDENT ground-truth check against the actual wall/boulder geometry (not what the
mission's sensor believed) -- negative means the route geometrically clips an obstacle regardless
of what the depth logic did or didn't catch.

**What it proves and doesn't:** validates flight-decision logic + route geometry against the real
course dimensions. Does NOT validate cuVSLAM/VIO, PX4 attitude control, real camera noise, or DDS
timing -- cuVSLAM is NVIDIA/CUDA-only and literally cannot run on a non-NVIDIA laptop. Treat a
"complete" result here as "the logic and geometry check out," not "this will fly on hardware."

**To test your own route/params** without touching the shared `route.json`: copy it, edit
`"route"` or `"params"`, and pass your copy's path as the argument instead. `course.py` has the
course geometry (walls/boulders/launch/landing) as plain constants if you need it directly.

**Solved (2026-11-06): `route.json` now completes all 26 legs in `local_sim`, no false stops, no
deadlocks.** Two things had to change from the mission's default `Params`, both already baked into
`route.json`'s `"params"` block:

1. Default `Params` (`corridor_half_m=0.6`, `stop_m=0.9`) don't fit this maze -- real corridors run
   ~1.07 m wide, so the default 0.6 m detection half-width clips the nearby wall immediately.
   Tuned to `corridor_half_m: 0.4, stop_m: 0.35`.
2. Tightening those alone caused a deadlock: near the end of short legs, the look-ahead distance
   (`stop_m + lead_max_m`) pokes past where the leg ends into open space with nothing in range to
   confirm, so `min_corridor_coverage` isn't met and the mission holds in place forever (there's no
   timeout on a corridor-confirmation hold, only on total depth loss). Fixed by shrinking
   `lead_max_m` to `0.2` and lowering `min_corridor_coverage` to `0.15`. Real-design note: a camera
   mounted with a slight downward tilt (rule 3.2 allows re-angling it) would likely get a steadier
   floor return and might not need `min_corridor_coverage` lowered this far -- worth revisiting
   once the real camera mount is decided, since a higher coverage requirement is more conservative.

Also re-planned the route itself at a 32cm clearance margin (was 30cm, 20 legs): the first working
route ground-truth-clipped a wall corner at only ~3cm past a 25cm placeholder drone radius, too
thin to trust. Now 26 legs, tightest point ~33cm raw clearance (leg 17, near the tunnel exit) --
**`DRONE_RADIUS_M=0.25` in `local_sim/sim.py` is still a placeholder**, replace with the real
airframe span once the kit's in hand and recheck leg 17 specifically. Full writeup in
`local_sim/README.md`.

## Flight visualizations

Two separate HTML replays -- don't confuse them, they show different things:

1. **`diagrams/sim-flight.html`** -- the `local_sim` replay. Colored by an INDEPENDENT
   ground-truth clearance check against the real course geometry (not what any sensor believed).
   Synthetic/kinematic, no camera/VIO involved. Regenerate after any route/params change:
   ```bash
   cd local_sim && python3 export_flight.py ../route.json ../diagrams/sim-flight.html
   ```
2. **`diagrams/real-flight.html`** -- a replay of an ACTUAL FlyX Lab run: real PX4 trajectory,
   real `refnav.core.Mission` phase/decisions, real depth-camera (and RGB, if available) frames.
   Two steps, because the recording has to happen in the cloud sim (no ROS/PX4 on this laptop):
   - In the FlyX Lab companion terminal, alongside a **team-approved** mission run (never run
     flight code just to record it -- see root `CLAUDE.md`):
     ```bash
     python3 -m refnav.node --ros-args -p route_file:=route.json -p use_sim_time:=true   # as usual
     python3 onboard/record_flight.py --ros-args -p use_sim_time:=true -p out_dir:=recordings/run1   # 2nd terminal
     ```
     `record_flight.py` is a pure observer: it publishes nothing and sends no commands, only
     subscribes (same topics `refnav.node` already proves work). **Unverified end to end** -- no
     ROS/PX4 available on this laptop to test it against a live run; only the offline half
     (`tools/build_real_flight_viz.py`) has been tested, against a synthetic manifest. Watch its
     log output on first use.
   - Download the `recordings/run1/` folder back to this repo, then on this laptop:
     ```bash
     python3 tools/build_real_flight_viz.py recordings/run1/manifest.json diagrams/real-flight.html
     ```

We explicitly are NOT building a local 3D physics/VIO/rendered-camera engine -- that's literally
what FlyX Lab already is (Gazebo + PX4 SITL + cuVSLAM), and cuVSLAM is CUDA-only, hard-blocked on
this Mac. `real-flight.html` gets the same visualization value by visualizing an actual cloud run
instead of re-simulating one.
