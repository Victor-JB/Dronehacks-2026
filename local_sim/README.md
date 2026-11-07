# Local sim (no ROS, no PX4, no GPU)

## WARNING (2026-11-07): the hardcoded route.json does NOT survive realistic conditions
`local_sim` now has a PX4-like physics model + VIO-style estimator error (`dynamics.py`), a tilting
noisy depth camera with latency, and `montecarlo.py`. Results for `route.json`, 16 seeds each:

| scenario | complete | ground-truth collision |
|---|---|---|
| ideal (no noise, physics only) | 1/1 | 0 |
| launch placement 2 cm / 1.5 deg, perfect VIO | 6/16 | 10/16 |
| placement + VIO drift 0.25% / 0.5% / 1% | 2/16 each | 14/16 each |
| jig placement 0.5 cm / 0.3 deg, perfect VIO | 16/16 | 0 |
| jig placement + good VIO 0.25% | 9/16 | 7/16 |

Why: the route has ~7 cm of margin; heading placement error alone swings the far end ~10 cm, and
VIO drifts 4-10 cm over the flight. The depth stop only looks ahead along each leg, so side-swipes
are never caught. Also found: real PX4 with refnav's position-only setpoints and lead_max_m 0.2
flies ~0.19 m/s (MPC_XY_P 0.95 x 0.2 m), so the run takes ~155 s, not ~95 s.

Also WRONG in local_sim/course.py (fix in progress): C1/C2a/C2b are thin 10 cm walls angled ~20 deg,
not the solid 0.65 m boxes modelled here, and the tunnel tube (oval 1.47 x 1.83 m, x 1.575-3.048,
y -0.305-1.219) is missing. route.json was planned against this wrong geometry. See the other team's
corridor-centred window route: https://github.com/machmoon/dronehacks-sim (mission/course.json).

## Update (2026-11-07): map planning (`plan_goal`) fixes most of it; centering does nothing here
`refnav/plan.py` adds a clearance-preferring A* planner over a known map plus camera-confirmed
obstacles. With `plan` + `plan_goal` the route file is only a destination and the planner chooses the
path (wider margins than the hand-built route). 16 seeds each, completions without ground-truth collision:

| scenario | base | center only | goal plan | goal plan + center |
|---|---|---|---|---|
| placement only (perfect VIO) | 6/16 | 6/16 | 13/16 | 13/16 |
| VIO drift 0.25% | 2/16 | 2/16 | 8/16 | 9/16 |
| VIO drift 0.5% | 2/16 | 2/16 | 7/16 | 7/16 |
| VIO drift 1% | 2/16 | 2/16 | 4/16 | 4/16 |

Lateral centering never triggers on this course: collisions happen beside wall ends, outside the
camera's field of view, and it only centres between two *seen* walls. Replanning on new obstacles
never fired (static course). Remaining failures track VIO error. Still subject to the course.py
geometry errors above and the placeholder 25 cm drone radius. Run: `MC_VARIANT=goal python3 montecarlo.py ../route.json 16`
or `python3 sim.py ../route.json --variant goal` (variants: base, center, plan, both, goal, goal+center).

Run: `cd local_sim && python3 sim.py ../route.json --seed 3` or `python3 montecarlo.py ../route.json 16`
(`MC_SET=jig` for the careful-placement scenarios; `--model kinematic` for the old idealised model).

A pure-Python stand-in for the FlyX Lab sim, for when it's down or for a fast iteration loop.
Runs the real `refnav.core.Mission` decision code, unmodified, against:

- a kinematic fake PX4 (same model `tests/test_refnav_core.py` already uses)
- a ray-cast depth camera seeing the **real course geometry** from `dronehacks-course-v3.obj`
  (`course.py`)
- an independent ground-truth clearance check against that same geometry (not what the mission
  *believed*, what's *actually* true), to catch a route that would clip an obstacle even if the
  mission's own sensor logic didn't flag it

```
python3 sim.py ../route.json
```

## What this can validate
Whether a given `route.json` + `Params` fly a geometrically sound path through the actual
course layout, and whether the depth-based obstacle stop would catch a bad leg, in well under a
second, with zero cloud/hardware dependency.

## What it cannot validate
cuVSLAM/VIO accuracy, PX4 attitude/rate control, real camera noise/calibration, DDS/network
timing, or anything else that needs the real estimator or hardware. **cuVSLAM specifically is
NVIDIA/CUDA-only and cannot run on this machine at all** — there's no way to locally reproduce
that part regardless of Docker/emulation. Treat this as a decision-logic and geometry check, not
a flight-readiness check.

## Bug fixed 2026-11-07: depth camera was rendered from the wrong place
Until this date `sim.run` passed the drone's position *relative to home* to the depth ray-caster,
but the obstacle segments are in *absolute* course coordinates -- so every depth frame was
rendered as if the drone had launched from the center of the course instead of the launch mark.
The ground-truth clearance check was always correct; the mission's depth-based safety logic was
being tested against misplaced walls. Also fixed: depth-model walls started 18 cm above the floor.
After both fixes `route.json` still completes 26/26 with zero obstacle stops/holds, and both
param fixes below are still required (re-verified), but the explanation of fix 1 changed.

## 3D flight video
```
python3 render3d.py ../route.json ../diagrams/course-run-3d.mp4
```
Ray-traced (pure numpy, no GPU) chase-cam video of the run: modeled quad, the real course, plus
FPV camera, the actual depth frames the mission was handed, depth point cloud / pseudo-LiDAR,
minimap, telemetry and the mission's own event log. ~10 min on 12 cores; `ONLY_FRAME=400
python3 render3d.py ../route.json still.png` renders one still. Roll/pitch are derived from the
trajectory's acceleration; the flight model is still the kinematic one above.

## Status: route.json completes all 26 legs (2026-11-06)
`python3 sim.py ../route.json` now returns `result: complete`, 26/26 legs, no false stops, no
deadlocks. Getting here took two separate fixes, both baked into `route.json`'s `params`:

1. **Default `Params` false-stop in the maze.** With the defaults (`corridor_half_m` 0.6,
   `stop_m` 0.9) the mission lands on leg 3 ("obstacle 0.82 m ahead"): the 0.6 m detection
   half-width and 0.9 m stop distance are wider/longer than this maze's ~1 m corridors and short
   legs allow, so walls beside or just past the end of a leg read as blocking it. Settled on
   `corridor_half_m: 0.4, stop_m: 0.35`. (An earlier version of this note blamed leg 1; that was
   an artifact of the depth-placement bug below.)

2. **Tightening those alone caused a deadlock.** With a smaller `corridor_half_m`, the mission
   would hold position indefinitely partway through a leg, unable to confirm the corridor ahead.
   Root cause: `corridor_coverage`'s lateral+height window only admits pixels whose ray, at the
   `stop_m + lead_max_m` look-ahead distance, both lands within the narrow corridor AND stays
   within `band_m` of flight height -- for a level, non-tilted camera, that's a narrow cone of
   near-level rays, and in a genuinely open stretch (the gaps between maze dividers, the open
   areas past the chicane) nothing terminates them within range, so coverage reads low even
   though nothing is actually in the way. Fixed by shrinking the look-ahead distance itself
   (`lead_max_m: 0.2`, so `stop_m + lead_max_m = 0.55` instead of the default 1.9) and lowering
   `min_corridor_coverage` to `0.15`. **Real-world note:** a camera physically angled down a few
   degrees (allowed by rule 3.2) would get a steadier floor return and might not need
   `min_corridor_coverage` lowered this far -- worth revisiting once the real camera mount is
   decided, since a higher coverage requirement is a more conservative safety margin.

3. **Route geometry itself was re-planned for more clearance.** The first working route (20 legs,
   30cm planning margin) completed the mission logic but ground-truth-clipped a `C1 chicane wall`
   corner at only ~3cm of margin past a 25cm placeholder drone radius -- too thin to trust.
   Re-planned at 32cm margin with finer path simplification (26 legs): tightest point is now
   ~33cm raw clearance (7.7cm past the same 25cm placeholder), at leg 17 near the tunnel exit.
   **`DRONE_RADIUS_M = 0.25` in `sim.py` is still a placeholder** -- replace it with the real
   airframe's motor-tip-to-motor-tip span once the kit is in hand and re-run; if the real radius
   is bigger than 25cm, leg 17 needs another look.
