# Local sim (no ROS, no PX4, no GPU)

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
