# Stage 5 architecture: exploring an unknown course (interfaces only; Milestone 3+)

The course has changed and there is no reliable CAD. Stage 5 must start from an empty map and never read a
course file, an obstacle layout or a recorded route. `stage5_interfaces.check_no_prior_map` enforces this
for declared inputs. Contracts: `ros2_ws/src/dh_core/dh_core/stage5_interfaces.py`. Input:
`ShadowMonitor.latest_observation()` → `NavObservation` (validated odometry, accepted depth, intrinsics,
extrinsic, localization mode, Stage 4 report).

## Pipeline
```
NavObservation ─► OccupancyMap.integrate ─► FrontierSelector / GoalDetector ─► goal
                                   │                                             │
                                   └──► A* (inflated, unknown = blocked) ─► clothoid smoothing ─► time profile
                                                                                 │
                         TrajectorySafety.check (3-D, whole trajectory) ◄────────┘
                                   │ ok
                         Stage 4 SafetyGate clamp ─► Intent ─► (main's skybeta_node publishes; never this code)
```

## Reusing main's `dronehacks_nav/mapping.py` (VoxelMap)
**Reuse as is:** `camera_rays` (depth → unit rays); the vectorised `integrate` (free carving up to the
return, occupied at the return, log-odds clamp, "no return = no evidence"); the UNKNOWN/FREE/OCCUPIED codes;
`slice` for 2-D planning bands.

**Missing for Stage 5:**
- **Rolling window.** The 20×20×4 m box is fixed and centred on the VIO origin, so the course must fit
  around the start point. Needed: `recentre()` that shifts the array by whole voxels, keeping world
  coordinates, with an option to page the evicted region out to a coarse persistent layer.
- **Per-voxel timestamps.** `last_seen` (any evidence) and `last_occupied`. These support dynamic obstacles
  (decay occupied evidence not re-confirmed while in view), staleness-aware planning, and "remembered
  after leaving the FOV".
- **Visibility memory.** Keep "seen free at time t" separate from "never seen", so frontiers are exact.
- **Inflation.** A distance transform over OCCUPIED ∪ UNKNOWN, inflated by vehicle radius + uncertainty
  margin + k·pos_std, as `route.py`/`planner_astar.ClearancePlanner` do in 2-D.
- **Exploration memory.** Visited cells, to avoid unnecessary backtracking and to score frontiers.
- **Pose at capture.** Integrate each frame at the odometry pose nearest its stamp (the shadow monitor
  already does this), and refuse frames during localization HOVER/ABORT. A VIO jump must not smear the map.

## Planning reuse
`route.py`'s A* + clothoid (`planner_astar.ClearancePlanner`, `timing.fillet_path`/`time_parameterize`) takes a
`Grid2D`. Feed it a `VoxelMap.slice` of the flight band with UNKNOWN treated as occupied, and replan on map
change or when `TrajectorySafety` rejects the current trajectory. The speed profile's `v_limit_fn` takes the
Stage 4 `max_speed_for_distance` of the inflated clearance, so planned speed respects braking distance.

## Integration constraints (organizer interface, for later)
ENU vision frame; existing fused odometry; position or world-frame velocity setpoints at about 10 Hz (main
currently sends pose at 20 Hz); twist capped at 1.5 m/s and altitude kept inside 0.3–3.0 m by
`flight_interface` (seen in `/skybeta/state.envelope`); existing Offboard handoff and pilot takeover;
exactly one `flight_interface` and one approved publisher (main's `skybeta_node`).

## Policy workflow compatibility (reviewed, not executed)
`promote.sh` (on `sim`) requires a `policy/<name>` branch that contains `main`, passes `python3 -m pytest -q tests`
on that commit, and has a "complete" sim run of that exact commit. It never pushes and never touches the drone.
`main` has no `ros2_ws/dh_core`, so porting means moving `safety_filter.py`, `health.py`'s
`LocalizationHealth` and `shadow/` (plus later Stage 5 modules) into main's `dronehacks_nav/` or `refnav/`
layout, with their tests in `tests/`. All of them are pure Python and import only numpy and `refnav.depth`, so
the port is a move plus import fixes, with no rewrite.
