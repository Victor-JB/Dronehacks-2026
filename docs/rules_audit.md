# Rules audit: every strategy and component against the official rulebook (2026-11-06)

> Historical audit; current source review is in `LOCAL_VALIDATION.md`. The illuminated tube no longer has a blind-motion exemption. VO status is now required by default. Fleet-ready 7efe7e4 requires a recent startup setpoint: only zero readiness twists precede Offboard; motion waits for fresh authority. No physical flight or hardware command is authorized by this document.


Rule numbers refer to the official rulebook as summarised in `../stanford_drone_hacks_research_handoff.md` §3–4.

Legend:
- ✅ complies;
- ⚠️ allowed only with organizer confirmation, or needs care on-site;
- ❌ conflicts as things stand.

## Verdict in one paragraph

All flight strategies are rule-compliant as designed:
- the three planners (fillet, SFC, MPL);
- the window and Gate B routes;
- full speed through the tube;
- map-based localization;
- line and pad cues.

Two backend features need written organizer permission: the `px4` backend, which arms, lands, disarms and re-arms, and is therefore needed for the return bonus; and any PX4 speed-limit change. **The real problem is R9.1, "your autonomy stack is built during the event".** This whole repository was written before the event, which may not be allowed. Ask at the briefing, before flying any of it.

## The drone's flight stack: organizers' Sky Beta interface contract (`docs/competitor_guide.md` in github.com/Artash-N/skybeta, `fleet-ready`)

This is the set of rules the drone's software must obey on the issued drone. **Use the default `backend: skybeta`.**

| Flight-stack rule | Our code | Status |
|---|---|---|
| Command the drone only through `/skybeta/setpoint/pose` or `/skybeta/setpoint/twist` | The `skybeta` backend publishes `/skybeta/setpoint/twist` (ENU velocity + yaw rate) at 30 Hz | ✅ |
| **"Do not publish to `/fmu/in/*` yourself"** (two OffboardControlMode publishers would fight) | The `skybeta` backend never touches `/fmu/in/*`. The `px4` backend **does**, so it is for SITL only, or with explicit organizer permission and their flight_interface stopped | ✅ `skybeta` / ⚠️ `px4` |
| Twist setpoints expire after 1.0 s | We publish at 30 Hz. If odometry goes stale the mission calls `/skybeta/hold` | ✅ |
| Safety envelope: altitude 0.3–3.0 m above the first odometry sample; twist norm ≤ 1.5 m/s; teams may tighten but not loosen it without asking | Cruise 0.85 m, return 2.05 m, final hover 0.35 m (above the 0.3 m floor); speed cap 1.0 m/s. We never change the envelope | ✅ |
| The interface **never arms and never changes mode**; the pilot arms and switches to Offboard; there is no arm service | The `skybeta` backend has no arm or mode code. `/skybeta/request_offboard` is called only by the `px4` path or in SITL, where the interface allows it (`allow_mode_request:=true`) | ✅ |
| **Team nodes must wait for `offboard_active` before commanding** | **Fixed today.** Before, `/dh/start` started the trajectory clock even if Offboard wasn't active yet. Now the mission holds with the clock frozen until `/skybeta/state.offboard_active` is true. `autostart_on_offboard:=true` makes the Offboard handover itself the start command | ✅ (fixed) |
| Landing is impossible through the interface (0.3 m altitude floor), but CP5 needs land + disarm, and the return needs re-arm | The `skybeta` backend ends hovering 0.35 m over the pad. It **cannot score CP5 or the return** until the organizers provide a land/arm mechanism | ⚠️ ask (organizer question 1) |
| `odom` is "ENU" but axes = the vehicle's axes when the SLAM node started; the origin is the startup pose | `MapAlignment` builds course ← odom from the launch mark + `launch_yaw_map` at start | ✅ |
| EKF2 local frame ≠ `odom` (constant yaw offset) | Only matters for the `px4` backend: `px4_yaw_offset` parameter | ✅ |
| Don't put `px4_msgs` in the team workspace | Our packages don't vendor it. README says to use the image's copy | ✅ |
| Team code in `~/team_ws/src`; started via `COMPETITOR_PKG` / `COMPETITOR_LAUNCH` in `/etc/skybeta/drone.env` | Packages `dh_core`, `dh_autonomy`; launch file `autonomy.launch.py` | ✅ |
| Pinned Python stack: **numpy < 2**, OpenCV from the Isaac image; extra pip packages only through a team Docker image | Tested today under **numpy 1.26.4 + OpenCV 4.10**: 11/11 unit tests, SIL identical, line test 14/14. No pip packages needed (scipy optional) | ✅ |
| Camera: raw `/oak/{left,right}/image_raw` + camera_info only, no depth; 1024×640 @ 30 Hz | `stereo_scan` computes depth on the Jetson from the raw pair. Its TF fallback covers missing stereo calibration | ✅ (not tested on hardware) |
| `/visual_slam/status` `vo_state` = 2 means tracking lost | Not used. We detect blackouts from odometry timing and dark zones in the map. Could be added as an extra trigger | ➖ |
| Record a bag for every flight (`host/run.sh bag`) | `tools/analyze_bag.py` reads those bags | ✅ |

## Strategy by strategy

| Strategy / component | Status | Rule | Notes |
|---|---|---|---|
| Plan the route at start-up from the published course model (all planners) | ✅ | R6.4, "Using the layout" | Preloading a map built from the published layout is explicitly allowed. The route is computed, not typed in, which is good for generality. If organizers post a changed layout, replace the OBJ file. |
| Fillet planner (default), SFC planner, MPL planner | ✅ | — | Same inputs and outputs. They differ only in how they shape the trajectory. |
| Window route (+5 pts) | ✅ | CP4 | Cruise height 0.85 m is inside the 0.305–1.219 m opening. |
| Gate B route | ✅ | CP4 | |
| Stay below the 1.727 m walls on the course | ✅ | R7.7 | Cruise 0.85 m. The planner refuses a cruise band that reaches the wall tops. The mission's safety check enforces a ceiling. |
| Return leg flown at 2.05 m over the barrier | ✅ | R7.7, R7.10 | Flying over walls is allowed only during the return. The state machine starts the return only after disarming on the pad. |
| Full speed through the dark tube | ✅ | — | No speed rule. Speed is limited only by PX4 and the interface. |
| One start command (`/dh/start`), then hands off | ✅ | R5.1–5.3, R7.11 | The mission needs no other human input. Abort stays on the RC kill switch, which our code never touches. |
| Team arms on the launch mark | ✅ (`skybeta`) / ⚠️ (`px4`) | R7.4 | The `skybeta` backend never arms. The `px4` backend sends an arm command at takeoff, which does nothing if the team already armed. |
| `px4` backend: offboard mode, land, disarm, re-arm from our node | ⚠️ | R5, R7.10; organizers' guide | Software mode changes are not human input. But the organizers' guide says not to publish to `/fmu/in/*`, so get explicit permission (organizer question 1). The **return bonus needs this** (or an organizer-provided equivalent). |
| PX4 failsafes, geofence, battery limits, kill switch | ✅ | R8.5 | Our code changes no PX4 parameters. Our software geofence is an extra layer on top of theirs. |
| Raising the speed cap (`MPC_XY_VEL_MAX` 1.0 → 1.5/2.0 m/s) | ⚠️ | R8.5, Code of Conduct ("flight limits") | Not on the R8.5 list, but the Code of Conduct forbids disabling flight limits. Ask in writing first (organizer question 2). See `planner_suite.md` Test S for what it buys. |
| Everything runs on the Jetson | ✅ | R5.4–5.6 | All nodes run onboard. A laptop may only watch (read-only). |
| Scan-to-map localizer (matches against the published model) | ✅ | R6.4 | Uses only the published layout and stores nothing from runs. |
| cuVSLAM in SLAM mode (organizers' stack) | ⚠️ | R6.4 | Its map can persist between runs. **Restart the visual-SLAM node before every run** so no run data feeds a later run (organizer question 4). |
| Floor guideline and landing-pad cues | ✅ | R6.2 | They are organizer-provided course features, not team-placed markers (inference; R6.2 allows features "provided by the organizers as part of the course"). |
| Recommended LiDAR (YDLIDAR T-mini Plus) | ⚠️ | R3.4, R4.1–4.8 | Seen at $79–95 and 45 g. It must be **< $100 at event time** and **< 50 g as weighed by an organizer** (tight margin), Class 1 laser (OK), powered from the drone battery (OK), and declared at inspection. Nothing else that draws power may be added. |
| Flight logs (`analyze_bag.py`) | ✅ | R6.4 | For analysis and tuning only. Never loaded by a later run. |
| Open-source ideas credited | ✅ | R9.1 | README "Credits" lists every paper and repository used. |
| AI-written code | ✅ / ⚠️ | R9.2 | AI tools are allowed, but **the team must be able to explain how it works**. Read `STRATEGY.md` and `paper_notes.md` before judging. |
| **Code written before the event** | ❌ / ⚠️ | **R9.1** | "Your autonomy stack is built during the event." Ask the organizers at the briefing (organizer question 8). If it is not allowed, treat this repository as research notes and rebuild during the event, which AI tools make fast. |

## Things the AI hard-coding review (R9.5) may flag, and the answer to give

| Item | Why it might be flagged | Answer, or fix before submission |
|---|---|---|
| `docs/waypoints_v3.md` | A list of course coordinates | Generated for reference and never read by the code. **Leave it out of the competition repository** to avoid the question. |
| `dark_zone_hints = ("tube", "tunnel")` in `route.py` | Finds dark zones by OBJ group name | The only course-specific semantic. On a new course, set the hint or mark dark zones in a config file. Everything else is derived from geometry. |
| `launch_yaw_map: 1.5708` | Assumes the drone faces north on the launch mark | A placement convention, not a route. It is a parameter. |
| `route: window` default | Chooses the window | The planner finds apertures geometrically. `route: auto` picks between routes by score. Generality tests: 15/15 modified courses planned, including one with the window bricked up. |
| `sim/make_gazebo_world.py` spawn pose | Contains the launch coordinates | Simulation tooling only. It never runs on the drone. |
