# Technical presentation outline (5–7 min + Q&A)

The judges score this part on autonomy design, technical execution, originality, and evidence of testing and iteration (25 pts). They also weigh generality (15 pts) and how well auxiliary sensors are used. Aim for one idea per slide and show data, not adjectives. Regenerate every figure from the flight logs and the tools on Sunday morning.

| # | Slide | Content | Evidence to show |
|---|---|---|---|
| 1 | Problem as we framed it | A known-layout, GPS-denied course: the hard parts are drift (above all in the dark tube), 0.91–0.95 m apertures, and autonomous land → disarm → return. Scoring math: finish > return (×1.25) > window (+5) > raw speed | The table in `STRATEGY.md` §1 |
| 2 | Architecture | course file → planner → follower → PX4, plus scan-to-map re-localization and the floor-line cue; one diagram | README diagram |
| 3 | Planning from geometry (generality) | Slice the official mesh at cruise height → clearance A* → shortcut → fillets → time-optimal speed profile; the window found as a hole in a wall mesh; the tube flown along its axis | `docs/figures/course_plan.png` |
| 4 | Generality test | The same code on 12 perturbed courses, a mirrored course, and a bricked-up window (switches to Gate B by itself) | `docs/figures/generality.png`, `docs/generality_results.md` |
| 5 | Staying localized | cuVSLAM coasts only ~0.5 s in the dark. Fixes: DLL-style signed-distance scan matching to the course model; observable-subspace updates; a blackout gate with two-match confirmation; full speed through the tube; slow after exit until re-localized | `docs/strategy_results_aiding.md`, `docs/sil_results.md` |
| 6 | Control details that mattered | Velocity feed-forward taken 0.3 s ahead cancels PX4's velocity-loop lag (4 touches → 0); reference-clock governor; descent ramp plus immediate disarm (−2.5 s) | SIL before/after numbers |
| 7 | Testing and iteration | Unit tests; Monte Carlo (>3,000 simulated runs); software-in-the-loop of the *real ROS nodes* against a simulated drone and a fake PX4; real-flight logs from the weekend | Test list; SIL tables; your bag plots |
| 8 | Algorithms we compared | Fillet + TOPP vs motion-primitive search (Liu et al. 2017, re-implemented): 19.8 s vs 24.3 s on this course, so we fly the former. ORB-SLAM3 and VGGT were evaluated and rejected for the Orin Nano | `docs/planner_comparison.md`, `docs/research_notes.md` |
| 9 | Results on the real course | Best run: time, route, return, touches; what failed and what we changed | Run videos and logs |

Q&A preparation:
- *"Did you hard-code the course?"* No. The only course-specific input is the official file plus the launch heading. Show the generality figure and `docs/waypoints_v3.md` ("generated, never typed").
- *"What if the layout changes?"* Re-plan from the updated file in under 2 s. Moved walls within ±15 cm are handled even without an update.
- *"Rule 6.4?"* The localizer uses only the published model. The SLAM node is restarted before every run. Logs are used only for analysis.
