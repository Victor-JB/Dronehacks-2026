# Flight comparison

Four completed simulation runs are compared. Each timing uses simulation timestamps: PX4 arm → verified hover, verified hover → route complete, route complete → landing, and arm → landing. Phase sections are telemetry-derived; brief sample gaps are shown as unobserved in the JSON. These are individual runs, not a statistical reliability estimate. The evidence is simulation-only; independent geometry checks use offline simulated truth and modeled CAD envelopes.

| Run | Controller / route profile | Guard | Arm → hover | Hover → route complete | Route complete → land | Total armed flight | Window clearance | Pad clearance |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| course-08 | waypoint controller · `course-window-x500.json` | not recorded | 14.032 s | 63.832 s | 5.136 s | 83.000 s | 0.106 m | 0.314 m |
| course-09 | waypoint controller · `course-window-x500.json` | not recorded | 13.312 s | 63.860 s | 4.828 s | 82.000 s | 0.110 m | 0.317 m |
| course-10 | waypoint controller · `course-window-x500-tuned.json` | not enabled | 14.132 s | 62.636 s | 5.232 s | 82.000 s | 0.108 m | 0.313 m |
| policy-03 | learned policy · `course-window-x500.json` | enabled | 13.332 s | 58.160 s | 5.508 s | 77.000 s | 0.093 m | 0.311 m |

| Aggregated route phase (s) | course-08 | course-09 | course-10 | policy-03 |
|---|---:|---:|---:|---:|
| First corridor | 5.200 | 5.100 | 5.064 | 5.224 |
| Maze | 18.068 | 18.432 | 17.772 | 17.376 |
| Gate A | 4.400 | 4.436 | 4.288 | 3.960 |
| Tunnel + chicane | 15.764 | 15.796 | 16.072 | 12.296 |
| Window approach / align / cross | 15.636 | 15.400 | 14.840 | 14.808 |
| Landing approach | 4.664 | 4.660 | 4.568 | 4.428 |

Course-10’s tuned route took 62.636 s, 1.224 s less than course-09’s 63.860 s. Both armed flights lasted 82.000 s. Policy-03 completed its route in 58.160 s and its armed flight in 77.000 s. These comparisons describe these recorded runs only. Aggregated movement-phase totals exclude the short hover-check, unobserved, and other unlabeled intervals; individual labeled sections and gaps are in the JSON.

All four runs passed the same eight independent geometry checks, completed 15/15 waypoints, crossed the window with the full modeled envelope, and landed inside the modeled pad. The JSON includes the check results, window and pad clearances, controller and route-profile hashes, policy checkpoint and guard flag, section timings, and SHA-256 hashes for each input archive and timing tool.

For policy-03, the boundary-brake guard was enabled (364 policy actions; 193 braking interventions across 4,689 guard setpoints). Its predictive guard settings are documented in the policy results JSON; this comparison does not imply a formal safety guarantee.

Local source archives are under `artifacts/markerfree/` and are not included in the public repository. The compact JSON contains their hashes.
