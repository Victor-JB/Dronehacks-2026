# Flight comparison

Six completed simulation runs are compared. Timings use simulation timestamps: PX4 arm → verified hover, verified hover → route complete, route complete → landing, and arm → landing. Phase sections are telemetry-derived; brief sample gaps are shown as unobserved in the JSON. These are individual runs, not a statistical reliability estimate. The evidence is simulation-only; independent geometry checks use offline simulated truth and modeled CAD envelopes.

| Run | Controller / route profile | Policy input configuration | Guard | Arm → hover | Hover → route complete | Route complete → land | Total armed flight | Window clearance | Pad clearance |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|
| course-08 | waypoint · `course-window-x500.json` | — | not recorded | 14.032 s | 63.832 s | 5.136 s | 83.000 s | 0.106 m | 0.314 m |
| course-09 | waypoint · `course-window-x500.json` | — | not recorded | 13.312 s | 63.860 s | 4.828 s | 82.000 s | 0.110 m | 0.317 m |
| course-10 | waypoint · `course-window-x500-tuned.json` | — | not enabled | 14.132 s | 62.636 s | 5.232 s | 82.000 s | 0.108 m | 0.313 m |
| policy-03 | learned · `course-window-x500.json` | raw depth · default `central2`; noise diagnostic unrecorded | enabled | 13.332 s | 58.160 s | 5.508 s | 77.000 s | 0.093 m | 0.311 m |
| policy-08 | learned · `course-window-x500.json` | raw depth · FALS7 · control depth, 0 mm configured Gaussian noise | enabled | 14.128 s | 58.200 s | 5.672 s | 78.000 s | 0.092 m | 0.311 m |
| policy-09 | learned · `course-window-x500.json` | raw depth · FALS7 · 10 mm Gaussian depth noise | enabled | 14.104 s | 58.200 s | 5.696 s | 78.000 s | 0.095 m | 0.312 m |

The policy-08/09 pair uses the same checkpoint, FALS7 normal configuration, route, 1.0 speed scale, 1.8 m heading lookahead, boundary brake, raw policy depth, and 120 ms maximum capture-gap setting. The configured depth noise differs: policy-08 is the control condition and policy-09 uses a 10 mm Gaussian standard deviation. Both armed flights lasted 78.000 s and both routes lasted 58.200 s. This matched pair is descriptive; it does not establish a speed improvement or reliability estimate.

| Aggregated route phase (s) | course-08 | course-09 | course-10 | policy-03 | policy-08 | policy-09 |
|---|---:|---:|---:|---:|---:|---:|
| First corridor | 5.200 | 5.100 | 5.064 | 5.224 | 5.132 | 5.148 |
| Maze | 18.068 | 18.432 | 17.772 | 17.376 | 17.100 | 16.988 |
| Gate A | 4.400 | 4.436 | 4.288 | 3.960 | 3.888 | 4.012 |
| Tunnel + chicane | 15.764 | 15.796 | 16.072 | 12.296 | 12.864 | 12.868 |
| Window approach / align / cross | 15.636 | 15.400 | 14.840 | 14.808 | 14.680 | 14.600 |
| Landing approach | 4.664 | 4.660 | 4.568 | 4.428 | 4.420 | 4.520 |

Course-10’s tuned route took 62.636 s, 1.224 s less than course-09’s 63.860 s; both armed flights lasted 82.000 s. Policy-03 completed its route in 58.160 s and armed flight in 77.000 s. The newer matched policy runs each took 58.200 s for the route and 78.000 s armed. These timings describe individual recorded runs only. Aggregated movement-phase totals exclude the short hover-check, unobserved, and other unlabeled intervals; per-waypoint sections and gaps are in the JSON.

All six runs passed the same eight independent geometry checks, completed 15/15 waypoints, crossed the window with the full modeled envelope, and landed inside the modeled pad. The JSON records run IDs, check results, clearances, configuration, per-section timings, archive hashes, route profile and timing tool hashes. The added policy-08/09 entries include explicit score-to-telemetry ID checks. For policy-08/09 it also records the runtime source hashes captured in telemetry.

For policy-03, the boundary-brake guard was enabled (364 policy actions; 193 braking interventions across 4,689 guard setpoints). Its predictive guard settings are documented in the policy results JSON; this comparison does not imply a formal safety guarantee. Policy-08 recorded 364 policy actions and 200 braking interventions; policy-09 recorded 363 policy actions and 237 braking interventions. These are run counters, not safety or reliability measures.

Local source archives are under `artifacts/markerfree/` and are not included in the public repository. The compact JSON contains their hashes.
