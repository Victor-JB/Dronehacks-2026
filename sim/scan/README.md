# The course from the point scan

`dronehacks-course-scan-2026-11-06.ply` is a Scaniverse scan of the real course (596k points).
`fit_route_to_scan.py` turns it into the two things that depend on where the walls really are:

```sh
python3 sim/scan/fit_route_to_scan.py --route-out /tmp/route.json   # ~2 min, numpy + opencv
```

- **The sim's walls:** `sim/models/course_v3/meshes/scan_walls.obj` (5 cm voxels of the scan, merged
  into boxes). `model.sdf` uses it in place of the CAD's walls, tube and boulders; floor, nets, pad,
  launch mark and c2b (not in the scan) stay from the CAD.
- **The route:** the CAD waypoints moved to clear the real walls at 1.15 m. Copy the printed route
  into `modes.json` mode `course_scan` on `main`.

What the scan showed (2026-11-06): m2, m3 and a1 stand 0.3-0.8 m from their CAD positions, enough
for the CAD route to hit walls on legs 3-7; the window opening is 0.88-1.52 m high, not 0.30-1.22.
Left out as transient (`EXCLUDE`): a person-shaped column in the maze. Kept but flagged: a thin
panel by the launch mark and something small in the window opening. Check both on site; if the
window one is not real, add it to `EXCLUDE` and re-run.
