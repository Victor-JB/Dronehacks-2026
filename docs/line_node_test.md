# line_node end-to-end test (rendered mono images)

Real node code on ray-cast 1024x640 mono images: camera pitched 20 deg down, body 0.85 m above the floor, 2-inch guideline, corridor walls. Grey levels: floor 200, line 75 (model colours: light-grey floor, dark-red line), noise sigma 4. The mission only uses observations with conf >= 0.5.

| case | true lateral (m) | true heading (deg) | error lateral / heading / conf | verdict |
|---|---|---|---|---|
| centred, light walls | +0.00 | +0 | +0.000 m / +0.1 deg / conf 1.00 | OK |
| offset left 0.20 | +0.20 | +0 | +0.000 m / +0.1 deg / conf 1.00 | OK |
| offset right 0.25, yaw 8 deg | -0.25 | +8 | +0.000 m / +0.1 deg / conf 1.00 | OK |
| yaw -12 deg | +0.05 | -12 | +0.001 m / +0.1 deg / conf 1.00 | OK |
| weak contrast (line 160 on 200) | +0.10 | +0 | +0.000 m / +0.1 deg / conf 1.00 | OK |
| dark walls | +0.00 | +0 | +0.000 m / +0.1 deg / conf 1.00 | OK |
| dark walls, offset 0.20, yaw 8 | +0.20 | +8 | +0.002 m / +0.1 deg / conf 1.00 | OK |
| dark walls, narrow corridor 0.38 | +0.05 | +0 | +0.000 m / +0.1 deg / conf 1.00 | OK |
| line 0.12 m from a dark wall | +0.36 | +0 | +0.001 m / +0.1 deg / conf 1.00 | OK |
| weak contrast + dark walls | +0.10 | +4 | +0.000 m / +0.1 deg / conf 1.00 | OK |
| no line, light walls | - | - | none | correct (no output) |
| no line, dark walls | - | - | none | correct (no output) |
| no line, dark walls, yaw 10 | - | - | none | correct (no output) |
| no line, dark walls, narrow 0.38 | - | - | none | correct (no output) |

Same result at 0.60 m flight height (`--height 0.6`): all 14 cases OK, no false detections.

**Bug this test found (fixed).** The first version marked pixels "darker than a 31×31 box-blurred background". With no guideline in view and dark walls, it reported each wall's base edge as a line, at lateral −0.48 m with confidence 1.00. The mission would have accepted that within 0.6 m of a guideline segment and pulled the estimate about 14 cm toward a wall.
- **Fix 1:** the detector now uses a morphological black-hat (closing − image) with an elliptical kernel about 2.2× the line's image width. Only dark structures narrower than the kernel, with floor on both sides, respond. A wall base is a step edge and gives no response.
- **Fix 2:** `mission_node` rejects line fixes more than `line_max_innovation` = 0.20 m from the expected line. The guideline is in zone 1, before the tube, so real drift there is far smaller.

Figure: `figures/line_node_test.png`.
