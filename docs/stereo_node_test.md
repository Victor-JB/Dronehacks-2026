# stereo_scan_node end-to-end test (rendered stereo pairs)

Real node code with OpenCV SGBM on ray-cast 1024x640 raw mono pairs (textured wall + floor), camera pitched 20 deg down, 7.5 cm baseline. 'rotated' = right camera yawed 1.0 deg and rolled 0.5 deg against the left. Body-x of the scan returns should equal the wall distance.

| rectification source | pair | wall (m) | beams with returns | median x (m) | error | p10-p90 x (m) | forward clearance (m) |
|---|---|---|---|---|---|---|---|
| camera_info | rotated | 0.80 | 67/81 | 0.793 | -0.9% | 0.792-0.793 | 0.793 |
| tf | rotated | 0.80 | 67/81 | 0.793 | -0.9% | 0.792-0.793 | 0.793 |
| baseline | aligned | 0.80 | 66/81 | 0.793 | -0.9% | 0.792-0.793 | 0.793 |
| baseline | rotated | 0.80 | 66/81 | 0.676 | -15.5% | 0.643-0.687 | 0.686 |
| camera_info | rotated | 1.50 | 69/81 | 1.476 | -1.6% | 1.474-1.480 | 1.475 |
| tf | rotated | 1.50 | 69/81 | 1.476 | -1.6% | 1.474-1.480 | 1.475 |
| baseline | aligned | 1.50 | 68/81 | 1.472 | -1.9% | 1.469-1.477 | 1.469 |
| baseline | rotated | 1.50 | 68/81 | 1.079 | -28.1% | 0.997-1.113 | 1.112 |
| camera_info | rotated | 2.50 | 69/81 | 2.420 | -3.2% | 2.408-2.430 | 2.413 |
| tf | rotated | 2.50 | 69/81 | 2.420 | -3.2% | 2.408-2.430 | 2.413 |
| baseline | aligned | 2.50 | 68/81 | 2.503 | +0.1% | 2.426-2.521 | 2.463 |
| baseline | rotated | 2.50 | 68/81 | 1.494 | -40.2% | 1.344-1.548 | 1.547 |
