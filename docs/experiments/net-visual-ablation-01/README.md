# Net visual ablation protocol

This registered experiment removes the five rendered net surfaces while retaining their collision geometry. It first compares stationary RGBD captures from two generated worlds, then permits a control hover and route followed by a treatment hover and route only if each preceding check passes. The [protocol](protocol.json) defines the checks before any capture or flight in these worlds.

Generate the paired base worlds with:

```sh
python3 -m tools.make_net_visual_ablation \
  --output-dir artifacts/markerfree/net-visual-ablation-NEW
```

Select a generated `control.sdf` or `treatment.sdf` through `SIM_WORLD` when starting the dedicated marker-free simulator. The generator checks that the only control-to-treatment difference is removal of the five named visual elements. It retains every collision and the other world fields. Neither generated world adds a navigation input or a truth-feedback path.

Gazebo uses visual elements for rendering, and RGBD sensors use that rendering scene. Removing a net visual can therefore replace its depth return with background or far-limit depth while its collision remains. The passive comparison must demonstrate a material depth change before a flight ablation claim is made. [Gazebo visual documentation](https://gazebosim.org/api/sim/8/adding_visuals.html), [Gazebo sensor-system source](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/sensors/Sensors.cc).

This is a missing-net-surface-return stress case. It is not a realistic woven-net model, and it changes both RGB and depth. The estimator still uses the original static map, including its net panels, so the sensor/map mismatch is intentional. A successful simulator run would not establish real-camera net detection or physical transfer.

## Passive comparison

The recorded stationary pair passed every preregistered passive gate: 32 matched RGBD captures per condition, maximum capture interval 52 ms, continuous truth alignment, and identical stationary truth pose. The registered depth rule marked 47,657 of 98,304 image pixels as changed (48.4792%); the gate was 1%. This is evidence that the rendered depth input changed for this pair. It does not establish navigation or flight performance. Flight checks remain conditional and are reported separately from this passive result.

The machine-readable report is [results.json](results.json), and [capture-comparison.png](capture-comparison.png) shows the recorded views and 32-frame median depth. The raw capture archives and observer logs are retained locally and are not included in a fresh clone. With those archives present and NumPy installed, reproduce the numerical report from the repository root:

```sh
python3 docs/experiments/net-visual-ablation-01/analyze.py \
  --artifacts artifacts/markerfree/net-visual-ablation-01 \
  --output /tmp/net-visual-ablation-results.json
```

The plot script uses NumPy and Matplotlib; run it in a Python environment with both available:

```sh
python3 docs/experiments/net-visual-ablation-01/plot.py \
  --artifacts artifacts/markerfree/net-visual-ablation-01 \
  --output /tmp/net-visual-ablation-comparison.png
```
