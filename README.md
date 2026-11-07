# Dronehacks-2026 — `victor/sim`

| Path | What it is |
|---|---|
| `refnav/core.py`, `refnav/depth.py` | Mission logic and depth processing, the same files as `main` and `jetson` |
| `tests/` | `python3 -m pytest -q tests` |
| `route.json` | Mission route |
| `controllers/` | Algorithms to fly in the sim, one folder each |
| `fly-sim.sh` | One command to run a controller in the sim |
| `sim/` | Gazebo + PX4 SITL course sim, from [machmoon/dronehacks-sim](https://github.com/machmoon/dronehacks-sim) (`codex/markerfree-flight`) |
| `latex/` | Write-ups of navigation approaches |
| `shared-context/` | Team context files |

## Sim

```sh
./fly-sim.sh controllers/waypoints     # start the sim, fly that controller, stream its logs
./fly-sim.sh controllers/hover --headless
```

Ctrl-C stops everything. Write a controller as a folder with `controller.py`: see
`controllers/README.md`. First time only: `cd sim && ./build.sh` to build the images.
`sim/stop.sh --vm` also stops the Colima VM.
