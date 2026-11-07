# Dronehacks-2026 — `victor/sim`

| Path | What it is |
|---|---|
| `refnav/core.py`, `refnav/depth.py` | Mission logic and depth processing, the same files as `main` and `jetson` |
| `tests/` | `python3 -m pytest -q tests` |
| `route.json` | Mission route |
| `sim/` | Gazebo + PX4 SITL course sim, from [machmoon/dronehacks-sim](https://github.com/machmoon/dronehacks-sim) (`codex/markerfree-flight`) |
| `latex/` | Write-ups of navigation approaches |
| `shared-context/` | Team context files |

## Sim

```sh
cd sim
./build.sh                      # once: builds the Gazebo and PX4 images
./runtime/run-markerfree.sh     # start (does not arm or fly)
./stop.sh                       # stop; --vm also stops the Colima VM
```

Gazebo view: <http://127.0.0.1:8082/vnc.html?autoconnect=1&resize=scale>. See `sim/README.md` for flights.
