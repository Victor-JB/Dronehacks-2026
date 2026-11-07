import json, sys

if len(sys.argv) != 3:
    sys.exit("usage: set_speed.py <mode_id> <speed_mps>")

mode_id, speed = sys.argv[1], float(sys.argv[2])

with open("modes.json") as f:
    cfg = json.load(f)

mode = cfg["modes"].get(mode_id)
if mode is None:
    sys.exit(f"No such mode: {mode_id}")

mode.setdefault("params_override", {})["speed_mps"] = speed

with open("modes.json", "w") as f:
    json.dump(cfg, f, indent=2)

print(f"mode {mode_id} ({mode['name']}) speed_mps -> {speed}")
