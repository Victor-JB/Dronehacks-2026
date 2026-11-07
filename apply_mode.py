import json, sys

with open("modes.json") as f:
    cfg = json.load(f)

mode = str(cfg["active_mode"])
chosen = cfg["modes"].get(mode)
if chosen is None:
    sys.exit(f"No such mode: {mode}")

params = {**cfg["global_params"], **chosen.get("params_override", {})}
route = [[leg["forward_m"], leg["left_m"]] for leg in chosen.get("route", [])]

with open("route.json", "w") as f:
    json.dump({"route": route, "params": params}, f, indent=2)

print(f"route.json <- mode {mode} ({chosen['name']}), {len(route)} leg(s)")
