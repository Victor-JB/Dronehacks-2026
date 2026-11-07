"""Run run_sil.py over seeds/configs (one subprocess each) and tabulate. Writes docs/sil_results.md."""
import itertools, json, os, subprocess, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.abspath(os.path.join(HERE, "..", "..", "docs", os.environ.get("DH_OUT", "sil_results.md")))
N = int(os.environ.get("DH_SEEDS", "10"))
rows = []
lines = ["# Software-in-the-loop results (real mission/localizer node code, simulated drone)\n",
         "Odom frame rotated 0.7 rad and offset from the course frame; noise 1 cm; drift 0.01 m/sqrt(m); "
         "tube blackout velocity error std 0.08 m/s; PX4-like 1.0 m/s velocity cap with 0.3 s lag; "
         "synthetic 80-degree forward scans (blind in the tube) when the localizer is on. Contact = drone centre within 0.24 m of an obstacle.\n",
         "| route | localizer | tube speed (m/s) | seeds | clean runs | mean course time (s) | mean touches | worst clearance (m) | mean pad error (m) |",
         "|---|---|---|---|---|---|---|---|---|"]
for route, locz, vd in itertools.product(("window", "gate"), (True, False), tuple(float(x) for x in os.environ.get("DH_VDARK", "0.85,0.6").split(","))):
    res = []
    for s in range(N):
        cmd = [sys.executable, "-B", os.path.join(HERE, "run_sil.py"), "--route", route, "--seed", str(s), "--v_dark", str(vd)]
        if locz:
            cmd.append("--localizer")
        out = subprocess.run(cmd, capture_output=True, text=True).stdout
        for ln in out.splitlines():
            if ln.startswith("RESULT"):
                res.append(json.loads(ln[7:]))
    ok = [r for r in res if r["touches"] == 0 and r["course_time_s"] is not None]
    row = "| %s | %s | %.2f | %d | %d | %.1f | %.2f | %+.3f | %.3f |" % (
        route, "on" if locz else "off", vd, len(res), len(ok),
        np.mean([r["course_time_s"] for r in res if r["course_time_s"]]), np.mean([r["touches"] for r in res]),
        min(r["min_clearance_m"] for r in res), np.mean([r["pad_xy_error_m"] for r in res]))
    print(row, flush=True); lines.append(row)
    open(OUT, "w").write("\n".join(lines) + "\n")
