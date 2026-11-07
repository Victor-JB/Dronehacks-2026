"""Export an animated, colored-by-clearance visualization of a local_sim flight.

Runs the real refnav.core.Mission decision code (via sim.run) against a route.json,
then writes a self-contained HTML page: the same scaled floor plan as
diagrams/route-plan.html, with the ACTUAL flown trajectory drawn on top, colored by
ground-truth clearance at every sampled instant, plus a scrub/playback control.

    cd local_sim && python3 export_flight.py ../route.json ../diagrams/sim-flight.html
"""
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sim as S  # noqa: E402
from course import HALF  # noqa: E402

SCALE = 90.0  # px per metre -- matches diagrams/route-plan.html (1 px = 1.11 cm)


def to_svg(x, y):
    return ((x + HALF) * SCALE, (HALF - y) * SCALE)


def bucket(clr):
    if clr is None or not math.isfinite(clr):
        return "unknown"
    if clr < 0.0:
        return "critical"
    if clr < 0.15:
        return "warning"
    return "good"


def build_frames(log, max_frames=700):
    n = len(log)
    step = max(1, n // max_frames)
    worst_idx = min(range(n), key=lambda i: log[i][4])
    keep = sorted(set(range(0, n, step)) | {0, n - 1, worst_idx})
    frames = []
    for i in keep:
        t, phase, cx, cy, clr = log[i]
        sx, sy = to_svg(cx, cy)
        frames.append({
            "t": round(t, 2), "phase": phase,
            "x": round(sx, 2), "y": round(sy, 2),
            "clr": round(clr, 3), "status": bucket(clr),
        })
    return frames, keep.index(worst_idx)


def main():
    route_path = sys.argv[1] if len(sys.argv) > 1 else str(Path(__file__).resolve().parents[1] / "route.json")
    out_path = sys.argv[2] if len(sys.argv) > 2 else str(Path(__file__).resolve().parents[1] / "diagrams" / "sim-flight.html")

    r = S.run(route_path)
    frames, worst_frame_idx = build_frames(r["log"])

    data = {
        "result": r["result"], "why": r["why"],
        "legs_done": r["legs_done"], "legs": r["legs"],
        "flight_s": round(r["flight_s"], 1),
        "worst_clearance_m": round(r["worst_clearance_m"], 3),
        "worst_frame_idx": worst_frame_idx,
        "left_net": r["left_net"],
        "counts": r["counts"],
        "frames": frames,
        "drone_radius_m": S.DRONE_RADIUS_M,
    }

    template = Path(__file__).with_name("flight_template.html").read_text()
    html = template.replace("__FLIGHT_DATA__", json.dumps(data))
    Path(out_path).write_text(html)
    print(f"wrote {out_path}")
    print(f"  result={r['result']}  legs={r['legs_done']}/{r['legs']}  "
          f"flight={r['flight_s']:.1f}s  worst_clearance={r['worst_clearance_m']:.3f}m  "
          f"frames_embedded={len(frames)}")


if __name__ == "__main__":
    main()
