#!/usr/bin/env python3
"""Build a self-contained HTML replay from a recording made by onboard/record_flight.py.

    python3 tools/build_real_flight_viz.py recordings/run1/manifest.json diagrams/real-flight.html

Projects the real recorded trajectory onto the same scaled floor plan as diagrams/route-plan.html,
colors it by the nearest depth frame's reported clearance (camera-reported, not ground truth --
there is no independent oracle for a real run), and embeds the recorded depth/rgb thumbnails as a
scrubbable camera feed.
"""
import base64
import bisect
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "local_sim"))
from course import HALF, LAUNCH_XY  # noqa: E402

SCALE = 90.0  # px per metre -- matches diagrams/route-plan.html
DRONE_RADIUS_M = 0.25  # same placeholder used throughout local_sim/ -- see its README


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


def nearest_by_t(sorted_list, t, max_gap=1.5):
    """sorted_list: list of dicts with a 't' key, ascending. Nearest within max_gap seconds."""
    if not sorted_list:
        return None
    ts = [e["t"] for e in sorted_list]
    i = bisect.bisect_left(ts, t)
    candidates = [c for c in (i - 1, i) if 0 <= c < len(ts)]
    if not candidates:
        return None
    best = min(candidates, key=lambda c: abs(ts[c] - t))
    return sorted_list[best] if abs(ts[best] - t) <= max_gap else None


def b64_png(path):
    try:
        return "data:image/png;base64," + base64.b64encode(Path(path).read_bytes()).decode("ascii")
    except OSError:
        return None


def main():
    if len(sys.argv) < 2:
        raise SystemExit("usage: build_real_flight_viz.py <manifest.json> [out.html]")
    manifest_path = Path(sys.argv[1])
    out_path = Path(sys.argv[2]) if len(sys.argv) > 2 else Path("diagrams/real-flight.html")
    rec_dir = manifest_path.parent

    doc = json.loads(manifest_path.read_text())
    traj = sorted(doc.get("trajectory", []), key=lambda e: e["t"])
    status_log = sorted(doc.get("status_log", []), key=lambda e: e["t"])
    depth_frames = sorted(doc.get("depth_frames", []), key=lambda e: e["t"])
    rgb_frames = sorted(doc.get("rgb_frames", []), key=lambda e: e["t"])

    if not traj:
        raise SystemExit(f"{manifest_path}: no trajectory samples recorded -- nothing to visualize")

    # one shared epoch across every recorded stream (they share the recorder's clock already,
    # just offset by whichever topic happened to arrive first), so playback stays in sync
    t0 = min([traj[0]["t"]]
             + ([status_log[0]["t"]] if status_log else [])
             + ([depth_frames[0]["t"]] if depth_frames else [])
             + ([rgb_frames[0]["t"]] if rgb_frames else []))
    flight_s = traj[-1]["t"] - t0

    frames = []
    worst_clr, worst_idx = math.inf, None
    for i, p in enumerate(traj):
        t = p["t"] - t0
        cx, cy = LAUNCH_XY[0] + p["e"], LAUNCH_XY[1] + p["n"]
        sx, sy = to_svg(cx, cy)
        df = nearest_by_t(depth_frames, p["t"])
        clr = (df["min_depth_m"] - DRONE_RADIUS_M) if (df and df.get("min_depth_m") is not None) else None
        st = nearest_by_t(status_log, p["t"])
        s = (st or {}).get("status", {}) or {}
        frames.append({
            "t": round(t, 2), "x": round(sx, 2), "y": round(sy, 2),
            "clr": round(clr, 3) if clr is not None else None,
            "status": bucket(clr),
            "phase": s.get("phase"), "why": s.get("why"),
            "armed": p.get("armed"), "landed": p.get("landed"),
        })
        if clr is not None and clr < worst_clr:
            worst_clr, worst_idx = clr, i

    # downsample for a lighter page, keeping endpoints + the worst-clearance frame
    max_frames = 800
    step = max(1, len(frames) // max_frames)
    keep = sorted(set(range(0, len(frames), step)) | {0, len(frames) - 1} | ({worst_idx} if worst_idx is not None else set()))
    frames_ds = [frames[i] for i in keep]
    worst_ds_idx = keep.index(worst_idx) if worst_idx is not None else None

    depth_images = {f["path"]: b64_png(rec_dir / f["path"]) for f in depth_frames}
    depth_images = {k: v for k, v in depth_images.items() if v}
    rgb_images = {f["path"]: b64_png(rec_dir / f["path"]) for f in rgb_frames}
    rgb_images = {k: v for k, v in rgb_images.items() if v}

    final_status = status_log[-1]["status"] if status_log else {}
    data = {
        "result": final_status.get("result") or final_status.get("phase"),
        "pose_count": len(traj),
        "flight_s": round(flight_s, 1),
        "depth_frame_count": len(depth_frames),
        "rgb_frame_count": len(rgb_frames),
        "worst_clearance_m": round(worst_clr, 3) if math.isfinite(worst_clr) else None,
        "worst_frame_idx": worst_ds_idx,
        "frames": frames_ds,
        "status_index": [{"t": round(e["t"] - t0, 2), "status": e["status"]} for e in status_log],
        "depth_index": [{"t": round(f["t"] - t0, 2), "path": f["path"]} for f in depth_frames if f["path"] in depth_images],
        "rgb_index": [{"t": round(f["t"] - t0, 2), "path": f["path"]} for f in rgb_frames if f["path"] in rgb_images],
        "depth_images": depth_images,
        "rgb_images": rgb_images,
    }

    template = Path(__file__).with_name("real_flight_template.html").read_text()
    html = template.replace("__FLIGHT_DATA__", json.dumps(data))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(html)
    size_mb = out_path.stat().st_size / 1e6
    print(f"wrote {out_path} ({size_mb:.1f} MB): {len(frames_ds)} frames, "
          f"{len(depth_images)} depth images, {len(rgb_images)} rgb images, "
          f"worst camera clearance {data['worst_clearance_m']}")


if __name__ == "__main__":
    main()
