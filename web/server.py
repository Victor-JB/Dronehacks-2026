#!/usr/bin/env python3
"""Tiny local viewer for the real Gazebo GUI and its telemetry JSONL."""
from __future__ import annotations

import json
import os
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parent
TELEMETRY = REPO / "artifacts" / "gazebo" / "telemetry.jsonl"
PX4_TELEMETRY = REPO / "artifacts" / "px4" / "telemetry.jsonl"
VISION = REPO / "artifacts" / "vision"
VNC_URL = "http://127.0.0.1:8080/vnc.html?autoconnect=1&resize=scale"
MAX_TAIL = 128 * 1024
MAX_HEAD = 32 * 1024
STALE_SECONDS = 3.0


def telemetry_path():
    preferred = os.environ.get("DRONEHACKS_TELEMETRY")
    if preferred:
        path = Path(preferred)
        return path if path.is_absolute() else REPO / path
    try:
        if PX4_TELEMETRY.exists() and (not TELEMETRY.exists() or
                                      PX4_TELEMETRY.stat().st_mtime > TELEMETRY.stat().st_mtime):
            return PX4_TELEMETRY
    except OSError:
        pass
    return TELEMETRY


def read_records(path):
    """Read bounded head and tail windows; ignore a partial leading tail line."""
    try:
        stat = path.stat()
        with path.open("rb") as f:
            head = f.read(MAX_HEAD)
            if stat.st_size <= MAX_TAIL:
                tail = head + f.read()
            else:
                f.seek(-MAX_TAIL, 2)
                f.readline()
                tail = f.read()
    except FileNotFoundError:
        return None, [], []

    def parse_window(raw):
        parsed = []
        for line in raw.splitlines():
            try:
                row = json.loads(line)
                if isinstance(row, dict):
                    parsed.append(row)
            except (ValueError, UnicodeDecodeError):
                continue
        return parsed

    return stat, parse_window(head), parse_window(tail)


def read_summary_sidecar(path):
    """Course writer atomically refreshes this small file at mission finish."""
    path = path.with_name(path.stem + ".summary.json")
    try:
        row = json.loads(path.read_text())
        return row if isinstance(row, dict) and row.get("type") == "summary" else None
    except (OSError, ValueError):
        return None


def state():
    path = telemetry_path()
    stat, head, tail = read_records(path)
    sample = next((r for r in reversed(tail) if r.get("type") == "sample" and
                   isinstance(r.get("position"), list) and len(r["position"]) >= 3), None)
    now = time.time()
    age = max(0.0, now - stat.st_mtime) if stat else None
    status = "waiting" if sample is None else ("stale" if age is None or age > STALE_SECONDS else "live")
    metadata = next((r for r in head if r.get("type") == "metadata"), None)
    if metadata is None:
        metadata = next((r for r in tail if r.get("type") == "metadata"), None)
    summary = read_summary_sidecar(path)
    if summary is None:
        summary = next((r for r in reversed(tail) if r.get("type") == "summary"), None)
    if metadata and metadata.get("run_id") and summary:
        if summary.get("run_id") != metadata["run_id"]:
            summary = None
    events = [r for r in tail if r.get("type") == "event"][-8:]
    is_px4 = bool(metadata and metadata.get("source") == "PX4 EKF2 + camera marker localization")
    vision = None
    vision_age = None
    vision_wall_age = None
    if is_px4:
        try:
            vision = json.loads((VISION / "latest.json").read_text())
            vision_wall_age = max(0.0, now - (VISION / "latest.json").stat().st_mtime)
            if not isinstance(vision, dict):
                vision = None
            elif sample and vision.get("stamp_sim_s") is not None and sample.get("sim_time") is not None:
                vision_age = max(0.0, float(sample["sim_time"]) - float(vision["stamp_sim_s"]))
        except (OSError, ValueError, TypeError):
            vision = None
    return {
        "status": status, "file_age_s": round(age, 2) if age is not None else None,
        "sample": sample, "metadata": metadata, "summary": summary, "events": events,
        "source": str(path.relative_to(REPO)) if path.is_relative_to(REPO) else str(path), "gazebo_url": VNC_URL,
        "mode": "px4" if is_px4 else "gazebo",
        "vision": vision, "vision_age_s": round(vision_age, 2) if vision_age is not None else None,
        "vision_wall_age_s": round(vision_wall_age, 2) if vision_wall_age is not None else None,
        "camera_url": "/camera.jpg" if is_px4 and (VISION / "camera.jpg").is_file() else None,
        "labels": (["PX4 SITL", "CAMERA POSITION", "MAPPED FIDUCIALS", "NOT HARDWARE VALIDATED"]
                   if is_px4 else ["REAL GAZEBO PHYSICS", "GROUND-TRUTH SIM STATE", "NOT PX4", "NOT A REAL DRONE"]),
    }


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT), **kwargs)

    def do_GET(self):
        if urlparse(self.path).path == "/api/state":
            body = json.dumps(state()).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if urlparse(self.path).path == "/camera.jpg":
            image = VISION / "camera.jpg"
            if image.is_file():
                body = image.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            else:
                self.send_error(404)
            return
        if urlparse(self.path).path == "/":
            self.path = "/index.html"
        return super().do_GET()

    def log_message(self, fmt, *args):
        print("[viewer] " + fmt % args)


if __name__ == "__main__":
    server = ThreadingHTTPServer(("127.0.0.1", 8081), Handler)
    print("Gazebo viewer: http://127.0.0.1:8081")
    server.serve_forever()
