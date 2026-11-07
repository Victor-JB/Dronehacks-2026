#!/usr/bin/env python3
"""Tiny local viewer for the real Gazebo GUI and its telemetry JSONL."""
from __future__ import annotations

import json
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parent
TELEMETRY = REPO / "artifacts" / "gazebo" / "telemetry.jsonl"
VNC_URL = "http://127.0.0.1:8080/vnc.html?autoconnect=1&resize=scale"
MAX_TAIL = 128 * 1024
MAX_HEAD = 32 * 1024
STALE_SECONDS = 3.0


def read_records():
    """Read bounded head and tail windows; ignore a partial leading tail line."""
    try:
        stat = TELEMETRY.stat()
        with TELEMETRY.open("rb") as f:
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


def read_summary_sidecar():
    """Course writer atomically refreshes this small file at mission finish."""
    path = TELEMETRY.with_name(TELEMETRY.stem + ".summary.json")
    try:
        row = json.loads(path.read_text())
        return row if isinstance(row, dict) and row.get("type") == "summary" else None
    except (OSError, ValueError):
        return None


def state():
    stat, head, tail = read_records()
    rows = head + tail
    sample = next((r for r in reversed(tail) if r.get("type") == "sample" and
                   isinstance(r.get("position"), list) and len(r["position"]) >= 3), None)
    now = time.time()
    age = max(0.0, now - stat.st_mtime) if stat else None
    status = "waiting" if sample is None else ("stale" if age is None or age > STALE_SECONDS else "live")
    metadata = next((r for r in head if r.get("type") == "metadata"), None)
    if metadata is None:
        metadata = next((r for r in tail if r.get("type") == "metadata"), None)
    summary = read_summary_sidecar()
    if summary is None:
        summary = next((r for r in reversed(tail) if r.get("type") == "summary"), None)
    events = [r for r in tail if r.get("type") == "event"][-8:]
    return {
        "status": status, "file_age_s": round(age, 2) if age is not None else None,
        "sample": sample, "metadata": metadata, "summary": summary, "events": events,
        "source": "artifacts/gazebo/telemetry.jsonl", "gazebo_url": VNC_URL,
        "labels": ["REAL GAZEBO PHYSICS", "GROUND-TRUTH SIM STATE", "NOT PX4", "NOT A REAL DRONE"],
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
        if urlparse(self.path).path == "/":
            self.path = "/index.html"
        return super().do_GET()

    def log_message(self, fmt, *args):
        print("[viewer] " + fmt % args)


if __name__ == "__main__":
    server = ThreadingHTTPServer(("127.0.0.1", 8081), Handler)
    print("Gazebo viewer: http://127.0.0.1:8081")
    server.serve_forever()
