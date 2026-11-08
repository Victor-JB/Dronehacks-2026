"""Deterministic replay: (rx, kind, msg) events -> ShadowMonitor -> reports at a fixed evaluation rate.
PURE. The clock is the event receipt time, never the wall clock, so a replay is bit-for-bit repeatable
(processing-time fields, which ARE wall clock, are dropped by `deterministic`).

There is no output channel other than the returned list: no publisher, no socket, no file unless the
caller writes one.
"""
from __future__ import annotations

import csv
import json
from types import SimpleNamespace as NS
from typing import Iterable, List, Tuple

from .monitor import ShadowMonitor, deterministic
from .synth import odom_msg

KINDS = ("odom", "vo", "state", "camera_info", "depth")


def feed(mon: ShadowMonitor, kind: str, msg, rx: float) -> None:
    if kind == "odom":
        mon.on_odom(msg, rx)
    elif kind == "vo":
        mon.on_vo_status(int(msg), rx)
    elif kind == "state":
        mon.on_state(msg.data, rx)
    elif kind == "camera_info":
        mon.on_camera_info(msg, rx)
    elif kind == "depth":
        mon.on_depth(msg, rx)
    else:
        raise ValueError("unknown event kind %r" % kind)


def replay(events: Iterable[Tuple[float, str, object]], mon: ShadowMonitor, eval_hz: float = 10.0,
           keep_processing: bool = False) -> List[dict]:
    # canonical order: receipt time, then a fixed kind order (input order never matters)
    ev = sorted(((float(rx), KINDS.index(k), k, m) for rx, k, m in events), key=lambda e: (e[0], e[1]))
    if not ev:
        return []
    out, k = [], 0
    t, t_end, dt = ev[0][0], ev[-1][0], 1.0 / eval_hz
    step = 0
    while t <= t_end + 1e-9:
        while k < len(ev) and ev[k][0] <= t + 1e-12:
            feed(mon, ev[k][2], ev[k][3], ev[k][0])
            k += 1
        rep = mon.evaluate(t)
        out.append(rep if keep_processing else deterministic(rep))
        step += 1
        t = ev[0][0] + step * dt
    return out


def to_jsonl(reports: List[dict]) -> str:
    return "\n".join(json.dumps(r, sort_keys=True, default=float) for r in reports) + "\n"


def load_odom_trace(path: str) -> List[Tuple[float, str, object]]:
    """CSV written by tools/replay/extract_odom_trace.py: real odometry + VO status + state events."""
    ev = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            rx = float(row["rx"])
            kind = row["kind"]
            if kind == "odom":
                q = tuple(float(row[c]) for c in ("qx", "qy", "qz", "qw"))
                m = odom_msg(float(row["stamp"]), [float(row[c]) for c in ("x", "y", "z")], q,
                             [float(row[c]) for c in ("vx", "vy", "vz")], pos_var=float(row["cov_xx"]))
                m.pose.covariance[7] = float(row["cov_yy"])
                m.pose.covariance[14] = float(row["cov_zz"])
                ev.append((rx, "odom", m))
            elif kind == "vo":
                ev.append((rx, "vo", int(float(row["vo_state"]))))
            elif kind == "state":
                ev.append((rx, "state", NS(data=row["json"])))
    return ev
