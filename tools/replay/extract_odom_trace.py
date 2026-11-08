"""Extract a replay fixture (odometry + VO status + /skybeta/state) from a drone MCAP bag.

Offline tool; reads a file and writes a CSV. Needs `pip install mcap mcap-ros2-support` (not a runtime
dependency). Tolerates bags without a summary/footer (a recording cut off by power-down), which is
the case for main's drone-logs/20261107-155647/20261107-155647_0.mcap.

  python tools/replay/extract_odom_trace.py BAG.mcap OUT.csv --from 160 --to 200

--from/--to are seconds relative to the first odometry message. Columns: kind, rx (bag log time,
receipt clock), stamp (header stamp), x y z, qx qy qz qw, vx vy vz (twist, child frame), cov_xx/yy/zz,
vo_state, json.
"""
import argparse
import csv
import io
import json

from mcap.records import Channel, Message, Schema
from mcap.stream_reader import StreamReader
from mcap_ros2.decoder import DecoderFactory

COLS = ["kind", "rx", "stamp", "x", "y", "z", "qx", "qy", "qz", "qw", "vx", "vy", "vz",
        "cov_xx", "cov_yy", "cov_zz", "vo_state", "json"]
TOPICS = {"/visual_slam/tracking/odometry": "odom", "/visual_slam/status": "vo", "/skybeta/state": "state"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("out")
    ap.add_argument("--from", dest="t_from", type=float, default=0.0)
    ap.add_argument("--to", dest="t_to", type=float, default=1e9)
    a = ap.parse_args()
    sr = StreamReader(io.BytesIO(open(a.bag, "rb").read()), skip_magic=False, record_size_limit=None)
    schemas, chans, decs, fac = {}, {}, {}, DecoderFactory()
    rows, t_first = [], None
    try:
        for r in sr.records:
            if isinstance(r, Schema):
                schemas[r.id] = r
            elif isinstance(r, Channel):
                chans[r.id] = r
            elif isinstance(r, Message):
                c = chans[r.channel_id]
                kind = TOPICS.get(c.topic)
                if kind is None:
                    continue
                dec = decs.setdefault(r.channel_id, fac.decoder_for(c.message_encoding, schemas[c.schema_id]))
                m = dec(r.data)
                rx = r.log_time * 1e-9
                if kind == "odom" and t_first is None:
                    t_first = rx
                if t_first is None or not (a.t_from <= rx - t_first <= a.t_to):
                    continue
                row = dict.fromkeys(COLS, "")
                row.update(kind=kind, rx="%.9f" % rx)
                if kind == "odom":
                    p, q, v, cv = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear, m.pose.covariance
                    row.update(stamp="%.9f" % (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9),
                               x=p.x, y=p.y, z=p.z, qx=q.x, qy=q.y, qz=q.z, qw=q.w, vx=v.x, vy=v.y, vz=v.z,
                               cov_xx=cv[0], cov_yy=cv[7], cov_zz=cv[14])
                elif kind == "vo":
                    row.update(vo_state=int(m.vo_state))
                else:
                    row.update(json=json.dumps(json.loads(m.data), separators=(",", ":")))
                rows.append(row)
    except Exception as e:                          # truncated tail: keep everything decoded so far
        print("stopped reading at a damaged record (%s); kept %d rows" % (type(e).__name__, len(rows)))
    with open(a.out, "w", newline="") as f:
        w = csv.DictWriter(f, COLS)
        w.writeheader()
        w.writerows(rows)
    print("wrote %d rows to %s" % (len(rows), a.out))


if __name__ == "__main__":
    main()
