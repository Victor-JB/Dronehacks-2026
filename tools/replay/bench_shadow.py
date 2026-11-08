"""Processing-time benchmark of the shadow pipeline on a synthetic 20 s approach (laptop numbers; the Orin
will differ and is NOT measured here). Usage: python tools/replay/bench_shadow.py"""
import os
import sys
import time

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path[:0] = [ROOT, os.path.join(ROOT, "ros2_ws", "src", "dh_core")]
from dh_core.shadow.monitor import ShadowMonitor  # noqa: E402
from dh_core.shadow.replay import replay  # noqa: E402
from dh_core.shadow.synth import Scenario, bag_extrinsic, straight  # noqa: E402

sc = Scenario(straight(v=0.3), boxes=[(7.0, 7.5, -4, 4, 0, 5), (3.0, 3.3, 0.6, 1.2, 0, 2)], duration=20.0, depth_hz=30.0)
ev = sc.events()
mon = ShadowMonitor(); mon.set_extrinsic(bag_extrinsic())
t = time.perf_counter()
reps = replay(ev, mon, keep_processing=True)
wall = time.perf_counter() - t
ev_s = np.array([r["_eval_proc_s"] for r in reps])
d = list(mon.streams["depth"].proc)
o = list(mon.streams["odom"].proc)
print("events %d, reports %d, replay wall %.2f s for %.0f s of data" % (len(ev), len(reps), wall, sc.duration))
print("depth frame (320x200, stride 4): p50 %.2f ms  p95 %.2f ms  max %.2f ms" % (
    1e3 * np.median(d), 1e3 * np.percentile(d, 95), 1e3 * max(d)))
print("odometry msg: p50 %.3f ms  max %.3f ms" % (1e3 * np.median(o), 1e3 * max(o)))
print("evaluate(): p50 %.3f ms  max %.3f ms" % (1e3 * np.median(ev_s), 1e3 * ev_s.max()))
