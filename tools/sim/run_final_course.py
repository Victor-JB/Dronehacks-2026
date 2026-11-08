"""Fly the CAD-free policy (Explorer + CommandSafety, same code as nav_policy) through FINAL Course v3 in the
offline simulator and record everything a presentation video needs. The policy never sees the CAD: it only
gets rendered depth/RGB and noisy odometry; the course is ground truth for rendering and collision scoring.

  python tools/sim/run_final_course.py out.pkl [--t-max 600] [--noise]
"""
import argparse
import os
import pickle
import sys
import time

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path[:0] = [os.path.join(ROOT, "ros2_ws", "src", "dh_core"), ROOT]
from dh_core.nav.online_map import MapParams          # noqa: E402
from dh_core.nav.sim_mesh_world import VoxelWorld      # noqa: E402
from dh_core.nav.sim_run import Sim, SimConfig        # noqa: E402

LAUNCH = (-2.515, 0.915)                               # 'Launch mark' centre in the STEP
GEOFENCE = (-2.8, 2.8, -2.8, 2.8)                      # 0.25 m inside the nets


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--t-max", type=float, default=600.0)
    ap.add_argument("--noise", action="store_true")
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--verbose", action="store_true")
    a = ap.parse_args()
    w = VoxelWorld.load(os.path.join(ROOT, "tools", "sim", "final_course_v3.npz"))
    cfg = SimConfig(t_max=a.t_max)
    if a.noise:
        cfg = SimConfig(t_max=a.t_max, depth_noise=0.02, depth_dropout=0.10, odom_noise=0.01, odom_drift_per_m=0.01, seed=1)
    sim = Sim(w, cfg, start=(LAUNCH[0], LAUNCH[1], 0.75), yaw0=0.0,
              mprm=MapParams(lo_xy=(-3.1, -3.1), size_xy=6.2), geofence=GEOFENCE)
    frames, seen = [], [0]
    render = w.render
    t_wall = time.time()

    def rec_render(o, R, K, ww, hh, *args, **kw):
        depth, rgb = render(o, R, K, ww, hh, *args, **kw)
        if not a.no_record:
            ex = sim.ex
            fr = dict(pos=sim.pos.copy(), yaw=sim.yaw, vel=sim.vel.copy(), state=ex.state, depth=depth, rgb=rgb,
                      goal=None if ex.goal is None else np.array(ex.goal), log_n=len(ex.log))
            f = ex.follower
            if f is not None:
                try:
                    fr["path"] = np.array([f.traj.sample(tt)[0] for tt in np.arange(0, 60, 0.5)])
                except Exception:
                    pass
            if len(frames) % 5 == 0 and ex.cls is not None:
                fr["cls"] = ex.cls.copy()
            frames.append(fr)
            if a.verbose:
                for L in ex.log[seen[0]:]:
                    print("  t=%.1f %s" % (len(frames) * 0.1, L), flush=True)
                seen[0] = len(ex.log)
            if len(frames) % 100 == 0:
                print("t=%.0f state=%s pos=%s wall=%.0fs" % (len(frames) * 0.1, ex.state, np.round(sim.pos, 2),
                                                             time.time() - t_wall), flush=True)
        return depth, rgb

    w.render = rec_render
    r = sim.run()
    summary = dict(reached=r.reached, state=r.final_state, sim_t=round(r.time, 1), collisions=r.collisions,
                   min_clear=round(r.min_clearance, 3), path=round(r.path_len, 1),
                   pad_err=None if r.pad_error is None else round(r.pad_error, 3),
                   z=(round(r.min_z, 3), round(r.max_z, 3)), replans=r.replans, wall_s=round(time.time() - t_wall))
    print(summary, flush=True)
    with open(a.out, "wb") as f:
        pickle.dump(dict(summary=summary, frames=frames, log=r.log, traj=r.traj, pad=w.pad,
                         map_lo=(-3.1, -3.1), map_voxel=MapParams().voxel, geofence=GEOFENCE), f)


if __name__ == "__main__":
    main()
