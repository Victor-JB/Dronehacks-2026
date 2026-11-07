"""Analyse a flight bag (rosbag2 / mcap) for debugging and for the presentation.

Run inside a ROS 2 Humble environment (rosbag2_py available), e.g. in the skybeta container:
  python3 tools/analyze_bag.py /bags/20261107-1530 out_prefix

Record the bag with the organizers' tool plus our topics, e.g.:
  ros2 bag record -s mcap -o /bags/$(date +%Y%m%d-%H%M%S) /dh/plan /dh/pose_map /dh/mission_state \
     /dh/map_fix /dh/loc_health /dh/health /dh/line_obs /dh/forward_clearance /dh/stereo_scan /dh/dry_run/cmd_vel \
     /visual_slam/tracking/odometry /visual_slam/status /skybeta/state /fmu/out/vehicle_local_position

Outputs: <prefix>_xy.png (plan vs flown in the course frame, coloured by speed, corrections marked),
<prefix>_timeline.png (state, speed, localizer health, correction size), and a printed summary
(phase durations, corrections accepted/rejected, max correction).
Logs are for analysis only: never feed them back into a later run (rule 6.4).
"""
import math
import sys

import numpy as np


def read_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    storage = "mcap" if path.endswith(".mcap") or "mcap" in path else ""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = {}
    while reader.has_next():
        topic, data, t = reader.read_next()
        msg = deserialize_message(data, get_message(types[topic]))
        out.setdefault(topic, []).append((t * 1e-9, msg))
    return out


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    bag, prefix = sys.argv[1], sys.argv[2]
    d = read_bag(bag)
    t0 = min(v[0][0] for v in d.values())
    pose = np.array([[t - t0, m.pose.position.x, m.pose.position.y, m.pose.position.z]
                     for t, m in d.get("/dh/pose_map", [])])
    plan = d.get("/dh/plan", [])
    plan_xy = np.array([[p.pose.position.x, p.pose.position.y] for p in plan[-1][1].poses]) if plan else None
    states = [(t - t0, m.data) for t, m in d.get("/dh/mission_state", [])]
    # map fixes are absolute course poses at the scan's capture time; the "correction" is fix - estimate then
    fixes = [(t - t0, m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, m.pose.pose.position.x, m.pose.pose.position.y)
             for t, m in d.get("/dh/map_fix", [])]
    corr = []
    if len(pose) and fixes:
        pst = np.array([[m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec] for _, m in d.get("/dh/pose_map", [])]).ravel()
        for tr, ts, x, y in fixes:
            k = int(np.argmin(np.abs(pst - ts)))
            corr.append([tr, x - pose[k, 1], y - pose[k, 2], 0.0])
    corr = np.array(corr)
    hl = [(t - t0, m.data) for t, m in d.get("/dh/health", [])]
    if hl:
        print("  health states seen: %s" % sorted({h.split(" ")[0] + " " + h.split(" ")[1] for _, h in hl}))
    health = np.array([[t - t0, *m.data] for t, m in d.get("/dh/loc_health", [])])

    # ---- summary
    print("bag:", bag)
    seen, phases = None, []
    for t, s in states:
        if s != seen:
            phases.append((t, s)); seen = s
    for (ta, sa), (tb, _) in zip(phases, phases[1:] + [(states[-1][0] if states else 0, "")]):
        print("  %-12s %6.2f s" % (sa, tb - ta))
    if len(corr):
        mag = np.hypot(corr[:, 1], corr[:, 2])
        print("  corrections accepted: %d, median %.3f m, max %.3f m" % (len(corr), np.median(mag), mag.max()))
    if len(health):
        print("  scans matched: %d, rejected by health check: %d" % (len(health), int((health[:, 1] < 0.5).sum())))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    if len(pose):
        sp = np.r_[0, np.hypot(np.diff(pose[:, 1]), np.diff(pose[:, 2])) / np.maximum(np.diff(pose[:, 0]), 1e-3)]
        fig, ax = plt.subplots(figsize=(7, 7))
        if plan_xy is not None:
            ax.plot(plan_xy[:, 0], plan_xy[:, 1], "k--", lw=1, label="plan")
        sc = ax.scatter(pose[:, 1], pose[:, 2], c=np.clip(sp, 0, 1.5), s=3, cmap="viridis", label="estimate")
        if len(corr):
            idx = [int(np.argmin(np.abs(pose[:, 0] - t))) for t in corr[:, 0]]
            ax.quiver(pose[idx, 1], pose[idx, 2], corr[:, 1], corr[:, 2], angles="xy", scale_units="xy", scale=1,
                      color="r", width=0.003, label="map corrections")
        plt.colorbar(sc, ax=ax, label="speed (m/s)")
        ax.set_aspect("equal"); ax.legend(); ax.set_xlim(-3.1, 3.1); ax.set_ylim(-3.1, 3.1)
        ax.set_title("Flight in the course frame")
        fig.savefig(prefix + "_xy.png", dpi=120, bbox_inches="tight")
        fig, axs = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
        axs[0].plot(pose[:, 0], sp); axs[0].set_ylabel("speed (m/s)")
        for t, s in phases:
            axs[0].axvline(t, color="grey", lw=0.5); axs[0].text(t, axs[0].get_ylim()[1] * 0.9, s, fontsize=7, rotation=90)
        axs[1].plot(pose[:, 0], pose[:, 3]); axs[1].set_ylabel("z (m)")
        if len(corr):
            axs[2].stem(corr[:, 0], np.hypot(corr[:, 1], corr[:, 2]))
        axs[2].set_ylabel("|correction| (m)"); axs[2].set_xlabel("time (s)")
        fig.savefig(prefix + "_timeline.png", dpi=120, bbox_inches="tight")
        print("wrote", prefix + "_xy.png", prefix + "_timeline.png")


if __name__ == "__main__":
    main()
