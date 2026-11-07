#!/usr/bin/env python3
"""READ-ONLY live report for the powered, stationary (props off, flight battery out) hardware test.

Subscribes only (best-effort QoS, so it never changes the publishers' behaviour) and prints, every --period s:
  rates and capture->now latency of camera, odometry, stereo scan, forward clearance, map fixes;
  forward-clearance verdicts (clear / obstacle / unknown) and the mission's health line;
  STATIONARY localization quality: spread of /dh/pose_map and of the raw odometry while the drone does not move;
  dry-run commands (/dh/dry_run/cmd_vel): rate and magnitude.
A stationary drone cannot validate flight: the trajectory clock freezes once the reference runs 0.35 m ahead,
and the commands then only describe "fly toward the frozen reference". That is expected.

  python3 tools/hil/stationary_report.py [--period 2] [--duration 60]
"""
import argparse
import math
import time
from collections import defaultdict, deque


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--period", type=float, default=2.0)
    ap.add_argument("--duration", type=float, default=60.0)
    args = ap.parse_args()
    import rclpy
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, LaserScan, Range
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("dh_stationary_report_readonly")
    now = lambda: node.get_clock().now().nanoseconds * 1e-9
    stamp = lambda m: m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
    count = defaultdict(int)
    lat = defaultdict(lambda: deque(maxlen=200))
    poses, odoms = deque(maxlen=2000), deque(maxlen=2000)
    fc = defaultdict(int)
    health = {"last": ""}
    cmds = deque(maxlen=200)

    def stamped(name):
        def cb(m):
            count[name] += 1
            s = stamp(m)
            if s > 0:
                lat[name].append(now() - s)
            if name == "odom":
                p = m.pose.pose.position; odoms.append((p.x, p.y, p.z))
            elif name == "pose_map":
                p = m.pose.position; poses.append((p.x, p.y, p.z))
            elif name == "clearance":
                r = m.range
                fc["unknown" if math.isnan(r) else ("clear" if r == math.inf else "obstacle")] += 1
            elif name == "dry_cmd":
                v = m.twist.linear; cmds.append(math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z))
        return cb
    q = qos_profile_sensor_data
    node.create_subscription(Image, "/oak/left/image_raw", stamped("left_image"), q)
    node.create_subscription(Image, "/oak/right/image_raw", stamped("right_image"), q)
    node.create_subscription(Odometry, "/visual_slam/tracking/odometry", stamped("odom"), q)
    node.create_subscription(LaserScan, "/dh/stereo_scan", stamped("stereo_scan"), q)
    node.create_subscription(Range, "/dh/forward_clearance", stamped("clearance"), q)
    node.create_subscription(PoseStamped, "/dh/pose_map", stamped("pose_map"), q)
    node.create_subscription(PoseWithCovarianceStamped, "/dh/map_fix", stamped("map_fix"), q)
    node.create_subscription(TwistStamped, "/dh/dry_run/cmd_vel", stamped("dry_cmd"), q)
    node.create_subscription(String, "/dh/health", lambda m: health.__setitem__("last", m.data), q)

    def spread(buf):
        if len(buf) < 5:
            return "n/a"
        xs = [b[0] for b in buf]; ys = [b[1] for b in buf]
        mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
        sd = math.sqrt(sum((x - mx) ** 2 + (y - my) ** 2 for x, y in zip(xs, ys)) / len(xs))
        return "xy std %.3f m, span %.3f m" % (sd, math.hypot(max(xs) - min(xs), max(ys) - min(ys)))

    t0 = time.time(); last = t0; prev = dict(count)
    while rclpy.ok() and time.time() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.time() - last >= args.period:
            dt = time.time() - last; last = time.time()
            print("---- t=%.0f s" % (last - t0))
            for k in ("left_image", "right_image", "odom", "stereo_scan", "clearance", "pose_map", "map_fix", "dry_cmd"):
                rate = (count[k] - prev.get(k, 0)) / dt
                l = lat[k]
                print("  %-12s %5.1f Hz  latency median %s" % (k, rate, ("%.0f ms" % (1e3 * sorted(l)[len(l) // 2])) if l else "n/a"))
            print("  clearance verdicts so far: %s" % dict(fc))
            print("  stationary odometry  : " + spread(odoms))
            print("  stationary pose_map  : " + spread(poses))
            print("  health: " + (health["last"] or "n/a"))
            if cmds:
                print("  dry-run |cmd| last %.2f m/s, max %.2f m/s (NOT sent to the flight controller)" % (cmds[-1], max(cmds)))
            prev = dict(count)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
