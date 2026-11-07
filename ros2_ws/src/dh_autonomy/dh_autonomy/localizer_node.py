"""Map-relative re-localisation node (scan -> known course distance field).

sub  /dh/pose_map        geometry_msgs/PoseStamped  current course-frame estimate (from mission node)
sub  <scan_topic>        sensor_msgs/LaserScan       body-frame scan (stereo-derived or 2D LiDAR)
pub  /dh/map_correction  geometry_msgs/Vector3Stamped (dx, dy, dyaw) = matched - estimate
pub  /dh/loc_health      std_msgs/Float32MultiArray [ok, inlier_ratio, rms, min_eig]

Only the observable part of the correction is published (see dh_core.localize).
"""
from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

from dh_core.course import Course
from dh_core.localize import DistanceFieldMatcher


class LocalizerNode(Node):
    def __init__(self):
        super().__init__("dh_localizer")
        P = self.declare_parameter
        P("course_obj", "/root/team_ws/course/dronehacks-course-v3.obj")
        P("scan_topic", "/dh/stereo_scan")
        P("res", 0.025)
        P("band_lo", 0.58)
        P("band_hi", 1.12)
        P("sensor_x", 0.0)           # scan origin ahead of the body origin (m): 0 for /dh/stereo_scan
                                     # (already body-relative); the LiDAR mount offset for a LiDAR /scan
        P("max_correction", 0.15)    # m per fix in normal tracking
        P("gate_growth", 0.25)       # m/s the gate widens while no fix is accepted (blackouts)
        P("confirm_tol", 0.08)       # m; a jump > max_correction needs 2 consistent matches
        P("max_yaw_correction", 0.15)
        P("consistent_n", 3)            # out-of-gate fix accepted after this many agreeing high-quality matches
        P("consistent_tol", 0.10)       # m; how closely they must agree
        P("consistent_min_inlier", 0.85)
        P("consistent_max_rms", 0.03)   # m
        P("consistent_max_jump", 1.0)   # m; never jump further than this
        P("max_pose_age", 0.15)
        P("min_points", 25)
        g = lambda n: self.get_parameter(n).value
        course = Course.from_obj(g("course_obj"))
        t0 = time.time()
        grid = course.occupancy(g("res"), g("band_lo"), g("band_hi"))
        self.matcher = DistanceFieldMatcher(grid)
        self.get_logger().info("distance field ready (%.1fs, %s cells)" % (time.time() - t0, grid.shape))
        self.pose = None
        self.last_accept = time.time()
        self.pending = None
        self.recent = []
        self.n_consistent = 0
        self.create_subscription(PoseStamped, "/dh/pose_map", self._on_pose, 10)
        self.create_subscription(LaserScan, g("scan_topic"), self._on_scan, 5)
        self.pub = self.create_publisher(Vector3Stamped, "/dh/map_correction", 10)
        self.pub_h = self.create_publisher(Float32MultiArray, "/dh/loc_health", 10)

    def _on_pose(self, m: PoseStamped):
        q = m.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (np.array([m.pose.position.x, m.pose.position.y, yaw]), time.time())

    def _on_scan(self, m: LaserScan):
        g = lambda n: self.get_parameter(n).value
        if self.pose is None or time.time() - self.pose[1] > g("max_pose_age"):
            return
        r = np.asarray(m.ranges, float)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r > m.range_min) & (r < m.range_max)
        if ok.sum() < g("min_points"):
            return
        pts = np.stack([r[ok] * np.cos(a[ok]) + g("sensor_x"), r[ok] * np.sin(a[ok])], 1)
        prior = self.pose[0]
        res = self.matcher.match(pts, prior)
        h = Float32MultiArray()
        h.data = [float(res.ok), res.inlier_ratio, float(min(res.residual_rms, 9.9)), res.min_eig]
        self.pub_h.publish(h)
        if not res.ok:
            return
        d = res.pose - prior
        d[2] = (d[2] + math.pi) % (2 * math.pi) - math.pi
        now = time.time()
        # Allowed jump grows while no fix has been accepted (e.g. after the dark tube),
        # but a large jump must be confirmed by two consecutive, consistent matches.
        allowed = g("max_correction") + g("gate_growth") * (now - self.last_accept)
        mag = float(np.linalg.norm(d[:2]))
        if mag > allowed or abs(d[2]) > g("max_yaw_correction"):
            self.pending = None
            # Consistency override: after a blackout the gate may still be too narrow (it is reset by any
            # small accept, and the mission applies only part of each fix). Several consecutive HIGH-QUALITY
            # matches that agree with each other are strong evidence, so accept their mean (up to a cap).
            good = (res.inlier_ratio >= g("consistent_min_inlier") and res.residual_rms <= g("consistent_max_rms")
                    and abs(d[2]) <= g("max_yaw_correction") and mag <= g("consistent_max_jump"))
            if not good:
                self.recent = []
                return
            self.recent = (self.recent + [d.copy()])[-int(g("consistent_n")):]
            R = np.array(self.recent)
            if len(R) < int(g("consistent_n")) or np.max(np.linalg.norm(R[:, :2] - R[:, :2].mean(0), axis=1)) > g("consistent_tol"):
                return
            d = R.mean(0)
            self.recent = []
            self.n_consistent += 1
            self.last_accept = now
            out = Vector3Stamped()
            out.header = m.header
            out.vector.x, out.vector.y, out.vector.z = float(d[0]), float(d[1]), float(d[2])
            self.pub.publish(out)
            return
        self.recent = []
        if mag > g("max_correction"):
            if self.pending is None or np.linalg.norm(self.pending - d[:2]) > g("confirm_tol"):
                self.pending = d[:2].copy()
                return
        self.pending = None
        self.last_accept = now
        out = Vector3Stamped()
        out.header = m.header
        out.vector.x, out.vector.y, out.vector.z = float(d[0]), float(d[1]), float(d[2])
        self.pub.publish(out)


def main():
    rclpy.init()
    n = LocalizerNode()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
