"""Map-relative re-localisation node (scan -> known course distance field), time-synchronised.

sub  /dh/pose_map        geometry_msgs/PoseStamped  course-frame estimate from the mission node; its
                         header.stamp is the ODOMETRY stamp it was computed from (not "now")
sub  <scan_topic>        sensor_msgs/LaserScan       stereo pseudo-scan (or a 2D LiDAR); header.stamp = capture time
pub  /dh/map_fix         geometry_msgs/PoseWithCovarianceStamped
                         ABSOLUTE matched course-frame pose (x, y, yaw) at the scan's capture time
                         (header.stamp = scan stamp). The mission re-applies it against its own odometry
                         history at that stamp, so camera latency and corrections that happen in between
                         cannot produce a wrong correction.
pub  /dh/loc_health      std_msgs/Float32MultiArray [ok, inlier_ratio, rms, min_eig, latency_s]

Time handling: the prior for a scan is the pose INTERPOLATED AT THE SCAN STAMP from a pose history. Scans
without a stamp, older than the history, newer than the newest pose, or later than max_scan_latency are
rejected (counted in self.rejected). Out-of-order scans are each matched against their own time.
Frames: scans must be in `base_frame`; another frame is accepted only if TF base_frame <- scan frame is
available (static mount), otherwise rejected.
Only the observable part of the correction is applied by the matcher (see dh_core.localize).
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

from dh_core.camera_geom import quat_to_rot
from dh_core.course import Course
from dh_core.health import PoseHistory, stamp_to_sec, wrap
from dh_core.localize import DistanceFieldMatcher


def quat_to_yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class LocalizerNode(Node):
    def __init__(self):
        super().__init__("dh_localizer")
        P = self.declare_parameter
        P("course_obj", "/root/team_ws/course/dronehacks-course-v3.obj")
        P("scan_topic", "/dh/stereo_scan")
        P("base_frame", "base_link")
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
        P("consistent_min_eig", 0.10)
        P("consistent_max_jump", 0.30)   # m; never jump further than this
        P("max_scan_latency", 0.5)      # s; scans older than this (vs the newest pose) are stale
        P("pose_history_s", 3.0)
        P("min_points", 25)
        g = lambda n: self.get_parameter(n).value
        course = Course.from_obj(g("course_obj"))
        grid = course.occupancy(g("res"), g("band_lo"), g("band_hi"))
        self.matcher = DistanceFieldMatcher(grid)
        self.get_logger().info("distance field ready (%s cells)" % (grid.shape,))
        self.hist = PoseHistory(span_s=g("pose_history_s"), max_gap_s=0.25)
        self.last_accept = None
        self.pending = None
        self.recent = []
        self.n_consistent = 0
        self.rejected = {"unstamped": 0, "stale": 0, "future": 0, "no_pose": 0, "frame": 0, "few_points": 0,
                         "match": 0, "gate": 0}
        self.n_published = 0
        self.waiting_scans = []
        try:
            from tf2_ros import Buffer, TransformListener
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        except ImportError:
            self.tf_buffer = None
        self.create_subscription(PoseStamped, "/dh/pose_map", self._on_pose, 50)
        self.create_subscription(LaserScan, g("scan_topic"), self._on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/dh/map_fix", 10)
        self.pub_h = self.create_publisher(Float32MultiArray, "/dh/loc_health", 10)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_pose(self, m: PoseStamped):
        t = stamp_to_sec(m.header.stamp)
        p = m.pose.position
        if m.header.frame_id != "course":
            return
        self.hist.add(t, (p.x, p.y, p.z, quat_to_yaw(m.pose.orientation)))
        pending, self.waiting_scans = self.waiting_scans, []
        for scan in pending:
            self._on_scan(scan)

    def _scan_to_base(self, frame: str, stamp):
        """(dx, dy, dyaw) of the scan frame in base_frame, or None if it cannot be resolved."""
        g = lambda n: self.get_parameter(n).value
        if frame == g("base_frame"):
            return (g("sensor_x"), 0.0, 0.0)
        if self.tf_buffer is None:
            return None
        try:
            from rclpy.time import Time
            tr = self.tf_buffer.lookup_transform(g("base_frame"), frame, Time.from_msg(stamp)).transform
        except Exception:  # noqa: BLE001 - any tf2 failure = unknown frame
            return None
        q = tr.rotation
        R = quat_to_rot(q.x, q.y, q.z, q.w)
        if not np.all(np.isfinite(R)) or np.linalg.norm(R[2, :2]) > 0.05:
            return None  # the 2-D map needs a horizontal scan plane
        return (tr.translation.x, tr.translation.y, quat_to_yaw(q))

    def _reject(self, why: str):
        self.rejected[why] += 1
        self.get_logger().debug("scan rejected: " + why)

    def _on_scan(self, m: LaserScan):
        g = lambda n: self.get_parameter(n).value
        t_scan = stamp_to_sec(m.header.stamp)
        if not math.isfinite(t_scan) or t_scan <= 0.0:
            return self._reject("unstamped")
        if t_scan <= (self.last_accept or 0):
            return self._reject("stale")
        if not len(self.hist):
            return self._reject("no_pose")
        if t_scan > self._now() + 0.02:
            return self._reject("future")              # pose for that time has not arrived (or clocks differ)
        if max(self.hist.newest - t_scan, self._now() - t_scan) > g("max_scan_latency"):
            return self._reject("stale")
        if t_scan > self.hist.newest:
            self.waiting_scans = (self.waiting_scans + [m])[-4:]
            return  # wait for bracketing odometry, never extrapolate camera priors
        prior = self.hist.at(t_scan, future_tol=0.0)
        if prior is None:
            return self._reject("no_pose")
        mount = self._scan_to_base(m.header.frame_id, m.header.stamp)
        if mount is None:
            self.get_logger().warn("scan frame %r not resolvable to %s; rejected" % (m.header.frame_id, g("base_frame")),
                                   throttle_duration_sec=5.0)
            return self._reject("frame")
        r = np.asarray(m.ranges, float)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r > m.range_min) & (r < m.range_max)
        if ok.sum() < g("min_points"):
            return self._reject("few_points")
        mx, my, myaw = mount
        aa = a[ok] + myaw
        pts = np.stack([r[ok] * np.cos(aa) + mx, r[ok] * np.sin(aa) + my], 1)
        prior3 = np.array([prior[0], prior[1], prior[3]])
        res = self.matcher.match(pts, prior3)
        latency = max(0.0, self._now() - t_scan)
        h = Float32MultiArray()
        h.data = [float(res.ok), float(res.inlier_ratio), float(min(res.residual_rms, 9.9)), float(res.min_eig),
                  float(latency)]
        self.pub_h.publish(h)
        if not res.ok:
            return self._reject("match")
        if not np.all(np.isfinite(res.pose)) or not np.isfinite(res.residual_rms):
            return self._reject("match")
        d = res.pose - prior3
        d[2] = wrap(d[2])
        if self.last_accept is None:
            self.last_accept = t_scan
        allowed = g("max_correction") + g("gate_growth") * max(0.0, t_scan - self.last_accept)
        mag = float(np.linalg.norm(d[:2]))
        if mag > g("consistent_max_jump"):
            self.pending, self.recent = None, []
            return self._reject("gate")
        if mag > allowed or abs(d[2]) > g("max_yaw_correction"):
            self.pending = None
            # Consistency override: several consecutive HIGH-QUALITY matches that agree are strong evidence
            # even when the jump gate is too narrow (e.g. after the dark tube).
            good = (res.min_eig >= g("consistent_min_eig") and res.inlier_ratio >= g("consistent_min_inlier") and res.residual_rms <= g("consistent_max_rms")
                    and abs(d[2]) <= g("max_yaw_correction") and mag <= g("consistent_max_jump"))
            if not good:
                self.recent = []
                return self._reject("gate")
            self.recent = (self.recent + [d.copy()])[-int(g("consistent_n")):]
            R = np.array(self.recent)
            if len(R) < int(g("consistent_n")) or np.max(np.linalg.norm(R[:, :2] - R[:, :2].mean(0), axis=1)) > g("consistent_tol"):
                return self._reject("gate")
            d = R.mean(0)
            self.recent = []
            self.n_consistent += 1
            return self._publish(m, t_scan, prior3 + d, res)
        self.recent = []
        if mag > g("max_correction"):
            if self.pending is None or np.linalg.norm(self.pending - d[:2]) > g("confirm_tol"):
                self.pending = d[:2].copy()
                return self._reject("gate")
        self.pending = None
        return self._publish(m, t_scan, prior3 + d, res)

    def _publish(self, m: LaserScan, t_scan: float, pose3, res):
        self.last_accept = t_scan
        out = PoseWithCovarianceStamped()
        out.header.stamp = m.header.stamp
        out.header.frame_id = "course"
        out.pose.pose.position.x, out.pose.pose.position.y = float(pose3[0]), float(pose3[1])
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = math.sin(pose3[2] / 2), math.cos(pose3[2] / 2)
        var = float(min(1.0, (res.residual_rms ** 2) / max(res.min_eig, 1e-3)))
        cov = [0.0] * 36
        cov[0] = cov[7] = var
        cov[35] = var
        out.pose.covariance = cov
        self.pub.publish(out)
        self.n_published += 1


def main():
    rclpy.init()
    n = LocalizerNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
