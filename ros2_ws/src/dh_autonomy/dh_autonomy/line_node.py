"""Floor-guideline detector (mono camera) -> lateral offset of the line in the body frame.

The OAK-D S2 stereo cameras are monochrome, so the dark-red 2-inch guideline
shows up as a thin DARK stripe on a lighter floor. We threshold dark, thin,
elongated structures in the lower part of the image, project the pixels onto
the floor plane using the camera pitch and the current height, fit a line, and
publish
  /dh/line_obs  geometry_msgs/Vector3Stamped  x = lateral_m (left +), y = heading_rad, z = confidence
                header.stamp = image capture time (the mission evaluates the line against the pose AT that time)
The mission node turns this into a line-normal position correction while the
drone flies along a guideline segment of the course map.
"""
from __future__ import annotations

import math

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from dh_core.camera_geom import fit_line, rays_to_floor
from dh_core.health import stamp_to_sec

from .stereo_scan_node import img_to_gray


class LineNode(Node):
    def __init__(self):
        super().__init__("dh_line")
        P = self.declare_parameter
        P("image", "/oak/left/image_raw")
        P("info", "/oak/left/camera_info")
        P("camera_pitch_deg", 20.0)
        P("camera_x", 0.10)
        P("camera_z", -0.02)
        P("scale", 0.5)
        P("dark_delta", 25)            # black-hat response threshold: line this much darker than the floor beside it
        P("tophat_px", 0)              # black-hat kernel (px, at `scale`); 0 = auto from line width and geometry
        P("line_width_m", 0.051)       # 2 in
        P("min_inliers", 60)
        P("max_range", 2.5)
        g = lambda n: self.get_parameter(n).value
        self.K = None
        self.D = None
        self.height = 0.85
        self.create_subscription(CameraInfo, g("info"), self._on_info, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/dh/pose_map", lambda m: setattr(self, "height", max(0.2, m.pose.position.z)), 5)
        self.create_subscription(Image, g("image"), self._on_img, qos_profile_sensor_data)
        self.pub = self.create_publisher(Vector3Stamped, "/dh/line_obs", 5)

    def _on_info(self, m: CameraInfo):
        K = np.array(m.k, float)
        if K.size != 9 or K[0] <= 0 or K[4] <= 0:
            return                                   # uncalibrated camera_info: ignore
        self.K = K.reshape(3, 3)
        self.D = np.array(m.d, float)

    def _auto_kernel(self, s):
        """Black-hat kernel: ~2.2x the line's image width at the nearest visible floor point (odd, >= 7)."""
        g = lambda n: self.get_parameter(n).value
        f = self.K[1, 1] * s
        half_vfov = math.atan2(self.K[1, 2], self.K[1, 1])
        steep = min(math.radians(g("camera_pitch_deg")) + half_vfov, math.radians(85))
        near = (self.height + g("camera_z")) / math.sin(steep)        # slant range to the bottom image row
        k = int(math.ceil(2.2 * f * g("line_width_m") / max(near, 0.2)))
        return max(7, k | 1)

    def _on_img(self, m: Image):
        if self.K is None or stamp_to_sec(m.header.stamp) <= 0.0:
            return
        g = lambda n: self.get_parameter(n).value
        s = g("scale")
        try:
            im = img_to_gray(m)
        except ValueError as e:
            self.get_logger().warn("line: " + str(e), throttle_duration_sec=5.0)
            return
        im = cv2.resize(im, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
        h, w = im.shape
        roi = im[h // 2:, :]                       # floor is in the lower half when pitched down
        # Black-hat (closing - image) keeps only dark structures NARROWER than the kernel, i.e. a stripe
        # with floor on both sides. A dark wall's base is a step edge, which closing preserves -> ~0
        # response. (A plain "darker than the local mean" test fires on wall bases.)
        k = int(g("tophat_px")) or self._auto_kernel(s)
        bh = cv2.morphologyEx(roi, cv2.MORPH_BLACKHAT, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
        mask = (bh > g("dark_delta")).astype(np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n, lab, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        if n <= 1:
            return
        k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        vs, us = np.nonzero(lab == k)
        if len(us) < g("min_inliers"):
            return
        vs = vs + h // 2
        pts = np.stack([us / s, vs / s], 1).astype(np.float32).reshape(-1, 1, 2)
        und = cv2.undistortPoints(pts, self.K, self.D).reshape(-1, 2)   # normalised optical coords
        X, Y, down = rays_to_floor(und[:, 0], und[:, 1], math.radians(g("camera_pitch_deg")),
                                   self.height, g("camera_x"), g("camera_z"))
        X, Y = X[down], Y[down]
        near = np.hypot(X, Y) < g("max_range")
        X, Y = X[near], Y[near]
        if len(X) < g("min_inliers"):
            return
        lateral, heading, resid90 = fit_line(X, Y)
        width_ok = resid90 < 3 * g("line_width_m")
        conf = min(1.0, len(X) / 400.0) * (1.0 if width_ok else 0.3)
        out = Vector3Stamped()
        out.header.stamp = m.header.stamp
        out.header.frame_id = "base_link"
        out.vector.x, out.vector.y, out.vector.z = float(lateral), float(heading), float(conf)
        self.pub.publish(out)


def main():
    rclpy.init()
    n = LineNode()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
