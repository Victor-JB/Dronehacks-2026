"""Stereo pair -> depth -> horizontal-plane pseudo-scan (+ forward clearance).

The organizers' OAK-D pipeline publishes only raw left/right images (no depth),
because on-device depth would exceed the camera's ~143 MB/s XLink budget. This
node computes depth on the Jetson with OpenCV StereoSGBM on a downscaled,
rectified pair, keeps points near the drone's horizontal plane, and publishes:
  /dh/stereo_scan         sensor_msgs/LaserScan (frame base_link, x forward)
  /dh/forward_clearance   std_msgs/Float32 (min range within +-forward_half_angle)

Rectification (`rectify_source`, default auto), in order of preference:
  1. camera_info  - K, D, R, P from each camera_info when P_right carries a baseline;
  2. tf           - cv2.stereoRectify from K, D and the left->right optical-frame transform in TF
                    (the depthai driver publishes the factory extrinsics on /tf_static);
  3. baseline     - cv2.stereoRectify assuming a pure x-translation of `baseline` (OAK-D S2: 0.075 m).
"""
from __future__ import annotations

import math
import time

import cv2
import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Float32

from dh_core.camera_geom import (camera_info_has_stereo, disparity_to_body, points_to_scan, quat_to_rot,
                                 stereo_rectify)


def img_to_gray(msg: Image) -> np.ndarray:
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ("mono8", "8UC1"):
        return buf.reshape(msg.height, msg.step)[:, :msg.width]
    ch = 4 if msg.encoding in ("bgra8", "rgba8") else 3
    im = buf.reshape(msg.height, msg.step)[:, :msg.width * ch].reshape(msg.height, msg.width, ch)
    return cv2.cvtColor(im, cv2.COLOR_BGR2GRAY if msg.encoding.startswith("bgr") else cv2.COLOR_RGB2GRAY)


class StereoScanNode(Node):
    def __init__(self):
        super().__init__("dh_stereo_scan")
        P = self.declare_parameter
        P("left_image", "/oak/left/image_raw")
        P("right_image", "/oak/right/image_raw")
        P("left_info", "/oak/left/camera_info")
        P("right_info", "/oak/right/camera_info")
        P("scale", 0.5)                 # 1024x640 -> 512x320
        P("num_disparities", 64)
        P("block_size", 5)
        P("baseline", 0.0)              # >0 overrides the baseline from any source
        P("rectify_source", "auto")     # auto: camera_info if it has a baseline, else TF, else `baseline`/0.075
        P("tf_wait_s", 3.0)             # auto mode: how long to wait for the left->right TF before falling back
        P("camera_pitch_deg", 20.0)     # down-tilt of the camera (repo example ~20 deg)
        P("camera_x", 0.10)
        P("camera_z", -0.02)
        P("band_half_height", 0.15)     # keep points within +-this of the camera's horizontal plane
        P("n_beams", 81)
        P("max_range", 4.0)
        P("forward_half_angle_deg", 15.0)
        g = lambda n: self.get_parameter(n).value
        self.info = {}
        self.t_first_info = None
        self.create_subscription(CameraInfo, g("left_info"), lambda m: self._on_info("l", m), 1)
        self.create_subscription(CameraInfo, g("right_info"), lambda m: self._on_info("r", m), 1)
        try:  # TF is only needed when camera_info lacks stereo calibration
            from tf2_ros import Buffer, TransformListener
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        except ImportError:
            self.tf_buffer = None
        self.maps = None
        nd = int(g("num_disparities")) // 16 * 16
        bs = int(g("block_size"))
        self.sgbm = cv2.StereoSGBM_create(minDisparity=0, numDisparities=nd, blockSize=bs,
                                          P1=8 * bs * bs, P2=32 * bs * bs, disp12MaxDiff=1,
                                          uniquenessRatio=10, speckleWindowSize=100, speckleRange=2,
                                          mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        sync = ApproximateTimeSynchronizer([Subscriber(self, Image, g("left_image")),
                                            Subscriber(self, Image, g("right_image"))], 4, 0.01)
        sync.registerCallback(self._on_pair)
        self.pub_scan = self.create_publisher(LaserScan, "/dh/stereo_scan", 5)
        self.pub_clear = self.create_publisher(Float32, "/dh/forward_clearance", 5)
        self.busy = False

    def _extrinsics_from_tf(self):
        """(R, T) mapping left-optical points into the right-optical frame, from TF; None if unavailable."""
        if self.tf_buffer is None:
            return None
        src = self.info["l"].header.frame_id
        dst = self.info["r"].header.frame_id
        if not src or not dst or src == dst:
            return None
        try:
            from rclpy.time import Time
            tr = self.tf_buffer.lookup_transform(dst, src, Time()).transform
        except Exception as e:  # noqa: BLE001 - any tf2 failure means "no extrinsics"
            self.get_logger().warn("TF %s -> %s unavailable: %s" % (src, dst, e), throttle_duration_sec=2.0)
            return None
        q, t = tr.rotation, tr.translation
        return quat_to_rot(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z])

    def _rectification(self, size_full):
        """Pick the rectification source. Returns (R1, R2, P1, P2, B, source) at FULL resolution, or None."""
        g = lambda n: self.get_parameter(n).value
        mode = g("rectify_source")
        L, Rt = self.info["l"], self.info["r"]
        K1, D1 = np.array(L.k, float).reshape(3, 3), np.array(L.d, float)
        K2, D2 = np.array(Rt.k, float).reshape(3, 3), np.array(Rt.d, float)
        if mode in ("auto", "camera_info") and camera_info_has_stereo(L.r, Rt.p):
            P1 = np.array(L.p, float).reshape(3, 4); P2 = np.array(Rt.p, float).reshape(3, 4)
            B = -P2[0, 3] / P2[0, 0]
            return (np.array(L.r, float).reshape(3, 3), np.array(Rt.r, float).reshape(3, 3), P1, P2, B,
                    "camera_info")
        if mode == "camera_info":
            self.get_logger().warn("camera_info has no stereo baseline; waiting (set rectify_source:=auto)",
                                   throttle_duration_sec=5.0)
            return None
        if mode in ("auto", "tf"):
            ext = self._extrinsics_from_tf()
            if ext is not None:
                R1, R2, P1, P2, B = stereo_rectify(K1, D1, K2, D2, size_full, *ext)
                return R1, R2, P1, P2, B, "tf"
            if mode == "tf" or time.time() - self.t_first_info < g("tf_wait_s"):
                return None              # keep waiting for TF
        B = g("baseline") or 0.075
        self.get_logger().warn("no stereo calibration in camera_info or TF; assuming a pure %.3f m x-baseline "
                               "(OAK-D S2). Depth will be biased if the pair is not factory-aligned." % B)
        R1, R2, P1, P2, B = stereo_rectify(K1, D1, K2, D2, size_full, np.eye(3), np.array([-B, 0.0, 0.0]))
        return R1, R2, P1, P2, B, "baseline"

    def _build_maps(self, w, h):
        g = lambda n: self.get_parameter(n).value
        s = g("scale")
        size = (int(w * s), int(h * s))
        rect = self._rectification((w, h))
        if rect is None:
            return False
        R1, R2, P1, P2, B, source = rect
        if g("baseline"):
            B = g("baseline")            # explicit override wins
        maps = []
        for key, Rr, Pm in (("l", R1, P1), ("r", R2, P2)):
            ci = self.info[key]
            K = np.array(ci.k, float).reshape(3, 3); K[:2, :] *= s
            Ps = np.array(Pm, float).reshape(3, 4).copy(); Ps[:2, :] *= s
            maps.append(cv2.initUndistortRectifyMap(K, np.array(ci.d, float), Rr, Ps[:, :3], size, cv2.CV_32FC1))
        Ps = np.array(P2, float).reshape(3, 4) * np.array([[s], [s], [1.0]])
        f, cx, cy = Ps[0, 0], Ps[0, 2], Ps[1, 2]
        if not (0.03 < B < 0.3):
            self.get_logger().error("implausible stereo baseline %.3f m from %s; using 0.075" % (B, source))
            B = 0.075
        self.maps, self.f, self.B, self.cx, self.cy = maps, f, B, cx, cy
        self.R1 = np.asarray(R1, float).reshape(3, 3)
        self.get_logger().info("rectification ready (%s): f=%.1f px B=%.3f m size=%s" % (source, f, B, size))
        return True

    def _on_info(self, key, m: CameraInfo):
        self.info[key] = m
        if self.t_first_info is None:
            self.t_first_info = time.time()

    def _on_pair(self, ml: Image, mr: Image):
        if self.busy or "l" not in self.info or "r" not in self.info:
            return
        self.busy = True
        try:
            self._process(ml, mr)
        finally:
            self.busy = False

    def _process(self, ml: Image, mr: Image):
        g = lambda n: self.get_parameter(n).value
        if self.maps is None and not self._build_maps(ml.width, ml.height):
            return
        s = g("scale")
        L = img_to_gray(ml); R = img_to_gray(mr)
        L = cv2.resize(L, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
        R = cv2.resize(R, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
        (l1, l2), (r1, r2) = self.maps
        Lr = cv2.remap(L, l1, l2, cv2.INTER_LINEAR)
        Rr = cv2.remap(R, r1, r2, cv2.INTER_LINEAR)
        disp = self.sgbm.compute(Lr, Rr).astype(np.float32) / 16.0
        xb, yb, zb = disparity_to_body(disp, self.f, self.B, self.cx, self.cy, math.radians(g("camera_pitch_deg")),
                                       g("camera_x"), g("camera_z"), R_rect=self.R1)
        n = int(g("n_beams"))
        fov = 2 * math.atan2(self.cx, self.f)
        ranges, amin, inc = points_to_scan(xb, yb, zb, g("camera_z"), g("band_half_height"), fov, n, g("max_range"))
        scan = LaserScan()
        scan.header = ml.header
        scan.header.frame_id = "base_link"
        scan.angle_min, scan.angle_max, scan.angle_increment = amin, amin + inc * (n - 1), inc
        scan.range_min, scan.range_max = 0.2, float(g("max_range"))
        scan.ranges = [float(x) for x in ranges]
        self.pub_scan.publish(scan)
        half = math.radians(g("forward_half_angle_deg"))
        a = amin + np.arange(n) * inc
        fwd = ranges[np.abs(a) <= half]
        self.pub_clear.publish(Float32(data=float(np.min(fwd)) if np.isfinite(fwd).any() else math.inf))


def main():
    rclpy.init()
    n = StereoScanNode()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
