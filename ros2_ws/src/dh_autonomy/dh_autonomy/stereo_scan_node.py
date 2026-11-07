"""Stereo pair -> depth -> horizontal-plane pseudo-scan (+ forward clearance).

The organizers' OAK-D pipeline publishes only raw left/right images (no depth),
because on-device depth would exceed the camera's ~143 MB/s XLink budget. This
node computes depth on the Jetson with OpenCV StereoSGBM on a downscaled,
rectified pair, keeps points near the drone's horizontal plane, and publishes:
  /dh/stereo_scan         sensor_msgs/LaserScan (frame base_link, x forward; stamp = image capture time)
                          REP 117: finite = return, +inf = stereo data but nothing within range_max,
                          NaN = no stereo data in that beam (textureless / too dark / closer than min depth)
  /dh/forward_clearance   sensor_msgs/Range (stamp = image capture time, frame base_link)
                          finite = nearest obstacle within +-forward_half_angle;
                          +inf  = VALID measurement, nothing within max_range (enough forward coverage);
                          NaN   = cannot vouch for a clear path (too little stereo data / too dark / no calibration)
  The mission treats a missing or stale Range as "unknown", never as clear.

Inputs: /oak/{left,right}/image_raw (the organizers' driver ships bgr8 on the mono sockets; mono8,
mono16, rgb8, bgra8, rgba8 also accepted) + camera_info, subscribed with sensor-data (best-effort) QoS,
paired by capture stamps (OAK stereo is hardware-synchronised).
Minimum measurable depth = f*B/num_disparities (~0.33 m at the defaults): anything closer gives no
disparity and therefore reads NaN, i.e. "unknown", not clear.

Rectification (`rectify_source`, default auto), in order of preference:
  1. camera_info  - K, D, R, P from each camera_info when P_right carries a baseline;
  2. tf           - cv2.stereoRectify from K, D and the left->right optical-frame transform in TF
                    (the depthai driver publishes the factory extrinsics on /tf_static);
  3. baseline     - explicit offline opt-in only; auto mode stays unknown without calibration.
"""
from __future__ import annotations

import math

import cv2
import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan, Range
from std_msgs.msg import Header

from dh_core.camera_geom import (camera_info_has_stereo, disparity_to_body, points_to_scan, quat_to_rot,
                                 scan_validity, stereo_rectify)
from dh_core.health import stamp_to_sec

ENCODINGS = ("mono8", "8UC1", "mono16", "16UC1", "bgr8", "rgb8", "bgra8", "rgba8")


def img_to_gray(msg: Image) -> np.ndarray:
    """sensor_msgs/Image -> uint8 grayscale. Raises ValueError on an unsupported or inconsistent image."""
    enc = msg.encoding
    channels = 4 if enc in ("rgba8", "bgra8") else 3 if enc in ("rgb8", "bgr8") else 1
    byte_width = 2 if enc in ("mono16", "16UC1") else 1
    if (msg.width <= 0 or msg.height <= 0 or msg.step < msg.width * channels * byte_width
            or msg.step % byte_width):
        raise ValueError("invalid image dimensions or row stride")
    if enc not in ENCODINGS:
        raise ValueError("unsupported image encoding " + repr(enc))
    if enc in ("mono16", "16UC1"):
        dt = np.dtype(">u2") if getattr(msg, "is_bigendian", 0) else np.dtype("<u2")
        buf = np.frombuffer(msg.data, dtype=dt)
        if buf.size * 2 < msg.height * msg.step:
            raise ValueError("image buffer smaller than height*step")
        im16 = buf[:msg.height * msg.step // 2].reshape(msg.height, msg.step // 2)[:, :msg.width]
        return (im16 >> 8).astype(np.uint8)
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if buf.size < msg.height * msg.step:
        raise ValueError("image buffer smaller than height*step")
    if enc in ("mono8", "8UC1"):
        return buf[:msg.height * msg.step].reshape(msg.height, msg.step)[:, :msg.width]
    ch = 4 if enc in ("bgra8", "rgba8") else 3
    im = buf[:msg.height * msg.step].reshape(msg.height, msg.step)[:, :msg.width * ch].reshape(msg.height, msg.width, ch)
    code = {"bgr8": cv2.COLOR_BGR2GRAY, "rgb8": cv2.COLOR_RGB2GRAY,
            "bgra8": cv2.COLOR_BGRA2GRAY, "rgba8": cv2.COLOR_RGBA2GRAY}[enc]
    return cv2.cvtColor(np.ascontiguousarray(im), code)


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
        P("allow_assumed_baseline", False)  # opt-in for offline comparison only
        P("max_image_age", 0.5)
        P("tf_wait_s", 3.0)             # auto mode: how long to wait for the left->right TF before falling back
        P("camera_pitch_deg", 20.0)     # down-tilt of the camera (repo example ~20 deg)
        P("camera_x", 0.10)
        P("camera_z", -0.02)
        P("band_half_height", 0.15)     # keep points within +-this of the camera's horizontal plane
        P("n_beams", 81)
        P("max_range", 4.0)
        P("forward_half_angle_deg", 15.0)
        P("min_points_per_beam", 5)      # stereo points a beam needs to count as measured
        P("forward_min_coverage", 0.6)   # fraction of forward beams that must be measured to report "clear"
        P("dark_mean_level", 12.0)       # mean left-image grey level below which stereo is declared blind
        P("max_pair_dt", 0.005)          # s; left/right stamps must agree to this (OAK is hardware-synced)
        P("base_frame", "base_link")
        P("mount_source", "tf")  # tf (measured organizer mount) | parameters (offline fixtures)
        P("scan_percentile", 50.0)       # per-beam statistic for /dh/stereo_scan (localization): median
        g = lambda n: self.get_parameter(n).value
        self.info = {}
        self.t_first_info = None
        self.create_subscription(CameraInfo, g("left_info"), lambda m: self._on_info("l", m), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, g("right_info"), lambda m: self._on_info("r", m), qos_profile_sensor_data)
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
        sync = ApproximateTimeSynchronizer(
            [Subscriber(self, Image, g("left_image"), qos_profile=qos_profile_sensor_data),
             Subscriber(self, Image, g("right_image"), qos_profile=qos_profile_sensor_data)], 4, g("max_pair_dt"))
        sync.registerCallback(self._on_pair)
        self.pub_scan = self.create_publisher(LaserScan, "/dh/stereo_scan", 5)
        self.pub_clear = self.create_publisher(Range, "/dh/forward_clearance", 5)
        self.busy = False
        self.map_size = None
        self.stats = {"pairs": 0, "rejected": 0, "dark": 0, "invalid_fwd": 0}

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
            if mode == "tf" or self._now() - self.t_first_info < g("tf_wait_s"):
                return None              # keep waiting for TF
        if not g("allow_assumed_baseline") and mode != "baseline":
            return None
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
            self.get_logger().error("implausible stereo baseline %.3f m from %s; rejecting" % (B, source))
            return False
        self.maps, self.f, self.B, self.cx, self.cy = maps, f, B, cx, cy
        self.map_size = (w, h)
        self.R1 = np.asarray(R1, float).reshape(3, 3)
        self.get_logger().info("rectification ready (%s): f=%.1f px B=%.3f m size=%s" % (source, f, B, size))
        return True

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_info(self, key, m: CameraInfo):
        K = np.array(m.k, float)
        arrays = [np.asarray(v, float) for v in (m.k, m.d, m.r, m.p)]
        if (K.size != 9 or len(m.r) != 9 or len(m.p) != 12 or K[0] <= 0 or K[4] <= 0
                or not all(np.all(np.isfinite(v)) for v in arrays) or not m.header.frame_id):
            self.info.pop(key, None)
            self.maps = None
            self.get_logger().warn("invalid camera_info: clearance unavailable", throttle_duration_sec=5.0)
            return
        old = self.info.get(key)
        if old is not None and any(getattr(old, f) != getattr(m, f) for f in ("k", "d", "r", "p", "width", "height")):
            self.maps = None
        self.info[key] = m
        if self.t_first_info is None:
            self.t_first_info = self._now()

    def _publish_invalid(self, header, reason: str):
        """No trustworthy depth this frame: forward clearance NaN (unknown), no scan."""
        self._publish_clearance(header, math.nan)
        self.get_logger().warn("stereo invalid: " + reason, throttle_duration_sec=2.0)

    def _publish_clearance(self, header, rng: float):
        g = lambda n: self.get_parameter(n).value
        r = Range()
        r.header = header
        r.radiation_type = Range.INFRARED    # closest enum; this is passive stereo (documented above)
        r.field_of_view = float(2 * math.radians(g("forward_half_angle_deg")))
        r.min_range = 0.2
        r.max_range = float(g("max_range"))
        r.range = float(rng)
        self.pub_clear.publish(r)

    def _on_pair(self, ml: Image, mr: Image):
        if self.busy:
            return
        self.busy = True
        try:
            try:
                self._process(ml, mr)
            except (cv2.error, ValueError, TypeError) as e:
                header = Header()
                header.stamp, header.frame_id = ml.header.stamp, self.get_parameter("base_frame").value
                self.stats["rejected"] += 1
                self._publish_invalid(header, str(e))
        finally:
            self.busy = False

    def _process(self, ml: Image, mr: Image):
        g = lambda n: self.get_parameter(n).value
        self.stats["pairs"] += 1
        header = Header()
        header.stamp = ml.header.stamp                 # capture time of the left image (copied, not aliased)
        header.frame_id = g("base_frame")
        tl, tr_ = stamp_to_sec(ml.header.stamp), stamp_to_sec(mr.header.stamp)
        if not math.isfinite(tl) or tl <= 0.0:
            self.stats["rejected"] += 1
            self.get_logger().warn("image without timestamp; frame dropped", throttle_duration_sec=2.0)
            return
        if (not math.isfinite(tr_) or tr_ <= 0 or tl > self._now() + 0.02
                or self._now() - tl > g("max_image_age")
                or abs(tl - tr_) > g("max_pair_dt")):
            self.stats["rejected"] += 1
            self._publish_invalid(header, "left/right stamps differ by %.1f ms" % (1e3 * abs(tl - tr_)))
            return
        if "l" not in self.info or "r" not in self.info:
            self._publish_invalid(header, "waiting for camera_info")
            return
        if (ml.width, ml.height) != (mr.width, mr.height):
            self._publish_invalid(header, "left/right image sizes differ")
            return
        for key, image in (("l", ml), ("r", mr)):
            ci = self.info[key]
            if ((ci.width, ci.height) != (image.width, image.height)
                    or image.header.frame_id != ci.header.frame_id):
                self._publish_invalid(header, "image dimensions/frame differ from camera_info")
                return
        if self.maps is not None and self.map_size != (ml.width, ml.height):
            self.get_logger().warn("image size changed; rebuilding rectification")
            self.maps = None
        if self.maps is None and not self._build_maps(ml.width, ml.height):
            self._publish_invalid(header, "rectification not ready")
            return
        s = g("scale")
        try:
            L = img_to_gray(ml); R = img_to_gray(mr)
        except ValueError as e:
            self.stats["rejected"] += 1
            self._publish_invalid(header, str(e))
            return
        if min(float(L.mean()), float(R.mean())) < g("dark_mean_level"):
            self.stats["dark"] += 1
            self._publish_invalid(header, "image too dark for stereo (mean %.1f)" % float(L.mean()))
            return
        L = cv2.resize(L, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
        R = cv2.resize(R, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
        (l1, l2), (r1, r2) = self.maps
        Lr = cv2.remap(L, l1, l2, cv2.INTER_LINEAR)
        Rr = cv2.remap(R, r1, r2, cv2.INTER_LINEAR)
        disp = self.sgbm.compute(Lr, Rr).astype(np.float32) / 16.0
        mount = None
        band_z = g("camera_z")
        if g("mount_source") == "tf":
            try:
                from rclpy.time import Time
                tr = self.tf_buffer.lookup_transform(g("base_frame"), ml.header.frame_id,
                                                     Time.from_msg(ml.header.stamp)).transform
                q, t = tr.rotation, tr.translation
                vals = np.array([q.x, q.y, q.z, q.w, t.x, t.y, t.z])
                if not np.all(np.isfinite(vals)) or abs(np.linalg.norm(vals[:4]) - 1.0) > 0.01:
                    raise ValueError("invalid optical mount TF")
                mount = (quat_to_rot(q.x, q.y, q.z, q.w), vals[4:])
                band_z = t.z
            except Exception as e:
                self._publish_invalid(header, "optical-to-body TF unavailable: " + str(e))
                return
        elif g("mount_source") != "parameters":
            self._publish_invalid(header, "invalid mount_source")
            return
        xb, yb, zb = disparity_to_body(disp, self.f, self.B, self.cx, self.cy, math.radians(g("camera_pitch_deg")),
                                       g("camera_x"), g("camera_z"), R_rect=self.R1, body_transform=mount)
        n = int(g("n_beams"))
        fov = 2 * math.atan2(self.cx, self.f)
        mp = int(g("min_points_per_beam"))
        # Two statistics per beam from the same points: the localizer gets an UNBIASED range (median), the
        # obstacle check a conservative one (10th percentile = nearest robust return).
        near, amin, inc, counts = points_to_scan(xb, yb, zb, band_z, g("band_half_height"), fov, n,
                                                 g("max_range"), min_pts=mp, pct=10.0, return_counts=True)
        mid, _, _ = points_to_scan(xb, yb, zb, band_z, g("band_half_height"), fov, n,
                                   g("max_range"), min_pts=mp, pct=g("scan_percentile"))
        a = amin + np.arange(n) * inc
        _, fwd = scan_validity(near, counts, a, math.radians(g("forward_half_angle_deg")), mp,
                               g("forward_min_coverage"))
        ranges, _ = scan_validity(mid, counts, a, math.radians(g("forward_half_angle_deg")), mp,
                                  g("forward_min_coverage"))
        scan = LaserScan()
        scan.header = header
        scan.angle_min, scan.angle_max, scan.angle_increment = amin, amin + inc * (n - 1), inc
        scan.range_min, scan.range_max = 0.2, float(g("max_range"))
        scan.ranges = [float(x) for x in ranges]
        self.pub_scan.publish(scan)
        if math.isnan(fwd):
            self.stats["invalid_fwd"] += 1
        self._publish_clearance(header, fwd)


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
