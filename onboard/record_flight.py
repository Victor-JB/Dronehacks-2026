#!/usr/bin/env python3
"""record_flight - a pure OBSERVER node for a real FlyX Lab run.

Publishes nothing, issues no commands, makes no flight decisions. Subscribes to the same
real topics refnav.node already uses (PX4 local position/attitude/status, the depth camera +
its CameraInfo) plus refnav's own /refnav/status (phase/why/leg/counters), and, if present,
a raw stereo image topic, so a REAL run can be visualized afterward: actual trajectory, actual
mission decisions, actual depth-camera frames and (best effort) an actual camera-feed thumbnail.

Run this ALONGSIDE a real, team-approved mission in the FlyX Lab companion terminal (not on
this laptop -- there is no ROS/PX4/simulator here). It does not arm or fly anything by itself:

    # terminal 1: the actual mission, as usual
    python3 -m refnav.node --ros-args -p route_file:=route.json -p use_sim_time:=true

    # terminal 2, same container, while it flies:
    python3 onboard/record_flight.py --ros-args -p use_sim_time:=true -p out_dir:=recordings/run1

On shutdown (Ctrl+C once the mission is done, or whenever you want to stop watching) it writes
<out_dir>/manifest.json plus small depth/rgb frame thumbnails under <out_dir>/frames/. Bring that
folder back to this laptop (it's plain files) and build the visualization with:

    python3 tools/build_real_flight_viz.py recordings/run1/manifest.json diagrams/real-flight.html

Unverified end to end (no ROS/PX4 available on this laptop to test against) -- modeled directly
on refnav/node.py's own subscriptions/QoS/topics/decode helpers, which ARE already proven against
the real sim, so topic names/types/QoS here should match; the frame encoder and manifest format
are new and worth a sanity check on the first real run (watch its own log output).
"""
from __future__ import annotations

import json
import math
import sys
import time
import zlib
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from refnav.depth import decode_depth, quat_to_mat  # noqa: E402

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)


# ---------------------------------------------------------------- zero-dependency PNG writer
def _png_chunk(tag, data):
    return (len(data).to_bytes(4, "big") + tag + data
            + (zlib.crc32(tag + data) & 0xFFFFFFFF).to_bytes(4, "big"))


def write_png(path, arr_u8):
    """arr_u8: (H,W) uint8 grayscale or (H,W,3) uint8 RGB. No PIL/cv2 required -- the companion
    container's exact imaging deps aren't known in advance, so this only needs numpy+zlib."""
    h, w = arr_u8.shape[:2]
    color_type = 0 if arr_u8.ndim == 2 else 2
    channels = 1 if color_type == 0 else 3
    raw = bytearray()
    for y in range(h):
        raw.append(0)  # filter type 0 (none) per scanline
        raw.extend(arr_u8[y].astype(np.uint8).tobytes())
    ihdr = (w.to_bytes(4, "big") + h.to_bytes(4, "big") + bytes([8, color_type, 0, 0, 0]))
    sig = b"\x89PNG\r\n\x1a\n"
    data = (sig + _png_chunk(b"IHDR", ihdr)
            + _png_chunk(b"IDAT", zlib.compress(bytes(raw), 6))
            + _png_chunk(b"IEND", b""))
    Path(path).write_bytes(data)
    return channels


def depth_to_png_u8(depth_m, rmin=0.15, rmax=4.0):
    """Near=bright, far=dark, no-return=black -- a quick-look depth thumbnail, not a measurement."""
    d = np.asarray(depth_m, np.float32)
    valid = np.isfinite(d) & (d > 0)
    t = np.clip((d - rmin) / (rmax - rmin), 0.0, 1.0)
    img = np.where(valid, (255.0 * (1.0 - t)), 0.0)
    return img.astype(np.uint8)


def image_msg_to_u8(m):
    """Best-effort sensor_msgs/Image -> uint8 array (gray 2D or RGB 3D) for a camera-feed thumbnail."""
    enc = m.encoding.lower()
    buf = bytes(m.data)
    if enc in ("mono8", "8uc1"):
        return np.frombuffer(buf, np.uint8).reshape(m.height, m.step)[:, :m.width]
    if enc == "rgb8":
        return np.frombuffer(buf, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
    if enc == "bgr8":
        a = np.frombuffer(buf, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
        return a[:, :, ::-1]
    raise ValueError(f"unsupported image encoding for a quick thumbnail: {m.encoding!r}")


def downsample(arr, max_w=160):
    h, w = arr.shape[:2]
    if w <= max_w:
        return arr
    stride = max(1, w // max_w)
    return arr[::stride, ::stride]


class FlightRecorder(Node):
    def __init__(self):
        super().__init__("flight_recorder")
        self.depth_topic = self.declare_parameter("depth_topic", "/camera/depth/image_raw").value
        info = self.declare_parameter("depth_info_topic", "").value
        self.info_topic = info or self.depth_topic.rsplit("/", 1)[0] + "/camera_info"
        self.rgb_topic = self.declare_parameter("rgb_topic", "/stereo/left/image").value
        self.status_topic = self.declare_parameter("status_topic", "/refnav/status").value
        self.frame_hz = float(self.declare_parameter("frame_hz", 2.0).value)
        out_dir = self.declare_parameter("out_dir", f"recordings/{int(time.time())}").value
        self.out_dir = Path(out_dir)
        (self.out_dir / "frames").mkdir(parents=True, exist_ok=True)

        from px4_msgs.msg import VehicleAttitude, VehicleLandDetected, VehicleLocalPosition, VehicleStatus
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self._on_lpos, PX4_QOS)
        self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude", self._on_att, PX4_QOS)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", self._on_status, PX4_QOS)
        self.create_subscription(VehicleLandDetected, "/fmu/out/vehicle_land_detected", self._on_land, PX4_QOS)
        self.create_subscription(CameraInfo, self.info_topic, self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(Image, self.rgb_topic, self._on_rgb, qos_profile_sensor_data)
        self.create_subscription(String, self.status_topic, self._on_mission_status, 10)

        self.K = None
        self.q = None  # (w,x,y,z), PX4 attitude
        self.t0 = None
        self.trajectory = []     # {t, n, e, d, heading, armed, nav_state, landed}
        self.status_log = []     # {t, status-json}
        self.depth_frames = []   # {t, path, w, h}
        self.rgb_frames = []     # {t, path, w, h}
        self._last_v = dict(armed=False, nav_state=None, landed=None, heading=None)
        self._last_depth_save = 0.0
        self._last_rgb_save = 0.0
        self.get_logger().info(f"recording to {self.out_dir} (observer only, publishes nothing); "
                               f"depth={self.depth_topic} rgb={self.rgb_topic} status={self.status_topic}")

    def _now(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = t
        return t - self.t0

    def _on_lpos(self, m):
        self._last_v["heading"] = m.heading
        self.trajectory.append(dict(t=round(self._now(), 3), n=m.x, e=m.y, d=m.z, heading=m.heading,
                                    armed=self._last_v["armed"], nav_state=self._last_v["nav_state"],
                                    landed=self._last_v["landed"]))

    def _on_att(self, m):
        self.q = tuple(float(c) for c in m.q)

    def _on_status(self, m):
        self._last_v["armed"] = (m.arming_state == 2)
        self._last_v["nav_state"] = int(m.nav_state)

    def _on_land(self, m):
        self._last_v["landed"] = bool(m.landed)

    def _on_info(self, m):
        if m.k[0] > 0 and m.k[4] > 0:
            self.K = (m.k[0], m.k[4], m.k[2], m.k[5])

    def _on_mission_status(self, m):
        try:
            st = json.loads(m.data)
        except ValueError:
            return
        self.status_log.append(dict(t=round(self._now(), 3), status=st))

    def _on_depth(self, m):
        t = self._now()
        if t - self._last_depth_save < 1.0 / self.frame_hz:
            return
        try:
            d = decode_depth(m.encoding, m.height, m.width, m.step, bytes(m.data))
            thumb = downsample(depth_to_png_u8(d))
            idx = len(self.depth_frames)
            rel = f"frames/depth_{idx:05d}.png"
            write_png(self.out_dir / rel, thumb)
            finite = d[np.isfinite(d) & (d > 0)]
            min_depth_m = float(finite.min()) if finite.size else None
            self.depth_frames.append(dict(t=round(t, 3), path=rel, w=int(thumb.shape[1]), h=int(thumb.shape[0]),
                                          min_depth_m=min_depth_m))
            self._last_depth_save = t
        except Exception as e:  # a recorder must never crash a real run it's observing
            self.get_logger().warning(f"depth frame not saved: {e!r}")

    def _on_rgb(self, m):
        t = self._now()
        if t - self._last_rgb_save < 1.0 / self.frame_hz:
            return
        try:
            img = downsample(image_msg_to_u8(m))
            idx = len(self.rgb_frames)
            rel = f"frames/rgb_{idx:05d}.png"
            write_png(self.out_dir / rel, img)
            self.rgb_frames.append(dict(t=round(t, 3), path=rel, w=int(img.shape[1]), h=int(img.shape[0])))
            self._last_rgb_save = t
        except Exception as e:
            self.get_logger().warning(f"rgb frame not saved: {e!r}")

    def flush(self):
        manifest = dict(
            recorded_at=time.strftime("%Y-%m-%dT%H:%M:%S"),
            camera_K=self.K, depth_topic=self.depth_topic, rgb_topic=self.rgb_topic,
            trajectory=self.trajectory, status_log=self.status_log,
            depth_frames=self.depth_frames, rgb_frames=self.rgb_frames,
        )
        (self.out_dir / "manifest.json").write_text(json.dumps(manifest))
        self.get_logger().info(f"wrote {self.out_dir / 'manifest.json'}: "
                               f"{len(self.trajectory)} pose samples, {len(self.status_log)} status updates, "
                               f"{len(self.depth_frames)} depth frames, {len(self.rgb_frames)} rgb frames")


def main(argv=None):
    rclpy.init(args=list(argv) if argv is not None else sys.argv)
    node = FlightRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.flush()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
