#!/usr/bin/env python3
"""dronehacks_nav.camera_web - the camera and the depth image as MJPEG on a plain web page, served by
the Jetson itself. Read-only: subscribes to two image topics, publishes nothing.

    http://skybeta-02.local:8080/            both streams side by side
    http://skybeta-02.local:8080/camera.mjpg
    http://skybeta-02.local:8080/depth.mjpg

Frames are re-encoded at most max_fps per stream, and only while someone is watching, so an idle
page costs nothing but the subscriptions. Depth (16UC1 millimetres) is coloured with TURBO from
0 to depth_max_m; pixels with no return are black.

Run:  ros2 run dronehacks_nav camera_web --ros-args -p port:=8080
"""
from __future__ import annotations

import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

PAGE = b"""<!doctype html><html><head><meta charset="utf-8"><title>skybeta camera</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>body{margin:0;background:#111;color:#ddd;font:14px sans-serif}
div{display:flex;flex-wrap:wrap;gap:8px;padding:8px}figure{margin:0;flex:1 1 400px}
img{width:100%;image-rendering:pixelated;background:#000}</style></head><body><div>
<figure><img src="/camera.mjpg"><figcaption>camera (left)</figcaption></figure>
<figure><img src="/depth.mjpg"><figcaption>depth (black = no return)</figcaption></figure>
</div></body></html>"""


def to_array(m):
    """sensor_msgs/Image -> numpy (honours the row step). mono8 / bgr8 / rgb8 / 16UC1."""
    enc = m.encoding.lower()
    if enc in ("16uc1", "mono16"):
        return np.frombuffer(m.data, np.uint16).reshape(m.height, m.step // 2)[:, :m.width]
    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)
    if enc in ("mono8", "8uc1"):
        return a[:, :m.width]
    if enc in ("bgr8", "rgb8"):
        a = a[:, :m.width * 3].reshape(m.height, m.width, 3)
        return a if enc == "bgr8" else a[:, :, ::-1]
    raise ValueError(f"unsupported encoding {m.encoding!r}")


class Stream:
    """The latest JPEG of one topic, and a condition the HTTP threads wait on."""

    def __init__(self):
        self.cond = threading.Condition()
        self.jpeg = None
        self.seq = 0
        self.viewers = 0
        self.t_last = 0.0

    def put(self, jpeg):
        with self.cond:
            self.jpeg, self.seq = jpeg, self.seq + 1
            self.cond.notify_all()


class CameraWeb(Node):
    def __init__(self):
        super().__init__("camera_web")
        self.port = int(self.declare_parameter("port", 8080).value)
        self.address = self.declare_parameter("address", "0.0.0.0").value
        camera_topic = self.declare_parameter("camera_topic", "/oak/left/image_mono").value
        depth_topic = self.declare_parameter("depth_topic", "/oak/stereo/image_raw").value
        self.max_fps = float(self.declare_parameter("max_fps", 10.0).value)
        self.depth_max_mm = 1000.0 * float(self.declare_parameter("depth_max_m", 4.0).value)
        self.quality = int(self.declare_parameter("jpeg_quality", 70).value)
        self.streams = {"camera": Stream(), "depth": Stream()}
        self.create_subscription(Image, camera_topic, lambda m: self._on_image("camera", m), qos_profile_sensor_data)
        self.create_subscription(Image, depth_topic, lambda m: self._on_image("depth", m), qos_profile_sensor_data)
        self.get_logger().info(f"camera {camera_topic}, depth {depth_topic}: http://{self.address}:{self.port}/")

    def _on_image(self, name, m):
        s = self.streams[name]
        now = self.get_clock().now().nanoseconds * 1e-9
        if s.viewers == 0 or now - s.t_last < 1.0 / self.max_fps:
            return
        s.t_last = now
        try:
            a = to_array(m)
        except ValueError as e:
            self.get_logger().warning(f"{name}: {e}", throttle_duration_sec=10.0)
            return
        if name == "depth":
            scaled = np.clip(a.astype(np.float32) * (255.0 / self.depth_max_mm), 0, 255).astype(np.uint8)
            colour = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
            colour[a == 0] = 0                      # no return: black, not "very near"
            a = colour
        ok, buf = cv2.imencode(".jpg", a, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if ok:
            s.put(buf.tobytes())

    def serve(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.end_headers()
                    self.wfile.write(PAGE)
                    return
                name = self.path.strip("/").removesuffix(".mjpg")
                if name not in node.streams:
                    self.send_error(404)
                    return
                s = node.streams[name]
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                with s.cond:
                    s.viewers += 1
                seq = s.seq
                try:
                    while rclpy.ok():
                        with s.cond:
                            s.cond.wait_for(lambda: s.seq != seq, timeout=2.0)
                            if s.seq == seq:
                                continue
                            seq, jpeg = s.seq, s.jpeg
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                         + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass
                finally:
                    with s.cond:
                        s.viewers -= 1

        server = ThreadingHTTPServer((self.address, self.port), Handler)
        server.daemon_threads = True
        threading.Thread(target=server.serve_forever, daemon=True).start()
        return server


def main(argv=None):
    rclpy.init(args=list(argv) if argv is not None else sys.argv)
    node = CameraWeb()
    server = node.serve()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
