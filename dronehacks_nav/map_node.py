#!/usr/bin/env python3
"""dronehacks_nav.map_node - builds dronehacks_nav.mapping.VoxelMap from the OAK depth and cuVSLAM odometry.
Read-only towards the flight stack: it subscribes, and publishes only map products.

  in   depth_topic (16UC1 mm or 32FC1 m) + its camera_info, odometry_topic (base_link in odom, ENU/FLU),
       TF base_link <- the depth frame (static, from the stack's URDF)
       status_topic (/refnav/status, the mission's JSON): its planned route and current setpoint
  out  /map/occupied   sensor_msgs/PointCloud2, occupied voxel centres in odom (Foxglove 3D panel)
       /map/image      sensor_msgs/Image bgr8, top-down slice of the slice_z band with the drone, its
                       trail, the planned route (blue) and the setpoint (pink) drawn over it
                       (camera_web shows it as the "map" stream)

Compute budget: depth arrives at 30 Hz; integrate_hz (2) times a second ONE frame is integrated with
every stride-th pixel (8: ~1000 rays at 320x200). Products are published at publish_hz (1).

Timing: cuVSLAM's odometry for an instant arrives ~0.5 s after the OAK's depth frame for the same
instant, so the newest depth frame never has its pose yet. The node keeps ~2 s of depth frames and
integrates the newest one that the odometry has caught up with (a pose within max_pose_dt_s of its
stamp); older frames are dropped unused.

Run:  ros2 run dronehacks_nav map_node
"""
from __future__ import annotations

import json
import math
import sys
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String

from refnav import depth as D

from . import frames as F
from . import mapping as M

COLOURS = {M.UNKNOWN: (45, 45, 45), M.FREE: (205, 205, 205), M.OCCUPIED: (40, 40, 230)}   # BGR


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class MapNode(Node):
    def __init__(self):
        super().__init__("dronehacks_map")
        p = lambda k, v: self.declare_parameter(k, v).value
        self.map = M.VoxelMap(voxel_m=float(p("voxel_m", 0.1)), size_xy_m=float(p("size_xy_m", 20.0)),
                              z_min_m=float(p("z_min_m", -1.0)), z_max_m=float(p("z_max_m", 3.0)))
        depth_topic = p("depth_topic", "/oak/stereo/image_raw")
        info_topic = p("depth_info_topic", "/oak/stereo/camera_info")
        odom_topic = p("odometry_topic", "/visual_slam/tracking/odometry")
        self.base_frame = p("base_frame", "base_link")
        self.frame = p("map_frame", "odom")
        self.stride = int(p("stride", 8))
        self.max_range = float(p("max_range_m", 4.0))
        self.max_pose_dt = float(p("max_pose_dt_s", 0.05))
        self.z_band = (float(p("slice_z_lo_m", 0.3)), float(p("slice_z_hi_m", 2.0)))
        self.scale = int(p("image_scale", 3))
        integrate_hz, publish_hz = float(p("integrate_hz", 2.0)), float(p("publish_hz", 1.0))

        from tf2_ros import Buffer, TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.K = None
        self.extrinsic = None           # (frame_id, R_body_opt, t_body_opt)
        self.depths = deque(maxlen=60)  # ~2 s of depth msgs, waiting for their odometry to arrive
        self.poses = deque(maxlen=90)   # (stamp, p_world (3,), R_world_body (3,3)) ~3 s of odometry
        self.trail = deque(maxlen=600)  # (x, y) every integrated frame
        self.skipped = {}
        self.plan = None                # (route [(x, y)] ENU from home, setpoint (x, y) or None)

        self.create_subscription(CameraInfo, info_topic, self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, depth_topic, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(String, p("status_topic", "/refnav/status"), self._on_status, 10)
        self.pub_cloud = self.create_publisher(PointCloud2, "/map/occupied", 1)
        self.pub_image = self.create_publisher(Image, "/map/image", 1)
        self.create_timer(1.0 / integrate_hz, self._integrate)
        self.create_timer(1.0 / publish_hz, self._publish)
        self.get_logger().info(f"mapping {depth_topic} with {odom_topic}: {self.map.shape} voxels of "
                               f"{self.map.voxel:.2f} m, {integrate_hz:g} Hz, stride {self.stride}")

    # ------------------------------------------------------------------ inputs
    def _on_info(self, m):
        if m.k[0] > 0 and m.k[4] > 0:
            self.K = (m.k[0], m.k[4], m.k[2], m.k[5])

    def _on_depth(self, m):
        self.depths.append(m)

    def _on_odom(self, m):
        p, o = m.pose.pose.position, m.pose.pose.orientation
        self.poses.append((stamp_s(m.header), np.array([p.x, p.y, p.z]), D.quat_to_mat(o.w, o.x, o.y, o.z)))

    def _on_status(self, m):
        """The mission's plan, NED in its status; drawn here in the odometry frame (ENU)."""
        try:
            st = json.loads(m.data)
        except ValueError:
            return
        home, targets, sp = st.get("home"), st.get("targets") or [], st.get("setpoint")
        route = []
        if home:
            route = [F.ned_to_enu(home[0], home[1], 0.0)[:2]] + [F.ned_to_enu(n, e, 0.0)[:2] for n, e in targets]
        self.plan = (route, F.ned_to_enu(*sp[:3])[:2] if sp else None)

    def _skip(self, why):
        self.skipped[why] = self.skipped.get(why, 0) + 1
        if self.skipped[why] in (1, 10, 100, 1000):
            self.get_logger().warning(f"depth frame not mapped ({self.skipped[why]}x): {why}")

    def _calibration(self, frame_id):
        if self.extrinsic is not None and self.extrinsic[0] == frame_id:
            return self.extrinsic
        from rclpy.time import Time
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, frame_id, Time())
        except Exception as e:                      # not (yet) on the TF tree
            self._skip(f"no TF {self.base_frame} <- {frame_id} ({type(e).__name__})")
            return None
        r, t = tf.transform.rotation, tf.transform.translation
        self.extrinsic = (frame_id, D.quat_to_mat(r.w, r.x, r.y, r.z), np.array([t.x, t.y, t.z]))
        return self.extrinsic

    def _pose_at(self, t):
        if not self.poses:
            return None
        best = min(self.poses, key=lambda q: abs(q[0] - t))
        return best if abs(best[0] - t) <= self.max_pose_dt else None

    # ------------------------------------------------------------------ work
    def _next_depth(self):
        """The newest buffered depth frame the odometry has caught up with; it and everything older
        leave the buffer. None while the odometry is still behind every frame."""
        if not self.poses:
            return None
        t_odom = self.poses[-1][0] + self.max_pose_dt
        ready = [i for i, m in enumerate(self.depths) if stamp_s(m.header) <= t_odom]
        if not ready:
            if len(self.depths) == self.depths.maxlen:
                self._skip(f"odometry is more than {len(self.depths)} depth frames behind")
            return None
        m = self.depths[ready[-1]]
        for _ in range(ready[-1] + 1):
            self.depths.popleft()
        return m

    def _integrate(self):
        m = self._next_depth()
        if m is None:
            return
        if self.K is None:
            return self._skip("no camera_info yet")
        cal = self._calibration(m.header.frame_id)
        if cal is None:
            return None
        pose = self._pose_at(stamp_s(m.header))
        if pose is None:
            return self._skip(f"no odometry within {self.max_pose_dt * 1000:.0f} ms of the depth stamp")
        try:
            d = D.decode_depth(m.encoding, m.height, m.width, m.step, bytes(m.data))
        except ValueError as e:
            return self._skip(str(e))
        _, p_wb, R_wb = pose
        _, R_bo, t_bo = cal
        dirs, ranges = M.camera_rays(d, self.K, self.stride)
        self.map.integrate(p_wb + R_wb @ t_bo, dirs @ (R_wb @ R_bo).T, ranges, self.max_range)
        self.trail.append((p_wb[0], p_wb[1], math.atan2(R_wb[1, 0], R_wb[0, 0])))
        return None

    def _publish(self):
        if self.map.frames == 0:
            return
        header = Header(frame_id=self.frame)
        header.stamp = self.get_clock().now().to_msg()
        t0 = time.perf_counter()
        pts = self.map.occupied().astype(np.float32)
        self.pub_cloud.publish(point_cloud2.create_cloud_xyz32(header, pts))
        img = self._render()
        msg = Image(header=header, height=img.shape[0], width=img.shape[1], encoding="bgr8",
                    is_bigendian=0, step=img.shape[1] * 3, data=img.tobytes())
        self.pub_image.publish(msg)
        dt = time.perf_counter() - t0
        if dt > 0.2:
            self.get_logger().warning(f"publishing the map took {dt * 1000:.0f} ms", throttle_duration_sec=30.0)

    def _render(self):
        s = self.map.slice(*self.z_band)
        img = np.empty(s.shape + (3,), np.uint8)
        for k, c in COLOURS.items():
            img[s == k] = c
        k = self.scale
        img = cv2.resize(img, (s.shape[1] * k, s.shape[0] * k), interpolation=cv2.INTER_NEAREST)
        px = lambda x, y: tuple(int((c + 0.5) * k) for c in reversed(self.map.pixel(x, y)))  # (col, row)
        r0 = px(0.0, 0.0)
        cv2.drawMarker(img, r0, (0, 160, 0), cv2.MARKER_CROSS, 4 * k, 1)                      # the origin
        if self.plan:                                                                          # the plan
            route, sp = self.plan
            pts = [px(x, y) for x, y in route]
            if len(pts) > 1:
                cv2.polylines(img, [np.array(pts, np.int32)], False, (255, 170, 68), 1)
            for q in pts[1:]:
                cv2.circle(img, q, max(2, k), (255, 170, 68), -1)
            if sp is not None:
                cv2.circle(img, px(*sp), 2 * k, (255, 68, 255), 2)
        trail = [px(x, y) for x, y, _ in self.trail]
        if len(trail) > 1:
            cv2.polylines(img, [np.array(trail, np.int32)], False, (60, 180, 60), 1)
        if self.trail:
            x, y, yaw = self.trail[-1]
            tip = px(x + 0.4 * math.cos(yaw), y + 0.4 * math.sin(yaw))
            cv2.arrowedLine(img, px(x, y), tip, (0, 220, 255), 2, tipLength=0.4)
        return img


def main(argv=None):
    rclpy.init(args=list(argv) if argv is not None else sys.argv)
    node = MapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
