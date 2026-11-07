"""Stand-in for the OAK-D S2 driver: Gazebo's rendered camera, on the drone's /oak/* topics.

  /sim/oak/depth_image (32FC1 m)  -> /oak/stereo/image_raw  16UC1 millimetres, 0 = no return
  /sim/oak/camera_info            -> /oak/stereo/camera_info
  /sim/oak/image (rgb8)           -> /oak/left/image_mono   mono8 (what the camera page shows)

Frame ids and the TF chain base_link -> oak_parent_frame -> oak_{left,right}_camera_optical_frame
follow the drone: skybeta's launch puts oak_parent_frame AT base_link (camera_x/y/z default 0),
and depthai hangs the cameras off it 7.5 cm apart. The rendered camera itself sits where the
OAK is on the airframe (sim/models/skybeta). Values are converted, never perturbed.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster

DEPTH_FRAME = 'oak_right_camera_optical_frame'
LEFT_FRAME = 'oak_left_camera_optical_frame'
BASELINE = 0.075


def optical(parent, child, y):
    """parent (FLU) -> optical (x right, y down, z forward), as depthai's URDF."""
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = parent, child
    t.transform.translation.y = y
    # roll -90, yaw -90 (Rz * Ry * Rx), as quaternion (w, x, y, z) = (0.5, -0.5, 0.5, -0.5)
    t.transform.rotation.w, t.transform.rotation.x = 0.5, -0.5
    t.transform.rotation.y, t.transform.rotation.z = 0.5, -0.5
    return t


class OakSim(Node):
    def __init__(self):
        super().__init__('oak_sim')
        self.pub_depth = self.create_publisher(Image, '/oak/stereo/image_raw', 10)
        self.pub_info = self.create_publisher(CameraInfo, '/oak/stereo/camera_info', 10)
        self.pub_mono = self.create_publisher(Image, '/oak/left/image_mono', 10)
        self.create_subscription(Image, '/sim/oak/depth_image', self.on_depth, 10)
        self.create_subscription(CameraInfo, '/sim/oak/camera_info', self.on_info, 10)
        self.create_subscription(Image, '/sim/oak/image', self.on_color, 10)
        self.static = StaticTransformBroadcaster(self)
        rig = TransformStamped()
        rig.header.frame_id, rig.child_frame_id = 'base_link', 'oak_parent_frame'
        rig.transform.rotation.w = 1.0
        self.static.sendTransform([rig, optical('oak_parent_frame', DEPTH_FRAME, -BASELINE/2),
                                   optical('oak_parent_frame', LEFT_FRAME, BASELINE/2)])

    def on_depth(self, m):
        if m.encoding != '32FC1':
            self.get_logger().warning(f'unexpected depth encoding {m.encoding}', throttle_duration_sec=10.0)
            return
        z = np.frombuffer(m.data, np.float32).reshape(m.height, m.step // 4)[:, :m.width]
        mm = np.where(np.isfinite(z) & (z > 0), np.round(z * 1000.0), 0.0)
        out = Image()
        out.header.stamp, out.header.frame_id = m.header.stamp, DEPTH_FRAME
        out.height, out.width, out.encoding, out.is_bigendian = m.height, m.width, '16UC1', 0
        out.step = 2 * m.width
        out.data = np.clip(mm, 0, 65535).astype(np.uint16).tobytes()
        self.pub_depth.publish(out)

    def on_info(self, m):
        m.header.frame_id = DEPTH_FRAME
        if not m.distortion_model:
            m.distortion_model, m.d = 'plumb_bob', [0.0] * 5
        self.pub_info.publish(m)

    def on_color(self, m):
        if m.encoding not in ('rgb8', 'bgr8'):
            return
        rgb = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :3 * m.width]
        rgb = rgb.reshape(m.height, m.width, 3).astype(np.float32)
        weights = (0.299, 0.587, 0.114) if m.encoding == 'rgb8' else (0.114, 0.587, 0.299)
        mono = np.clip(rgb @ np.array(weights, np.float32), 0, 255).astype(np.uint8)
        out = Image()
        out.header.stamp, out.header.frame_id = m.header.stamp, LEFT_FRAME
        out.height, out.width, out.encoding, out.step = m.height, m.width, 'mono8', m.width
        out.data = mono.tobytes()
        self.pub_mono.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OakSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
