"""Stand-in for cuVSLAM: Gazebo's ground-truth pose, published exactly where cuVSLAM publishes.

On the drone cuVSLAM starts its `odom` frame at the camera rig's pose when it starts and reports
base_link in it. This node does the same with Gazebo's true pose: the first pose it sees becomes
the odom origin. Nothing is added (no noise, drift, latency or dropouts).

  in   /sim/truth/odometry            nav_msgs/Odometry, ENU world, from ros_gz_bridge
  out  /visual_slam/tracking/odometry nav_msgs/Odometry, odom -> base_link (ENU/FLU), as cuVSLAM
       /visual_slam/status            vo_state = 1 (tracking) with every pose
       TF odom -> base_link, map -> odom (identity), as cuVSLAM publishes them
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus

COVARIANCE = 1e-4      # a variance must be set (cuVSLAM always fills one); PX4 floors it at EKF2_EVP_NOISE^2


def q_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw*bw - ax*bx - ay*by - az*bz, aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx, aw*bz + ax*by - ay*bx + az*bw)


def q_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def q_rotate(q, v):
    w, x, y, z = q_mul(q_mul(q, (0.0, *v)), q_conj(q))
    return (x, y, z)


class VioSim(Node):
    def __init__(self):
        super().__init__('vio_sim')
        self.origin = None     # (p0, q0) in the world, the odom frame
        self.pub = self.create_publisher(Odometry, '/visual_slam/tracking/odometry', 10)
        self.pub_status = self.create_publisher(VisualSlamStatus, '/visual_slam/status', 10)
        self.tf = TransformBroadcaster(self)
        self.tf_static = StaticTransformBroadcaster(self)
        identity = TransformStamped()
        identity.header.frame_id, identity.child_frame_id = 'map', 'odom'
        identity.transform.rotation.w = 1.0
        self.tf_static.sendTransform(identity)
        self.create_subscription(Odometry, '/sim/truth/odometry', self.on_truth, 10)
        self.get_logger().info('cuVSLAM stand-in: ground truth -> /visual_slam/tracking/odometry')

    def on_truth(self, m):
        p = m.pose.pose.position
        o = m.pose.pose.orientation
        p, q = (p.x, p.y, p.z), (o.w, o.x, o.y, o.z)
        if self.origin is None:
            self.origin = (p, q)
            self.get_logger().info(f'odom origin at world ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})')
        p0, q0 = self.origin
        rel = q_rotate(q_conj(q0), tuple(a - b for a, b in zip(p, p0)))
        q_rel = q_mul(q_conj(q0), q)

        out = Odometry()
        out.header.stamp = m.header.stamp
        out.header.frame_id, out.child_frame_id = 'odom', 'base_link'
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = rel
        r = out.pose.pose.orientation
        r.w, r.x, r.y, r.z = q_rel
        out.twist.twist = m.twist.twist          # body frame (base_link) on both sides
        for i in range(6):
            out.pose.covariance[7*i] = COVARIANCE
            out.twist.covariance[7*i] = COVARIANCE
        self.pub.publish(out)

        tf = TransformStamped()
        tf.header = out.header
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = rel
        tf.transform.rotation = out.pose.pose.orientation
        self.tf.sendTransform(tf)

        status = VisualSlamStatus()
        status.header.stamp, status.header.frame_id = m.header.stamp, 'odom'
        status.vo_state = 1
        self.pub_status.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = VioSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
