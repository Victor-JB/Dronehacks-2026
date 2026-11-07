"""SITL helper: republish PX4 /fmu/out/vehicle_odometry (NED/FRD) as nav_msgs/Odometry (ENU/FLU).

Stands in for cuVSLAM's /visual_slam/tracking/odometry when flying PX4 SITL in Gazebo.
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .backends import PX4_QOS


class SitlOdom(Node):
    def __init__(self):
        super().__init__("dh_sitl_odom")
        from px4_msgs.msg import VehicleOdometry
        self.declare_parameter("out_topic", "/dh/sim_odom")
        self.pub = self.create_publisher(Odometry, self.get_parameter("out_topic").value, 10)
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self._cb, PX4_QOS)

    def _cb(self, m):
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = "odom"
        o.child_frame_id = "base_link"
        n, e, d = m.position
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = float(e), float(n), float(-d)
        w, x, y, z = m.q
        yaw_ned = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        yaw = math.pi / 2 - yaw_ned
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        vn, ve, vd = m.velocity
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z = float(ve), float(vn), float(-vd)
        self.pub.publish(o)


def main():
    rclpy.init()
    n = SitlOdom()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
