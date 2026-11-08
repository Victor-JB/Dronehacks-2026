import json
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# ENU metres, relative to where the drone was when Offboard started.
WAYPOINTS = [(0.0, 0.0, 1.0), (1.0, 0.0, 1.0), (1.0, 1.0, 1.0)]
ARRIVED_M = 0.3


class MyMission(Node):

    def __init__(self):
        super().__init__('my_mission')
        self.pub = self.create_publisher(PoseStamped, '/skybeta/setpoint/pose', 10)
        self.create_subscription(
            Odometry, '/visual_slam/tracking/odometry', self.on_odom, 10)
        self.create_subscription(String, '/skybeta/state', self.on_state, 10)
        self.position = None     # current ENU position
        self.offboard = False
        self.start = None        # ENU position when Offboard started
        self.index = 0
        self.create_timer(0.1, self.tick)   # 10 Hz

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.position = (p.x, p.y, p.z)

    def on_state(self, msg):
        self.offboard = bool(json.loads(msg.data).get('offboard_active', False))

    def send(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        # orientation left all-zero: do not command yaw
        self.pub.publish(msg)

    def tick(self):
        if self.position is None:
            return                      # no VIO yet: publish nothing
        if not self.offboard:
            if self.start is not None:
                self.get_logger().warn('Offboard ended: the pilot has it. Mission reset.')
                self.start, self.index = None, 0
            self.send(*self.position)   # keeps a fresh setpoint: required for the takeover
            return
        if self.start is None:
            self.start = self.position
            self.get_logger().info('Offboard active: mission start')
        dx, dy, dz = WAYPOINTS[self.index]
        target = (self.start[0] + dx, self.start[1] + dy, self.start[2] + dz)
        self.send(*target)
        if math.dist(self.position, target) < ARRIVED_M and self.index < len(WAYPOINTS) - 1:
            self.index += 1
            self.get_logger().info(f'waypoint {self.index}: {WAYPOINTS[self.index]}')


def main():
    rclpy.init()
    node = MyMission()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                            # Ctrl-C or the stack stopping
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

    
