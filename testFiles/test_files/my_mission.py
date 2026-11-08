import json
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# ENU metres, relative to where the drone was when Offboard started.
WAYPOINTS = [(0.0, 0.0, 1.0), (1.0, 0.0, 1.0), (1.0, 1.0, 1.0)]
ARRIVED_M = 0.3

# Landing, after the last waypoint is reached.
#   The 0.3 m floor only applies to pose setpoints, so the descent is a
#   downward twist. Nothing here disarms: PX4 disarms by itself
#   COM_DISARM_LAND (2 s) after it detects the landing. Never publish to
#   /fmu/in/* (autonomy_pipeline.md).
HOVER_BEFORE_LAND_S = 2.0   # settle at the last waypoint before descending
DESCENT_MPS = 0.3           # descent speed, m/s
LAND_TIMEOUT_S = 20.0       # no touchdown by then: stop pushing down, hover, pilot lands


class MyMission(Node):

    def __init__(self):
        super().__init__('my_mission')
        self.pub = self.create_publisher(PoseStamped, '/skybeta/setpoint/pose', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/skybeta/setpoint/twist', 10)
        self.create_subscription(
            Odometry, '/visual_slam/tracking/odometry', self.on_odom, 10)
        self.create_subscription(String, '/skybeta/state', self.on_state, 10)
        self.position = None     # current ENU position
        self.offboard = False
        self.armed = False
        self.landed = None       # PX4's land detector, from /skybeta/state
        self.start = None        # ENU position when Offboard started
        self.index = 0
        self.phase = 'idle'      # idle -> waypoints -> settle -> land -> done
        self.phase_since = None  # seconds, when the current phase began
        self.create_timer(0.1, self.tick)   # 10 Hz

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.position = (p.x, p.y, p.z)

    def on_state(self, msg):
        state = json.loads(msg.data)
        self.offboard = bool(state.get('offboard_active', False))
        self.armed = bool(state.get('armed', False))
        self.landed = state.get('landed')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def set_phase(self, phase):
        self.phase = phase
        self.phase_since = self.now()

    def send(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        # orientation left all-zero: do not command yaw
        self.pub.publish(msg)

    def send_velocity(self, vx, vy, vz):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.twist.linear.x = float(vx)    # world ENU axes, not body
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        self.twist_pub.publish(msg)

    def tick(self):
        if self.position is None:
            return                      # no VIO yet: publish nothing
        if not self.offboard:
            if self.start is not None:
                self.get_logger().warn('Offboard ended: the pilot has it. Mission reset.')
                self.start, self.index = None, 0
                self.set_phase('idle')
            self.send(*self.position)   # keeps a fresh setpoint: required for the takeover
            return
        if self.start is None:
            self.start = self.position
            self.set_phase('waypoints')
            self.get_logger().info('Offboard active: mission start')

        if self.phase == 'waypoints':
            dx, dy, dz = WAYPOINTS[self.index]
            target = (self.start[0] + dx, self.start[1] + dy, self.start[2] + dz)
            self.send(*target)
            if math.dist(self.position, target) < ARRIVED_M:
                if self.index < len(WAYPOINTS) - 1:
                    self.index += 1
                    self.get_logger().info(f'waypoint {self.index}: {WAYPOINTS[self.index]}')
                else:
                    self.get_logger().info('last waypoint reached: settling before landing')
                    self.set_phase('settle')

        elif self.phase == 'settle':
            dx, dy, dz = WAYPOINTS[-1]
            self.send(self.start[0] + dx, self.start[1] + dy, self.start[2] + dz)
            if self.now() - self.phase_since > HOVER_BEFORE_LAND_S:
                self.get_logger().info(f'landing: descending at {DESCENT_MPS} m/s')
                self.set_phase('land')

        elif self.phase == 'land':
            if self.landed or not self.armed:
                self.get_logger().info('touchdown: PX4 disarms by itself in ~2 s')
                self.set_phase('done')
            elif self.now() - self.phase_since > LAND_TIMEOUT_S:
                self.get_logger().warn(
                    f'no touchdown after {LAND_TIMEOUT_S:.0f} s: hovering. Pilot, take over and land.')
                self.set_phase('done')
                self.send(*self.position)   # hold here
            else:
                self.send_velocity(0.0, 0.0, -DESCENT_MPS)

        elif self.phase == 'done':
            if self.armed and self.landed:
                self.send_velocity(0.0, 0.0, -DESCENT_MPS)   # stay pressed down until PX4 disarms
            # otherwise: publish nothing new; the last pose holds (or the twist expires
            # after 1 s and flight_interface holds position)


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
