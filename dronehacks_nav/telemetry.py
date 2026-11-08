#!/usr/bin/env python3
"""dronehacks_nav.telemetry - one live line: height, attitude, gyro, motors. Read-only.

    ros2 run dronehacks_nav telemetry            (in the container: docker exec -it skybeta-stack bash)

  VIO z     /visual_slam/tracking/odometry, metres above where the stack started
  PX4 z     /fmu/out/vehicle_local_position (EKF2: VIO + rangefinder), metres up
  range     the same message's dist_bottom: the downward rangefinder, metres to the floor
  att       /fmu/out/vehicle_attitude: roll / pitch / yaw, degrees
  gyro      /fmu/out/sensor_combined: body rates, deg/s
  motors    /fmu/out/actuator_motors: each motor's command 0..1 ("-" if PX4 does not publish it over DDS)
  mode      /skybeta/state: armed, nav_state, Offboard
"""
from __future__ import annotations

import json
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)


def rpy(w, x, y, z):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return tuple(math.degrees(a) for a in (roll, pitch, yaw))


class Telemetry(Node):
    def __init__(self):
        super().__init__("dronehacks_telemetry")
        from px4_msgs.msg import SensorCombined, VehicleAttitude, VehicleLocalPosition
        self.v = {}
        self.create_subscription(Odometry, "/visual_slam/tracking/odometry", self._odom, 10)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self._lpos, PX4_QOS)
        self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude", self._att, PX4_QOS)
        self.create_subscription(SensorCombined, "/fmu/out/sensor_combined", self._imu, PX4_QOS)
        self.create_subscription(String, "/skybeta/state", self._sky, 10)
        try:
            from px4_msgs.msg import ActuatorMotors
            self.create_subscription(ActuatorMotors, "/fmu/out/actuator_motors", self._motors, PX4_QOS)
        except ImportError:
            pass
        self.create_timer(0.1, self._print)

    def _odom(self, m):
        self.v["vio_z"] = m.pose.pose.position.z

    def _lpos(self, m):
        self.v["px4_z"] = -m.z
        self.v["range"] = m.dist_bottom if m.dist_bottom_valid else None

    def _att(self, m):
        self.v["att"] = rpy(*m.q)

    def _imu(self, m):
        self.v["gyro"] = tuple(math.degrees(g) for g in m.gyro_rad)

    def _motors(self, m):
        self.v["motors"] = [c for c in m.control[:4]]

    def _sky(self, m):
        try:
            s = json.loads(m.data)
        except ValueError:
            return
        self.v["mode"] = f"{'ARMED' if s.get('armed') else 'disarmed'} {s.get('nav_state_name', '?')}" \
                         f"{' OFFBOARD' if s.get('offboard_active') else ''}"

    def _print(self):
        v, f = self.v, (lambda x, n=2: "-" if x is None else f"{x:.{n}f}")
        att, gyro, mot = v.get("att"), v.get("gyro"), v.get("motors")
        line = (f"VIO z {f(v.get('vio_z'))}  PX4 z {f(v.get('px4_z'))}  range {f(v.get('range'))} m | "
                f"att r {f(att and att[0], 0)} p {f(att and att[1], 0)} y {f(att and att[2], 0)} deg | "
                f"gyro {' '.join(f(g, 0) for g in gyro) if gyro else '-'} deg/s | "
                f"motors {' '.join('-' if c != c else f(c) for c in mot) if mot else '-'} | {v.get('mode', '-')}")
        sys.stdout.write("\r" + line.ljust(150))
        sys.stdout.flush()


def main(argv=None):
    rclpy.init(args=list(argv) if argv is not None else sys.argv)
    node = Telemetry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
