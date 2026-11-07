"""The safety pilot, scripted. On the drone a human does exactly this with the RC transmitter:

  1. waits for the mission to report READY (/refnav/status phase "arm")
  2. switches to Altitude and arms (sticks centred, so PX4 holds on the ground)
  3. switches to Position; skybeta_flight_interface (auto_offboard_on_arm) then takes Offboard
  4. when the mission ends and holds ("land" / "done"), takes over and lands (Auto Land)

The sticks are a MAVLink MANUAL_CONTROL stream, centred, 20 Hz: the stand-in for the RC link
(PX4 COM_RC_IN_MODE = 1). The mode switches and arming are the MAVLink commands a GCS would send.
This node never touches /fmu/in/* and never sends a setpoint.
"""
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pymavlink import mavutil

ALTCTL, POSCTL, AUTO, AUTO_LAND = 2, 3, 4, 6          # PX4 custom main modes / auto sub-mode
NAV_OFFBOARD = 14


class Pilot(Node):
    def __init__(self):
        super().__init__('pilot')
        url = self.declare_parameter('mavlink_url', 'udpin:127.0.0.1:14540').value
        self.arm_timeout = float(self.declare_parameter('offboard_timeout_s', 15.0).value)
        self.mav = mavutil.mavlink_connection(url, source_system=255, source_component=190)
        self.state, self.sky, self.phase = 'WAIT_READY', None, None
        self.t_state = time.monotonic()
        self.flew = False
        self.create_subscription(String, '/skybeta/state', self.on_sky, 10)
        self.create_subscription(String, '/refnav/status', self.on_status, 10)
        self.create_timer(0.05, self.tick)
        self.last_hb = 0.0
        self.get_logger().info(f'pilot on {url}: waiting for READY')

    def on_sky(self, m):
        try:
            self.sky = json.loads(m.data)
        except ValueError:
            pass

    def on_status(self, m):
        try:
            self.phase = json.loads(m.data).get('phase')
        except ValueError:
            pass

    def say(self, text):
        self.get_logger().info(f'[pilot] {text}')

    def go(self, state, text):
        self.state, self.t_state = state, time.monotonic()
        self.say(text)

    def command(self, cmd, *params):
        p = list(params) + [0.0] * (7 - len(params))
        self.mav.mav.command_long_send(1, 1, cmd, 0, *p)

    def mode(self, main, sub=0):
        self.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, main, sub)

    def tick(self):
        now = time.monotonic()
        while self.mav.recv_match(blocking=False) is not None:
            pass
        # sticks centred: roll/pitch/yaw 0, throttle 500 (mid: hold height in Altitude/Position)
        self.mav.mav.manual_control_send(1, 0, 0, 500, 0, 0)
        if now - self.last_hb > 1.0:
            self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.last_hb = now
        sky = self.sky or {}
        armed, nav = sky.get('armed'), sky.get('nav_state')
        age = now - self.t_state

        if self.state == 'WAIT_READY':
            if self.phase == 'arm' and sky.get('px4_link'):
                self.mode(ALTCTL)
                self.go('ARMING', 'mission READY: Altitude, arm')
        elif self.state == 'ARMING':
            if not armed and age > 0.5 and int(age * 2) % 4 == 0:
                self.mode(ALTCTL)
                self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            if armed:
                self.go('TO_POSITION', 'armed: switching to Position')
            elif age > 10:
                self.go('WAIT_READY', 'arming failed, waiting for READY again')
        elif self.state == 'TO_POSITION':
            if age > 1.0 and int(age * 2) % 4 == 0:
                self.mode(POSCTL)
            if nav == NAV_OFFBOARD:
                self.flew = True
                self.go('FLYING', 'Offboard: the mission has control')
            elif age > self.arm_timeout:
                self.mode(AUTO, AUTO_LAND)
                self.go('LANDING', 'no Offboard: landing')
        elif self.state == 'FLYING':
            if self.phase in ('land', 'done') or (nav is not None and nav != NAV_OFFBOARD):
                self.mode(AUTO, AUTO_LAND)
                self.go('LANDING', f'mission phase {self.phase}, nav_state {nav}: taking over, landing')
        elif self.state == 'LANDING':
            if armed is False:
                self.go('DONE', 'landed and disarmed')
            elif int(age) % 3 == 0 and int(age * 20) % 20 == 0:
                self.mode(AUTO, AUTO_LAND)


def main(args=None):
    rclpy.init(args=args)
    node = Pilot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
