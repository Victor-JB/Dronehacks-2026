"""Flight-controller backends.

SkybetaBackend  - the organizers' competitor API (github.com/Artash-N/skybeta,
                  docs/competitor_guide.md): ENU twist/pose setpoints in the
                  cuVSLAM `odom` frame; never arms / changes mode / lands.
Px4Backend      - direct px4_msgs over uXRCE-DDS (OffboardControlMode,
                  TrajectorySetpoint, VehicleCommand). For SITL, or on the real
                  drone ONLY if the organizers allow it: their guide says not to
                  publish /fmu/in/* while their flight_interface runs.

Both take commands in the ENU `odom` frame (the frame of
/visual_slam/tracking/odometry). Yaw is ENU (CCW from +x).

Command-gating contract (Sky Beta competitor guide, as recorded in the research handoff section 6.3):
  * the organizers' flight_interface streams OffboardControlMode itself (10 Hz) and never arms or
    changes mode; team nodes must WAIT FOR `offboard_active` BEFORE COMMANDING. So SkybetaBackend
    publishes nothing until /skybeta/state reports offboard_active (requires_setpoint_stream = False);
  * twist setpoints expire after 1.0 s (then the interface holds its last safe position);
  * /skybeta/hold freezes at the last safe position; /skybeta/request_offboard is SITL-only.
  Px4Backend is the raw-PX4 path (SITL only unless the organizers approve): PX4 itself requires a
  setpoint stream before it accepts Offboard (requires_setpoint_stream = True).
"""
from __future__ import annotations

import json
import math
import time
from typing import Optional

from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from std_msgs.msg import String

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class BackendBase:
    can_arm = False
    can_land = False
    requires_setpoint_stream = False   # True: setpoints must stream BEFORE Offboard (raw PX4)

    def __init__(self, node: Node):
        self.node = node

    # --- commands (odom ENU frame)
    def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float) -> None: ...
    def send_position(self, x: float, y: float, z: float, yaw: Optional[float]) -> None: ...
    def hold(self) -> None: ...
    def arm(self) -> bool: return False
    def disarm(self) -> bool: return False
    def land(self) -> bool: return False
    def request_offboard(self) -> bool: return False

    # --- status
    def offboard_active(self) -> bool: return False
    def armed(self) -> Optional[bool]: return None
    def landed(self) -> Optional[bool]: return None
    def tick(self) -> None: ...


class SkybetaBackend(BackendBase):
    """Organizers' interface: /skybeta/setpoint/{pose,twist}, /skybeta/state, /skybeta/hold."""

    def __init__(self, node: Node, ns: str = "/skybeta", state_timeout_s: float = 1.0):
        super().__init__(node)
        ns = ns.rstrip("/")
        self.ns = ns
        self.state_timeout = state_timeout_s
        self.pub_twist = node.create_publisher(TwistStamped, ns + "/setpoint/twist", 10)
        self.pub_pose = node.create_publisher(PoseStamped, ns + "/setpoint/pose", 10)
        self.state = {}
        self.t_state = None
        self.n_sent = 0
        node.create_subscription(String, ns + "/state", self._on_state, qos_profile_sensor_data)
        self._hold_cli = None
        try:
            from std_srvs.srv import Trigger
            self._Trigger = Trigger
            self._hold_cli = node.create_client(Trigger, ns + "/hold")
            self._off_cli = node.create_client(Trigger, ns + "/request_offboard")
        except ImportError:
            self._off_cli = None

    def _now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _on_state(self, msg: String):
        try:
            st = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(st, dict):
            self.state = st
            self.t_state = self._now()

    def state_fresh(self) -> bool:
        return self.t_state is not None and self._now() - self.t_state <= self.state_timeout

    def send_velocity(self, vx, vy, vz, yaw_rate):
        m = TwistStamped()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.header.frame_id = "odom"
        m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = float(vx), float(vy), float(vz)
        m.twist.angular.z = float(yaw_rate)
        self.pub_twist.publish(m)
        self.n_sent += 1

    def send_position(self, x, y, z, yaw):
        m = PoseStamped()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.header.frame_id = "odom"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
        if yaw is not None:  # all-zero quaternion = "hold heading" in the skybeta interface
            qx, qy, qz, qw = yaw_to_quat(yaw)
            m.pose.orientation.x, m.pose.orientation.y = qx, qy
            m.pose.orientation.z, m.pose.orientation.w = qz, qw
        else:
            m.pose.orientation.w = 0.0
        self.pub_pose.publish(m)

    def hold(self) -> bool:
        if self._hold_cli is not None and self._hold_cli.service_is_ready():
            self._hold_cli.call_async(self._Trigger.Request())
            return True
        return False

    def request_offboard(self):
        if self._off_cli is not None and self._off_cli.service_is_ready():
            self._off_cli.call_async(self._Trigger.Request())
            return True
        return False

    def offboard_active(self):
        # unknown (no or stale /skybeta/state) counts as NOT active: never command blind
        return self.state_fresh() and bool(self.state.get("offboard_active", False))

    def frozen(self):
        return bool(self.state.get("frozen", False))

    def armed(self):
        return self.state.get("armed")

    def clamped(self):
        return self.state.get("clamped", [])


FORBIDDEN_IN_DRY_RUN = ("/skybeta", "/fmu")   # topic/service prefixes of the physical flight-control path


def _forbidden(name: str) -> bool:
    n = "/" + name.lstrip("/")
    return any(n == p or n.startswith(p + "/") for p in FORBIDDEN_IN_DRY_RUN)


class DryRunBackend(BackendBase):
    """Hardware-in-the-loop dry run: commands go ONLY to /dh/dry_run/* and the log.

    Isolation is enforced in code, not configuration:
      * this class never creates a publisher or service client on /skybeta/* or /fmu/*
        (its own topic names are checked against FORBIDDEN_IN_DRY_RUN at construction);
      * `verify_isolation(node)` (called by the mission node at start-up) inspects every publisher and
        client the node owns and raises if any reaches the flight-control interface.
    It may READ /skybeta/state (subscriptions cannot command anything) to report what the real interface
    shows. "Offboard" is simulated: authority is granted after /dh/start so the pipeline can be exercised
    on a STATIONARY drone; the generated commands therefore describe a drone that does not move.
    """
    can_arm = False
    can_land = False

    def __init__(self, node: Node, out_ns: str = "/dh/dry_run"):
        super().__init__(node)
        out_ns = out_ns.rstrip("/")
        if _forbidden(out_ns):
            raise RuntimeError("dry-run output namespace %r is a flight-control namespace" % out_ns)
        self.pub_cmd = node.create_publisher(TwistStamped, out_ns + "/cmd_vel", 10)
        self.pub_evt = node.create_publisher(String, out_ns + "/events", 10)
        self.real_state = {}
        node.create_subscription(String, "/skybeta/state", self._on_state, qos_profile_sensor_data)
        self.authority = False
        self.n_sent = 0
        self.last_cmd = (0.0, 0.0, 0.0, 0.0)

    def _on_state(self, msg: String):
        try:
            st = json.loads(msg.data)
            if isinstance(st, dict):
                self.real_state = st
        except ValueError:
            pass

    def _event(self, text: str):
        self.pub_evt.publish(String(data=text))
        self.node.get_logger().info("[DRY RUN] " + text)

    def grant_authority(self):
        if not self.authority:
            self.authority = True
            self._event("simulated Offboard authority granted (nothing is sent to the flight controller)")

    def send_velocity(self, vx, vy, vz, yaw_rate):
        m = TwistStamped()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.header.frame_id = "odom"
        m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = float(vx), float(vy), float(vz)
        m.twist.angular.z = float(yaw_rate)
        self.pub_cmd.publish(m)
        self.n_sent += 1
        self.last_cmd = (float(vx), float(vy), float(vz), float(yaw_rate))

    def send_position(self, x, y, z, yaw):
        self._event("position setpoint (not sent): %.2f %.2f %.2f" % (x, y, z))

    def hold(self):
        self._event("hold requested (not sent)")
        return True

    def offboard_active(self):
        return self.authority


def verify_isolation(node: Node, dry_run: bool) -> list:
    """Return the node's publisher/client names; in dry-run raise if any is on the flight-control path."""
    names = []
    for pub in getattr(node, "publishers", []):
        names.append(getattr(pub, "topic_name", getattr(pub, "topic", "")))
    for cli in getattr(node, "clients", []):
        names.append(getattr(cli, "srv_name", getattr(cli, "service_name", "")))
    if dry_run:
        bad = [n for n in names if n and _forbidden(n)]
        if bad:
            raise RuntimeError("DRY RUN ISOLATION VIOLATED: node owns flight-control endpoints %s" % bad)
    return names


class Px4Backend(BackendBase):
    """Direct PX4 control over uXRCE-DDS (px4_msgs, release/1.16 message set).
    SITL ONLY unless the organizers approve: writes /fmu/in/*, which their guide forbids on the drone."""
    can_arm = True
    can_land = True
    requires_setpoint_stream = True

    def __init__(self, node: Node, yaw_offset_rad: float = 0.0, target_system: int = 1):
        super().__init__(node)
        from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                                  VehicleLandDetected)
        self.M = dict(OCM=OffboardControlMode, TS=TrajectorySetpoint, VC=VehicleCommand)
        self.yaw_off = yaw_offset_rad
        self.sys = target_system
        self.pub_ocm = node.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", PX4_QOS)
        self.pub_ts = node.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", PX4_QOS)
        self.pub_cmd = node.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", PX4_QOS)
        self._landed = None
        self._status = None
        node.create_subscription(VehicleLandDetected, "/fmu/out/vehicle_land_detected",
                                 lambda m: setattr(self, "_landed", bool(m.landed)), PX4_QOS)
        self._sub_status(node)
        self._mode = "velocity"
        self._last = (math.nan,) * 3, (0.0, 0.0, 0.0), math.nan, 0.0

    def _sub_status(self, node):
        # v1.16 versions VehicleStatus -> topic suffix _v1 (resolve from MESSAGE_VERSION)
        from px4_msgs.msg import VehicleStatus
        ver = getattr(VehicleStatus, "MESSAGE_VERSION", 0)
        topic = "/fmu/out/vehicle_status" + ("_v%d" % ver if ver else "")
        node.create_subscription(VehicleStatus, topic, lambda m: setattr(self, "_status", m), PX4_QOS)

    def _us(self):
        return int(self.node.get_clock().now().nanoseconds / 1000)

    def _enu_to_ned_vec(self, x, y, z):
        c, s = math.cos(self.yaw_off), math.sin(self.yaw_off)
        xr, yr = c * x - s * y, s * x + c * y
        return yr, xr, -z

    def _enu_to_ned_yaw(self, yaw):
        return math.pi / 2.0 - (yaw + self.yaw_off)

    def send_velocity(self, vx, vy, vz, yaw_rate):
        self._mode = "velocity"
        self._last = ((math.nan,) * 3, self._enu_to_ned_vec(vx, vy, vz), math.nan, -yaw_rate)

    def send_position(self, x, y, z, yaw):
        self._mode = "position"
        self._last = (self._enu_to_ned_vec(x, y, z), (math.nan,) * 3,
                      self._enu_to_ned_yaw(yaw) if yaw is not None else math.nan, math.nan)

    def hold(self):
        self.send_velocity(0.0, 0.0, 0.0, 0.0)

    def tick(self):
        """Call at >= 10 Hz: heartbeat + current setpoint."""
        ocm = self.M["OCM"]()
        ocm.timestamp = self._us()
        ocm.position = self._mode == "position"
        ocm.velocity = self._mode == "velocity"
        self.pub_ocm.publish(ocm)
        ts = self.M["TS"]()
        ts.timestamp = self._us()
        pos, vel, yaw, yawspeed = self._last
        ts.position = [float(v) for v in pos]
        ts.velocity = [float(v) for v in vel]
        ts.acceleration = [math.nan] * 3
        ts.yaw = float(yaw)
        ts.yawspeed = float(yawspeed)
        self.pub_ts.publish(ts)

    def _command(self, command, p1=0.0, p2=0.0, p7=0.0):
        VC = self.M["VC"]
        m = VC()
        m.timestamp = self._us()
        m.command = command
        m.param1, m.param2, m.param7 = float(p1), float(p2), float(p7)
        m.target_system, m.target_component = self.sys, 1
        m.source_system, m.source_component = 1, 1
        m.from_external = True
        self.pub_cmd.publish(m)
        return True

    def request_offboard(self):
        return self._command(self.M["VC"].VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def arm(self):
        return self._command(self.M["VC"].VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def disarm(self):
        return self._command(self.M["VC"].VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)

    def land(self):
        return self._command(self.M["VC"].VEHICLE_CMD_NAV_LAND)

    def offboard_active(self):
        return self._status is not None and self._status.nav_state == self._status.NAVIGATION_STATE_OFFBOARD

    def armed(self):
        if self._status is None:
            return None
        return self._status.arming_state == self._status.ARMING_STATE_ARMED

    def landed(self):
        return self._landed
