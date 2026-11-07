#!/usr/bin/env python3
"""dronehacks_nav.skybeta_node - the ROS 2 shell around refnav.core on the skybeta drone. No flight
decisions here: refnav.core makes them, exactly as on the simulator branch.

skybeta's contract (~/skybeta/docs/competitor_guide.md) replaces refnav.node's direct PX4 link:

  pose         /visual_slam/tracking/odometry (nav_msgs/Odometry, ENU odom, FLU body) -> the
               mission's NED position, heading and attitude (dronehacks_nav.frames)
  PX4 state    /skybeta/state (std_msgs/String JSON, 5 Hz): armed, nav_state, px4_link, local_enu
               /fmu/out/vehicle_land_detected (read-only)
  health       derived here, fail-closed: odometry fresh, cuVSLAM vo_state == 1 (tracking) and
               fresh, /skybeta/state fresh with px4_link and a local position
  depth        sensor_msgs/Image + CameraInfo, topics by parameter. skybeta publishes NO depth
               image: with depth_topic empty the mission waits for depth and never asks to fly
  setpoint     /skybeta/setpoint/pose (geometry_msgs/PoseStamped, ENU odom, yaw as quaternion)
  status       /refnav/status (std_msgs/String JSON), as on the simulator branch

What the mission's commands become, because skybeta never arms, never changes mode and has no
land command (skybeta_flight_interface owns /fmu/in/*; never publish there):

  offboard, arm   nothing: the safety pilot arms on Altitude, then switches to Position, and the
                  flight interface enters Offboard if this node is streaming setpoints
  land            nothing: the setpoint stays where the mission stopped it (a hover) and the node
                  tells the pilot to take over and land
  disarm          nothing (the mission only asks for it on the ground)

Motion check (dronehacks_nav.motion_check), while Offboard is ours and nothing is clamped: if commanding
the vehicle does not move it, or VIO's height disagrees with PX4's rangefinder height, the node FREEZES:
it commands the position the odometry reports at that moment, ends the mission through its own landing
(no more legs) and tells the pilot to take over. If VIO itself is what failed, that position is only
as good as VIO; the pilot is the real recovery.

Run (inside the skybeta container; the launch file also starts skybeta_flight_interface):
    ros2 launch dronehacks_nav mission.launch.py
"""
from __future__ import annotations

import json
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from refnav import core as C
from refnav import depth as D

from . import frames as F
from .motion_check import MotionCheck

# PX4's uXRCE-DDS endpoints are BEST_EFFORT + TRANSIENT_LOCAL; a different durability never matches
PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
VO_TRACKING = 1                      # VisualSlamStatus.vo_state: 0 unknown, 1 success, 2 failed


def load_route(node):
    path = node.declare_parameter("route_file", "").value
    flat = list(node.declare_parameter("route", [0.0]).value)
    if path:
        with open(path) as f:
            doc = json.load(f)
        pts = doc["route"] if isinstance(doc, dict) else doc
        return [(float(a), float(b)) for a, b in pts], doc if isinstance(doc, dict) else {}
    if len(flat) >= 2 and len(flat) % 2 == 0:
        return [(float(flat[i]), float(flat[i + 1])) for i in range(0, len(flat), 2)], {}
    raise SystemExit("dronehacks_nav: give route_file:=<json> or route:=[f1,l1,f2,l2,...] (forward/left metres)")


class SkybetaNav(Node):
    def __init__(self):
        super().__init__("dronehacks_nav")
        route, doc = load_route(self)
        prm = C.Params(route=route)
        for k, v in vars(C.Params()).items():
            if k == "route":
                continue
            default = doc.get("params", {}).get(k, v)
            val = self.declare_parameter(k, default).value
            setattr(prm, k, type(v)(val))
        self.mission = C.Mission(prm)
        self.depth_topic = self.declare_parameter("depth_topic", "").value
        info = self.declare_parameter("depth_info_topic", "").value
        self.info_topic = info or (self.depth_topic.rsplit("/", 1)[0] + "/camera_info" if self.depth_topic else "")
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.odom_topic = self.declare_parameter("odometry_topic", "/visual_slam/tracking/odometry").value
        self.odom_stale_s = float(self.declare_parameter("odom_stale_s", 0.3).value)
        self.state_stale_s = float(self.declare_parameter("state_stale_s", 1.0).value)
        self.exit_when_done = bool(self.declare_parameter("exit_when_done", True).value)
        self.motion_on = bool(self.declare_parameter("motion_check", True).value)
        self.motion = MotionCheck(
            err_m=float(self.declare_parameter("motion_err_m", 0.3).value),
            window_s=float(self.declare_parameter("motion_window_s", 3.0).value),
            min_progress_m=float(self.declare_parameter("motion_min_progress_m", 0.1).value),
            z_mismatch_m=float(self.declare_parameter("motion_z_mismatch_m", 0.3).value))
        self.frozen = None              # (x, y, z, yaw) ENU the node holds after a motion-check failure

        from tf2_ros import Buffer, TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub_sp = self.create_publisher(PoseStamped, "/skybeta/setpoint/pose", 10)
        status_topic = self.declare_parameter("status_topic", "/refnav/status").value
        self.pub_status = self.create_publisher(String, status_topic, 10)

        from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
        from px4_msgs.msg import VehicleLandDetected
        self.v = C.Vehicle()
        self.odom_t = None
        self.vo = (None, None)                         # (vo_state, arrival time)
        self.sky = None                                # latest /skybeta/state dict
        self.sky_t = None
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)
        self.create_subscription(VisualSlamStatus, "/visual_slam/status", self._on_vo, 10)
        self.create_subscription(String, "/skybeta/state", self._on_state, 10)
        self.create_subscription(VehicleLandDetected, "/fmu/out/vehicle_land_detected", self._on_land, PX4_QOS)
        if self.depth_topic:
            self.create_subscription(CameraInfo, self.info_topic, self._on_info, qos_profile_sensor_data)
            self.create_subscription(Image, self.depth_topic, self._on_depth, qos_profile_sensor_data)

        self.K = None
        self.extrinsic = None           # (frame_id, R_body_cam, t_body_cam)
        self.rejects = {}
        self.done_at = None
        self._wait_log_t = None
        self._said = {}                 # log key -> last time said
        self.create_timer(0.05, self._tick)
        self.create_timer(0.1, self._health)
        self.create_timer(0.5, self._publish_status)
        self.get_logger().info(f"route {len(route)} legs; odometry {self.odom_topic}; "
                               f"depth {self.depth_topic or 'NONE (the mission will wait for depth and never fly)'}")

    # ------------------------------------------------------------------ inputs
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _say(self, key, text, every_s=5.0, warn=False):
        now = self._now()
        if key in self._said and now - self._said[key] < every_s:
            return
        self._said[key] = now
        (self.get_logger().warning if warn else self.get_logger().info)(text)

    def _on_odom(self, m):
        p, o = m.pose.pose.position, m.pose.pose.orientation
        self.odom_enu = (p.x, p.y, p.z)
        self.odom_yaw_enu = F.yaw_from_quat_xyzw(o.x, o.y, o.z, o.w)
        self.v.pos = F.enu_to_ned(p.x, p.y, p.z)
        self.v.heading = F.yaw_swap(self.odom_yaw_enu)
        self.v.q = F.quat_enu_flu_to_ned_frd(o.x, o.y, o.z, o.w)
        self.odom_t = self._now()

    def _on_vo(self, m):
        self.vo = (int(m.vo_state), self._now())

    def _on_state(self, m):
        try:
            self.sky = json.loads(m.data)
        except ValueError:
            return
        self.sky_t = self._now()
        if self.sky.get("clamped"):
            self._say("clamped", f"the flight interface is CLAMPING our setpoints ({', '.join(self.sky['clamped'])}) "
                                 f"to the envelope {self.sky.get('envelope')}", warn=True)

    def _on_land(self, m):
        self.v.landed = bool(m.landed)

    def _health(self):
        """Fail closed: every input the mission flies on must be present and fresh."""
        now = self._now()
        vo, vo_t = self.vo
        s = self.sky if self.sky_t is not None and now - self.sky_t <= self.state_stale_s else None
        ok = (self.odom_t is not None and now - self.odom_t <= self.odom_stale_s
              and vo == VO_TRACKING and now - vo_t <= self.state_stale_s
              and s is not None and bool(s.get("px4_link")) and s.get("local_enu") is not None)
        self.mission.on_health(now, ok)

    def _on_info(self, m):
        if m.k[0] > 0 and m.k[4] > 0:
            self.K = (m.k[0], m.k[4], m.k[2], m.k[5])

    def _reject(self, key, detail=None, record=True):
        """Count a refused depth frame by its reason; log the reason WITH its numbers at the 1st, 10th,
        100th and 1000th time; tell the mission (which reports it while it waits)."""
        detail = detail or key
        if record:
            self.mission.depth_refused(key, detail)
        self.rejects[key] = self.rejects.get(key, 0) + 1
        if self.rejects[key] in (1, 10, 100, 1000):
            self.get_logger().warning(f"depth frame not used ({self.rejects[key]}x): {detail}")

    def _calibration(self, frame_id):
        if self.extrinsic is not None and self.extrinsic[0] == frame_id:
            return self.extrinsic
        from rclpy.time import Time
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, frame_id, Time())
        except Exception as e:                      # not (yet) on the TF tree
            self._reject("no_tf", f"no TF {self.base_frame} <- {frame_id} ({type(e).__name__})")
            return None
        r, t = tf.transform.rotation, tf.transform.translation
        R = D.quat_to_mat(r.w, r.x, r.y, r.z)
        fwd = R[:, 2]                               # the optical axis in the body frame
        ang = math.degrees(math.acos(max(-1.0, min(1.0, fwd[0] / np.linalg.norm(fwd)))))
        if ang > 25.0:
            self._reject("camera_not_forward",
                         f"camera {frame_id} looks {ang:.0f} deg off the body's forward axis: this package needs a forward camera")
            return None
        self.extrinsic = (frame_id, R, np.array([t.x, t.y, t.z]))
        self.get_logger().info(f"calibration from TF: {self.base_frame} <- {frame_id} t=({t.x:.3f},{t.y:.3f},{t.z:.3f}) "
                               f"optical axis {ang:.1f} deg off forward")
        return self.extrinsic

    def _on_depth(self, m):
        if self.K is None:
            return self._reject("no_camera_info", f"no camera_info yet on {self.info_topic}")
        cal = self._calibration(m.header.frame_id)
        if cal is None:
            return None
        if self.v.q is None:
            return self._reject("no_attitude", "no odometry attitude yet")
        try:
            d = D.decode_depth(m.encoding, m.height, m.width, m.step, bytes(m.data))
        except ValueError as e:
            return self._reject("undecodable", str(e))
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if not self.mission.on_depth(stamp, d, self.K, cal[1], cal[2], self.v.q, now=self._now()):
            key, detail = self.mission.depth_reject or ("refused", "refused by the mission")
            self._reject(key, detail, record=False)
        return None

    def _vehicle(self, now):
        """The mission's view of the vehicle this tick: unknown wherever the inputs are stale."""
        v = self.v
        v.pos_ok = (self.odom_t is not None and now - self.odom_t <= self.odom_stale_s and v.pos is not None
                    and all(math.isfinite(c) for c in (*v.pos, v.heading)))
        if self.sky_t is not None and now - self.sky_t <= self.state_stale_s:
            armed, nav = self.sky.get("armed"), self.sky.get("nav_state")
            v.armed = None if armed is None else bool(armed)
            v.nav_state = None if nav is None else int(nav)
        else:
            v.armed = v.nav_state = None
        return v

    # ------------------------------------------------------------------ outputs
    def _check_motion(self, now, out):
        """Run the motion check while Offboard is ours; on a failure, freeze where the vehicle is."""
        s = self.sky if self.sky_t is not None and now - self.sky_t <= self.state_stale_s else None
        active = (self.motion_on and self.frozen is None and out.setpoint is not None
                  and self.mission.phase in C.AIRBORNE and s is not None and s.get("offboard_active")
                  and not s.get("clamped") and self.odom_t is not None and now - self.odom_t <= self.odom_stale_s)
        if not active:
            if self.frozen is None:
                self.motion.reset()                  # a clamp, a pause or a takeover starts a new window
            return
        local = s.get("local_enu")
        why = self.motion.step(now, F.ned_to_enu(*out.setpoint[:3]), self.odom_enu,
                               None if local is None else local[2])
        if why:
            self.frozen = (*self.odom_enu, self.odom_yaw_enu)
            self.get_logger().error(f"MOTION CHECK FAILED: {why}. FREEZING at ENU "
                                    f"({self.frozen[0]:.2f}, {self.frozen[1]:.2f}, {self.frozen[2]:.2f}): "
                                    f"PILOT, TAKE OVER AND LAND")
            self.mission.request_land(f"motion check: {why}")

    def _send(self, n, e, d, yaw_ned):
        if self.frozen is not None:                  # after a motion-check failure, only the freeze point
            x, y, z, yaw_enu = self.frozen
            yaw_ned = F.yaw_swap(yaw_enu)
            n, e, d = F.enu_to_ned(x, y, z)
        x, y, z = F.ned_to_enu(n, e, d)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = float(x), float(y), float(z)
        qx, qy, qz, qw = F.quat_xyzw_from_yaw(F.yaw_swap(yaw_ned))
        msg.pose.orientation.x, msg.pose.orientation.y = qx, qy
        msg.pose.orientation.z, msg.pose.orientation.w = qz, qw
        self.pub_sp.publish(msg)

    def _tick(self):
        now = self._now()
        out = self.mission.step(now, self._vehicle(now))
        for e in out.events:
            self.get_logger().info(e)
        if self.mission.phase == "wait" and self.mission.t_phase is not None:
            if self._wait_log_t is None or now - self._wait_log_t >= 10.0:   # say why it is not flying, every 10 s
                if self._wait_log_t is not None:
                    self.get_logger().info(f"not ready after {now - self.mission.t_phase:.0f} s "
                                           f"(gives up at {self.mission.p.wait_s:.0f} s): {self.mission.why}")
                self._wait_log_t = now
        self._check_motion(now, out)
        if out.events:
            self._publish_status()
        if out.stream and out.setpoint is not None:
            self._send(*out.setpoint)
        for c in out.commands:
            if c in ("offboard", "arm"):
                self._say("ready", f"READY: pilot, arm on Altitude, then switch to Position within a few seconds "
                                   f"(this mission gives up after {self.mission.p.arm_wait_s:.0f} s)")
            elif c == "land":
                self._say("land", f"MISSION ENDED ({self.mission.land_reason}): holding position. skybeta has no "
                                  f"land command: PILOT, TAKE OVER AND LAND", warn=True)
            elif c == "disarm":
                self._say("disarm", "mission ended before Offboard: pilot, disarm", warn=True)
        if out.phase == "done" and self.done_at is None:
            self.done_at = now
            self.get_logger().info(f"mission done: {self.mission.result}")

    def _publish_status(self):
        st = self.mission.status(self._now())
        st["rejects"] = dict(self.rejects)
        st["motion_check"] = ("off" if not self.motion_on else
                              f"FAILED: {self.motion.failure}" if self.frozen is not None else "ok")
        st["frozen_enu"] = None if self.frozen is None else [round(c, 3) for c in self.frozen[:3]]
        self.pub_status.publish(String(data=json.dumps(st)))
        if self.exit_when_done and self.done_at is not None and self._now() - self.done_at > 1.5:
            raise SystemExit(0 if self.mission.result == "complete" else 1)


def main(argv=None):
    rclpy.init(args=list(argv) if argv is not None else sys.argv)
    node = SkybetaNav()
    code = 1
    try:
        rclpy.spin(node)
    except SystemExit as e:
        code = int(e.code or 0)
    except KeyboardInterrupt:
        code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
