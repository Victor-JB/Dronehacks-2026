"""Read-only shadow telemetry + Stage 4 safety node. OBSERVES ONLY.

Subscribes to odometry, /skybeta/state, VO status, depth + camera_info and reads TF. It creates NO
publisher, NO service client and NO action client, and checks this itself at start-up (guard.py):
any such endpoint makes it refuse to run. Results go to the ROS log (once per log_period_s) and,
optionally, to a JSON-lines file. It never touches /skybeta/setpoint/*, /fmu/in/* or any flight service,
and it is independent of mission_node and of the organizer's flight_interface.

Not registered as a console script on purpose (keeps it out of packaged builds). Run by hand:
  python3 -m dh_autonomy.shadow_node --ros-args -p depth_topic:=/oak/stereo/image_raw

Topic defaults and their evidence (docs/STAGE4_SHADOW.md):
  odometry_topic  /visual_slam/tracking/odometry  organizer contract; in the 2026-11-07 bag (29.3 Hz)
  state_topic     /skybeta/state                  organizer contract; in the bag (5 Hz JSON)
  vo_status_topic /visual_slam/status             in the bag (29.3 Hz); message package only on the drone
  depth_topic     /oak/stereo/image_raw           main's config/drone.yaml; NOT in the bag (unverified)
  depth_info_topic /oak/stereo/camera_info        main's config/drone.yaml; in the bag (30 Hz)
"""
from __future__ import annotations

import json

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from dh_core.shadow.guard import check_endpoints
from dh_core.shadow.monitor import ShadowConfig, ShadowMonitor
from dh_core.shadow.transforms import validate_extrinsic
from refnav import depth as D


class ShadowNode(Node):
    def __init__(self):
        super().__init__("dh_shadow")
        self.declare_parameter("odometry_topic", "/visual_slam/tracking/odometry")
        self.declare_parameter("state_topic", "/skybeta/state")
        self.declare_parameter("vo_status_topic", "/visual_slam/status")
        self.declare_parameter("depth_topic", "/oak/stereo/image_raw")
        self.declare_parameter("depth_info_topic", "/oak/stereo/camera_info")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("eval_hz", 10.0)
        self.declare_parameter("log_period_s", 1.0)
        self.declare_parameter("log_path", "")
        self.declare_parameter("twist_in_child_frame", True)
        g = lambda n: self.get_parameter(n).value
        self.base_frame = g("base_frame")
        self.mon = ShadowMonitor(ShadowConfig(twist_in_child_frame=bool(g("twist_in_child_frame"))))
        self.log_period = float(g("log_period_s"))
        self.log_path = g("log_path")
        self._last_log = None
        self._log_file = open(self.log_path, "a") if self.log_path else None

        from tf2_ros import Buffer, TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Odometry, g("odometry_topic"), lambda m: self.mon.on_odom(m, self._now()), 10)
        self.create_subscription(String, g("state_topic"), lambda m: self.mon.on_state(m.data, self._now()), 10)
        self.create_subscription(CameraInfo, g("depth_info_topic"), lambda m: self.mon.on_camera_info(m, self._now()),
                                 qos_profile_sensor_data)
        self.create_subscription(Image, g("depth_topic"), self._on_depth, qos_profile_sensor_data)
        try:
            from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
        except ImportError:
            VisualSlamStatus = None
            self.get_logger().warning("isaac_ros_visual_slam_interfaces not available: no VO status, "
                                      "localization stays HOVER (require_vo_status)")
        if VisualSlamStatus is not None:
            self.create_subscription(VisualSlamStatus, g("vo_status_topic"),
                                     lambda m: self.mon.on_vo_status(int(m.vo_state), self._now()), 10)
        self.create_timer(1.0 / float(g("eval_hz")), self._tick)
        self.assert_read_only()

    def endpoint_violations(self):
        pubs = [getattr(p, "topic_name", str(p)) for p in getattr(self, "publishers", [])]
        clis = [getattr(c, "srv_name", str(c)) for c in getattr(self, "clients", [])]
        srvs = [getattr(s, "srv_name", str(s)) for s in getattr(self, "services", [])]
        return check_endpoints(pubs, clis, srvs)

    def assert_read_only(self):
        v = self.endpoint_violations()
        if v:
            raise RuntimeError("shadow node refuses to run with command-capable endpoints: %s" % v)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_depth(self, m):
        frame = m.header.frame_id
        if self.mon.extr is None or self.mon.extr.frame_id != frame:
            try:
                from rclpy.time import Time
                tf = self.tf_buffer.lookup_transform(self.base_frame, frame, Time())
                r, t = tf.transform.rotation, tf.transform.translation
                ext, why = validate_extrinsic(frame, D.quat_to_mat(r.w, r.x, r.y, r.z), [t.x, t.y, t.z])
                self.mon.set_extrinsic(ext, why)
            except Exception as e:                     # not (yet) on the TF tree
                self.mon.set_extrinsic(None, "no_tf")
        self.mon.on_depth(m, self._now())

    def _tick(self):
        now = self._now()
        rep = self.mon.evaluate(now)
        if self._last_log is None or now - self._last_log >= self.log_period:
            self._last_log = now
            self.get_logger().info("shadow %s max %.2f m/s (%s) free %s loc %s" % (
                rep["verdict"], rep["max_speed"], rep["reason"], rep["free_distance"], rep["loc_mode"]))
            if self._log_file is not None:
                self._log_file.write(json.dumps(rep, default=float) + "\n")
                self._log_file.flush()


def main():
    rclpy.init()
    node = ShadowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
