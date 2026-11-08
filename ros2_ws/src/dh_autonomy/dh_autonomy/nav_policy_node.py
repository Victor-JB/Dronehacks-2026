"""nav_policy: CAD-free exploration policy on the organizers' interface (autonomy_pipeline.md).

  sensors -> ShadowMonitor (validated odometry/depth/VO/state, TF extrinsic, freshness, clock checks)
          -> Explorer (online map over main's VoxelMap, frontiers, dead-end memory, A* + clothoid,
             TrajectoryFollower, opening verification, optional blue-square tracking)
          -> CommandSafety (Stage 4 SafetyGate on the FINAL velocity: stopping distance, body clearance,
             unknown = blocked, stale data, localization, flight band, geofence)
          -> backend (SkybetaBackend: the ONLY publisher of /skybeta/setpoint/*; DryRunBackend by default)

Phases (10 Hz):
  WAIT      before Offboard: a zero twist every tick (the organizer's "setpoint in the last second";
            SkybetaBackend allows only a zero twist before authority). Nothing else is sent.
  TAKEOFF   Offboard active and preflight OK: a pose at the start xy and cruise height until reached.
            Preflight NOT OK at Offboard: zero twist only (stays on the ground); the reason is logged.
  NAV       Explorer intent -> CommandSafety -> world-frame twist (ENU, angular.z = yaw rate).
  HOLD      no route / over the target / blocked: pose hold at the current position (VIO healthy).
  LOC_LOST  zero twist for brake_s, then STOP COMMANDING: flight_interface keeps its own hold and the pilot
            takes over. Position hold is NOT trusted without valid localization; this is logged loudly.
  YIELDED   Offboard ended after it was active (pilot or failsafe): never resume; reset only after the
            vehicle is reported disarmed (one go per arming, as the organizers specify).
Descent/landing is DISABLED in this policy: over the target it hovers and reports.

Dry run is the default (dry_run:=true): commands go to /dh/dry_run/cmd_vel and nothing on /skybeta or /fmu
is created. Real commanding needs dry_run:=false AND a configured geofence.
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Empty, String

from dh_core.nav.explorer import Explorer, ExplorerParams
from dh_core.nav.online_map import MapParams
from dh_core.nav.safety3d import CommandSafety, CommandSafetyParams
from dh_core.safety_filter import SafetyParams
from dh_core.shadow.monitor import ShadowConfig, ShadowMonitor
from dh_core.shadow.observation import stamp_of
from dh_core.shadow.transforms import validate_extrinsic
from refnav import depth as D

from .backends import DryRunBackend, SkybetaBackend, verify_isolation


def decode_rgb(m):
    """sensor_msgs/Image bgr8/rgb8 -> (H, W, 3) uint8 RGB, or None."""
    enc = str(m.encoding).lower()
    if enc not in ("rgb8", "bgr8"):
        return None
    a = np.frombuffer(bytes(m.data), np.uint8).reshape(int(m.height), int(m.step))[:, :int(m.width) * 3]
    a = a.reshape(int(m.height), int(m.width), 3)
    return a[..., ::-1].copy() if enc == "bgr8" else a.copy()


class NavPolicyNode(Node):
    def __init__(self):
        super().__init__("nav_policy")
        d = self.declare_parameter
        d("dry_run", True)
        d("odometry_topic", "/visual_slam/tracking/odometry")
        d("state_topic", "/skybeta/state")
        d("vo_status_topic", "/visual_slam/status")
        d("depth_topic", "/oak/stereo/image_raw")          # main's config/drone.yaml
        d("depth_info_topic", "/oak/stereo/camera_info")
        d("rgb_topic", "")                                  # NONE on the drone today (see docs); '' disables the target
        d("base_frame", "base_link")
        d("rate_hz", 10.0)
        d("geofence", [0.0])                                # [xmin, xmax, ymin, ymax] ENU odom; anything else = unset
        d("cruise_z", 0.75)
        d("v_max", 0.35)                                    # conservative: below the 0.6 the Stage 2/3 mission flies
        d("map_lo_xy", [-7.0, -7.0])
        d("map_size", 14.0)
        d("takeoff_tol", 0.10)
        d("brake_s", 1.5)
        d("brake_decel", 1.0)                               # PLACEHOLDER: measure on the drone
        d("latency_s", 0.8)                                 # PLACEHOLDER: measure on the drone
        g = lambda n: self.get_parameter(n).value
        gf = list(g("geofence"))
        self.geofence = tuple(float(v) for v in gf) if len(gf) == 4 and gf[0] < gf[1] and gf[2] < gf[3] else None
        lo = list(g("map_lo_xy"))
        self.mprm = MapParams(lo_xy=(float(lo[0]), float(lo[1])), size_xy=float(g("map_size")), cruise_z=float(g("cruise_z")),
                              stride=4)
        self.eprm = ExplorerParams(v_explore=float(g("v_max")))
        self.ex = Explorer(self.mprm, self.eprm)
        sp = SafetyParams(v_max=float(g("v_max")), brake_decel=float(g("brake_decel")), latency_s=float(g("latency_s")))
        self.safety = CommandSafety(CommandSafetyParams(safety=sp, band=self.mprm.band, robot_radius=self.mprm.robot_radius,
                                                        geofence=self.geofence))
        self.mon = ShadowMonitor(ShadowConfig())
        self.dry = bool(g("dry_run"))
        self.backend = DryRunBackend(self) if self.dry else SkybetaBackend(self, g("state_topic").rsplit("/", 1)[0])
        self.base_frame = g("base_frame")
        self.takeoff_tol, self.brake_s = float(g("takeoff_tol")), float(g("brake_s"))
        self.phase, self.t_phase = "WAIT", 0.0
        self.start = None
        self.was_active = False
        self.last_verdict = None
        self.rgb = None
        self._last_depth_used = None
        self._said = {}

        from tf2_ros import Buffer, TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, g("odometry_topic"), lambda m: self.mon.on_odom(m, self._now()), 10)
        self.create_subscription(String, g("state_topic"), lambda m: self.mon.on_state(m.data, self._now()), 10)
        self.create_subscription(CameraInfo, g("depth_info_topic"), lambda m: self.mon.on_camera_info(m, self._now()),
                                 qos_profile_sensor_data)
        self.create_subscription(Image, g("depth_topic"), self._on_depth, qos_profile_sensor_data)
        if g("rgb_topic"):
            self.create_subscription(Image, g("rgb_topic"), self._on_rgb, qos_profile_sensor_data)
        try:
            from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
            self.create_subscription(VisualSlamStatus, g("vo_status_topic"),
                                     lambda m: self.mon.on_vo_status(int(m.vo_state), self._now()), 10)
        except ImportError:
            self.get_logger().warning("no isaac_ros_visual_slam_interfaces: VO status unavailable, localization stays HOVER")
        if self.dry:   # dry run: simulated authority after /dh/start, exactly as mission_node does
            self.create_subscription(Empty, "/dh/start", lambda _m: self.backend.grant_authority(), 10)
        self.create_timer(1.0 / float(g("rate_hz")), self._tick)
        verify_isolation(self, self.dry)                    # raises if a dry run could reach /skybeta or /fmu
        self.get_logger().info("nav_policy %s; geofence %s; v_max %.2f m/s; rgb %s" % (
            "DRY RUN" if self.dry else "LIVE (SkybetaBackend)", self.geofence, float(g("v_max")), g("rgb_topic") or "off"))

    # ------------------------------------------------------------------ inputs
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_rgb(self, m):
        img = decode_rgb(m)
        if img is not None:
            self.rgb = (stamp_of(m), str(m.header.frame_id), img)

    def _on_depth(self, m):
        frame = m.header.frame_id
        if self.mon.extr is None or self.mon.extr.frame_id != frame:
            try:
                from rclpy.time import Time
                tf = self.tf_buffer.lookup_transform(self.base_frame, frame, Time())
                r, t = tf.transform.rotation, tf.transform.translation
                ext, why = validate_extrinsic(frame, D.quat_to_mat(r.w, r.x, r.y, r.z), [t.x, t.y, t.z])
                self.mon.set_extrinsic(ext, why)
            except Exception:
                self.mon.set_extrinsic(None, "no_tf")
        now = self._now()
        self.mon.on_depth(m, now)
        self._integrate(now)

    def _integrate(self, now):
        """Map the newest ACCEPTED depth frame at the odometry pose nearest its capture stamp."""
        mon = self.mon
        if mon.depth_status != "OK" or mon.depth_obs is None or mon.depth_obs is self._last_depth_used:
            return
        dep = mon.depth_obs
        self._last_depth_used = dep
        pose = min(mon.odom_hist, key=lambda o: abs(o.stamp - dep.stamp), default=None)
        loc_mode = mon.loc.evaluate(now)[0]
        if pose is None or loc_mode not in ("OK", "DEGRADED"):
            return
        R_wb = pose.R
        R_wc = R_wb @ mon.extr.R_body_cam
        o_w = pose.pos + R_wb @ mon.extr.t_body_cam
        rgb = None
        if self.rgb is not None and self.rgb[0] is not None and abs(self.rgb[0] - dep.stamp) < 0.05 \
                and self.rgb[1] == dep.frame_id and self.rgb[2].shape[:2] == dep.depth_m.shape:
            rgb = self.rgb[2]                                # only an aligned, same-size, synchronous image
        self.ex.on_frame(now, dep.depth_m, rgb, mon.intr.K, R_wc, o_w, True)

    # ------------------------------------------------------------------ helpers
    def _say(self, key, text, every=5.0, warn=False):
        now = self._now()
        if now - self._said.get(key, -1e9) >= every:
            self._said[key] = now
            (self.get_logger().warning if warn else self.get_logger().info)(text)

    def _set(self, phase, why=""):
        if phase != self.phase:
            self.get_logger().info("phase %s -> %s %s" % (self.phase, phase, why))
            self.phase, self.t_phase = phase, self._now()

    def preflight(self, now):
        """Reasons the policy may not navigate (empty = OK)."""
        r = []
        if self.geofence is None:
            r.append("geofence not configured")
        o = self.mon.odom
        if o is None or not o.valid or now - o.rx > self.mon.cfg.safety.odom_timeout_s:
            r.append("odometry missing/stale")
        mode = self.mon.loc.evaluate(now)[0]
        if mode not in ("OK", "DEGRADED"):
            r.append("localization %s" % mode)
        rx = self.mon.clear_meta.get("rx") if self.mon.clear is not None else None
        if self.mon.depth_status != "OK" or rx is None or now - rx > self.mon.cfg.safety.depth_timeout_s:
            r.append("depth %s" % ("stale" if self.mon.depth_status == "OK" else self.mon.depth_status))
        return r

    def _armed(self):
        st = self.mon.state
        return None if st is None else st.armed

    # ------------------------------------------------------------------ control loop
    def _tick(self):
        now = self._now()
        active = self.backend.offboard_active()
        loc_mode = self.mon.loc.evaluate(now)[0]
        o = self.mon.odom
        if self.phase == "YIELDED":
            if self._armed() is False:
                self._set("WAIT", "disarmed: ready for the next arming")
                self.ex = Explorer(self.mprm, self.eprm)
                self.was_active, self.start = False, None
            return                                          # never command after a takeover
        if not active:
            if self.was_active:
                self._set("YIELDED", "Offboard ended (pilot or failsafe): mission stopped until disarm")
                return
            self._set("WAIT")
            self.backend.send_velocity(0.0, 0.0, 0.0, 0.0)  # keeps a fresh setpoint for the takeover
            return
        self.was_active = True
        if loc_mode not in ("OK", "DEGRADED") or o is None:
            if self.phase != "LOC_LOST":
                self._set("LOC_LOST", "localization %s: braking, then pilot" % loc_mode)
            if now - self.t_phase <= self.brake_s:
                self.backend.send_velocity(0.0, 0.0, 0.0, 0.0)
            self._say("loc", "LOCALIZATION LOST: not commanding; PILOT, TAKE OVER", warn=True)
            return
        if self.phase == "LOC_LOST":
            self._say("loc2", "localization back, but the mission stays stopped: pilot has control", warn=True)
            return                                          # no automatic resumption after a loss
        if self.phase == "WAIT":
            why = self.preflight(now)
            if why:
                self._say("pre", "Offboard active but NOT navigating: %s (zero twist only)" % "; ".join(why), warn=True)
                self.backend.send_velocity(0.0, 0.0, 0.0, 0.0)
                return
            self.start = (float(o.pos[0]), float(o.pos[1]), float(o.yaw))
            self._set("TAKEOFF")
        if self.phase == "TAKEOFF":
            z = self.mprm.cruise_z
            self.backend.send_position(self.start[0], self.start[1], z, self.start[2])
            if abs(o.pos[2] - z) < self.takeoff_tol:
                self._set("NAV")
            return
        # NAV / HOLD
        intent = self.ex.step(now, o.pos, o.yaw, o.vel_world, True)
        if intent.state in ("OVER_TARGET", "HOLD") and float(np.linalg.norm(intent.velocity[:2])) < 1e-6:
            self._set("HOLD", intent.note)
            self.backend.send_position(float(o.pos[0]), float(o.pos[1]), self.mprm.cruise_z, None)
            return
        self._set("NAV")
        cls = self.ex.map.classify()
        grid = self.ex.map.planning_grid(cls)
        dist = grid.distance()
        c = self.mon.clear_meta
        v, yr, verdict = self.safety.filter(intent.velocity, intent.yaw_rate, o.pos, now, dist, grid,
                                            o.stamp, o.rx, c.get("stamp"), c.get("rx"), loc_mode,
                                            pos_std=o.pos_std)
        self.last_verdict = verdict
        self.backend.send_velocity(float(v[0]), float(v[1]), float(v[2]), float(yr))


def main():
    rclpy.init()
    node = NavPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
