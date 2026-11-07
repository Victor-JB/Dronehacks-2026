"""DroneHacks mission node: plan from the course model, align frames, fly, land, return.

Pipeline (all generated at start-up from the course file, no hand-typed waypoints):
  course OBJ -> occupancy slice at cruise height -> clearance-aware A* (+ optional
  forced pass through the wall aperture) -> line-of-sight post-smoothing + clothoid corners ->
  time-optimal speed profile -> trajectory follower -> safety/health gating -> backend setpoints.

Topics
  sub  /visual_slam/tracking/odometry  nav_msgs/Odometry (ENU, odom frame; header.stamp used for sync)
  sub  /visual_slam/status             isaac_ros_visual_slam_interfaces/VisualSlamStatus (optional; vo_state)
  sub  /dh/start                        std_msgs/Empty   (the single start command)
  sub  /dh/map_fix                      geometry_msgs/PoseWithCovarianceStamped (absolute pose at scan stamp)
  sub  /dh/line_obs                     geometry_msgs/Vector3Stamped (lateral, heading, confidence at image stamp)
  sub  /dh/forward_clearance            sensor_msgs/Range (REP 117: +inf clear, NaN unknown, finite obstacle)
  pub  /dh/pose_map                     geometry_msgs/PoseStamped (frame "course", stamp = odometry stamp)
  pub  /dh/plan                         nav_msgs/Path (frame "course")
  pub  /dh/mission_state                std_msgs/String
  pub  /dh/health                       std_msgs/String  "<mode> fc=<state> <reason>"

Command gating (Sky Beta backend): nothing is published to the flight interface until /skybeta/state
reports offboard_active; the trajectory clock does not start before that. If Offboard is lost after
it was active (pilot takeover = abort under R5) the mission goes to ABORTED and stops commanding.
Unhealthy localization or missing/stale/invalid forward clearance -> zero-velocity setpoints with the
trajectory clock frozen (HOVER); persistent loss -> ABORTED + the interface's own /skybeta/hold.
No mode, arm, land or PX4 commands are sent by the Sky Beta backend.
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Vector3Stamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import Empty, String

from dh_core.course import Course
from dh_core.localize import raycast_scan
from dh_core.health import (ABORT, HOVER, ClearanceParams, ForwardClearanceMonitor, LocalizationHealth,
                            LocHealthParams, PoseHistory, stamp_to_sec)
from dh_core.route import RouteParams, dark_zones, plan_mission, return_leg
from dh_core.timing import time_parameterize
from dh_core.tracker import FollowerParams, MapAlignment, TrajectoryFollower, wrap

from .backends import DryRunBackend, Px4Backend, SkybetaBackend, verify_isolation

FLIGHT_STATES = ("TAKEOFF", "COURSE", "RETURN")


def quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MissionNode(Node):
    def __init__(self):
        super().__init__("dh_mission")
        P = self.declare_parameter
        P("course_obj", "/root/team_ws/course/dronehacks-course-v3.obj")
        P("dry_run", True)                      # DEFAULT ON: commands go only to /dh/dry_run/*; enforced in code
        P("backend", "skybeta")                 # skybeta | px4 (px4 = SITL only unless organizers approve)
        P("skybeta_ns", "/skybeta")             # tests remap this so they can never reach the real interface
        P("route", "window")                    # gate | window | auto
        P("cruise_z", 0.85)
        P("robot_radius", 0.27)
        P("v_max", 0.85)                        # planned speed (leave headroom below PX4 cap)
        P("v_cap", 1.0)                         # PX4 MPC_XY_VEL_MAX (do not raise without organizer approval)
        P("v_narrow", 0.5)
        P("v_dark", 0.85)
        P("a_max", 1.5)
        P("a_lat", 1.5)
        P("launch_yaw_map", math.pi / 2)        # drone heading on the launch mark (map ENU); +pi/2 = north
        P("ground_z", 0.0)
        P("enable_return", False)
        P("return_z", 2.05)
        P("takeoff_on_start", True)
        P("autostart_on_offboard", False)       # skybeta: start when the pilot hands over Offboard
        P("align_mode", "start")                # start: align on /dh/start; first_odom: align when the
                                                # stack starts with the drone on the launch mark
        P("odom_topic", "/visual_slam/tracking/odometry")
        P("odom_frame", "odom")                 # expected header.frame_id of the odometry (warned if different)
        P("vo_status_topic", "/visual_slam/status")
        P("odom_timeout", 0.4)                  # s without odometry -> hover
        P("odom_abort_s", 2.0)                  # s without odometry -> abort (interface hold)
        P("vo_lost_abort_s", 3.0)
        P("max_dark_s", 4.0)
        P("correction_gain", 0.5)
        P("corner", "clothoid")                 # fillet corners: clothoid (continuous curvature) | arc
        P("a_total", 1.5)                       # combined acceleration limit for the speed profile (0 = separate)
        P("window_straight", 0.6)               # m of straight path required each side of the window
        P("planner", "fillet")                  # fillet | sfc | mpl
        P("line_gain", 0.3)
        P("line_max_innovation", 0.20)          # m; reject line fixes further than this from the expected line
        P("min_forward_clearance", 0.45)
        P("require_forward_clearance", True)    # False only with use_stereo:=false (then no obstacle check!)
        P("fc_timeout", 0.5)
        P("fc_invalid_scale", 0.35)
        P("fc_invalid_grace_s", 1.5)
        P("fc_invalid_floor_scale", 0.2)        # creep factor after the grace period (0 = hover)
        P("px4_yaw_offset", 0.0)
        P("land_hover_z", 0.35)                 # skybeta interface floor is 0.3 m
        P("geofence_margin", 0.25)
        P("rate_hz", 30.0)
        P("disarm_after_landed_s", 0.4)
        P("post_dark_scale", 0.25)              # speed factor after the tube until a fresh map fix (1.0 = off)
        P("post_dark_timeout", 2.5)
        g = lambda n: self.get_parameter(n).value

        self.course = Course.from_obj(g("course_obj"))
        self.rp = RouteParams(route=g("route"), cruise_z=g("cruise_z"), robot_radius=g("robot_radius"),
                              v_max=g("v_max"), v_narrow=g("v_narrow"), v_dark=g("v_dark"),
                              a_max=g("a_max"), a_lat=g("a_lat"), return_z=g("return_z"), planner=g("planner"),
                              corner=g("corner"), a_total=g("a_total"), window_straight=g("window_straight"))
        self.plan = plan_mission(self.course, self.rp, with_return=g("enable_return"))
        main = self.plan.legs[0].traj
        self.get_logger().info("planned route=%s planner=%s length=%.2f m time=%.1f s lengths=%s"
                               % (self.plan.route_used, self.plan.info.get("planner"), main.length, main.duration,
                                  self.plan.info["route_lengths"]))
        self.fprm = FollowerParams(v_cap=g("v_cap"))
        self.dry_run = bool(g("dry_run"))
        if self.dry_run:
            self.backend = DryRunBackend(self)
            self.get_logger().warn("DRY RUN: no publisher to /skybeta/* or /fmu/*; commands -> /dh/dry_run/cmd_vel")
        elif g("backend") == "px4":
            self.backend = Px4Backend(self, g("px4_yaw_offset"))
        else:
            self.backend = SkybetaBackend(self, ns=g("skybeta_ns"))
        self.align = MapAlignment()
        self.odom = None                        # (p (3,), yaw, stamp_sec, receipt_sec)
        self.odom_hist = PoseHistory(span_s=4.0, max_gap_s=0.3)
        self.state = "WAIT_START"
        self.follower: Optional[TrajectoryFollower] = None
        self.state_t0 = self._now()
        self._landed_since = None
        self.last_fix_t = -1.0                  # receipt time of the last applied map fix
        self.last_fix_stamp = 0.0
        self.was_active = False                 # Offboard has been active during this flight
        self.abort_reason = ""
        self.counters = {"fix_applied": 0, "fix_rejected": 0, "line_applied": 0, "hover_ticks": 0, "commands": 0}
        self.dark = [b[:, :2] for b in dark_zones(self.course, self.rp.dark_zone_hints)]
        self.ground_z = g("ground_z")
        self.fc = ForwardClearanceMonitor(ClearanceParams(
            min_clearance=g("min_forward_clearance"), timeout_s=g("fc_timeout"), invalid_scale=g("fc_invalid_scale"),
            invalid_grace_s=g("fc_invalid_grace_s"), invalid_floor_scale=g("fc_invalid_floor_scale"),
            required=g("require_forward_clearance")))
        self.health = LocalizationHealth(LocHealthParams(
            odom_timeout_s=g("odom_timeout"), odom_abort_s=g("odom_abort_s"), vo_lost_abort_s=g("vo_lost_abort_s"),
            max_dark_s=g("max_dark_s")))
        self._warned = set()

        self.create_subscription(Odometry, g("odom_topic"), self._on_odom, qos_profile_sensor_data)
        self._sub_vo_status(g("vo_status_topic"))
        self.create_subscription(Empty, "/dh/start", lambda _: self._start(), 10)
        self.create_subscription(PoseWithCovarianceStamped, "/dh/map_fix", self._on_fix, 10)
        self.create_subscription(Vector3Stamped, "/dh/line_obs", self._on_line, 10)
        self.create_subscription(Range, "/dh/forward_clearance", self._on_clearance, qos_profile_sensor_data)
        self.pub_pose = self.create_publisher(PoseStamped, "/dh/pose_map", 10)
        self.pub_state = self.create_publisher(String, "/dh/mission_state", 10)
        self.pub_health = self.create_publisher(String, "/dh/health", 10)
        self.pub_path = self.create_publisher(Path, "/dh/plan", 1)
        self._publish_plan()
        self.timer = self.create_timer(1.0 / g("rate_hz"), self._tick)
        self.last_tick = self._now()
        self.endpoints = verify_isolation(self, self.dry_run)    # raises in dry-run if isolation is broken

    # ------------------------------------------------------------ helpers
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _warn_once(self, key, msg):
        if key not in self._warned:
            self._warned.add(key)
            self.get_logger().warn(msg)

    def _sub_vo_status(self, topic):
        try:
            from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
        except ImportError:
            self.get_logger().warn("isaac_ros_visual_slam_interfaces not importable: VO tracking status is NOT "
                                   "monitored (odometry freshness/jumps still are)")
            return
        self.create_subscription(VisualSlamStatus, topic,
                                 lambda m: self.health.on_vo_status(self._now(), int(m.vo_state)),
                                 qos_profile_sensor_data)

    def _map_range(self, pm, yaw) -> float:
        """Forward range the course map predicts (same sector as the stereo forward clearance)."""
        if pm[2] > self.course.wall_height + 0.1:
            return math.inf                       # above the walls (return leg): the map has nothing there
        pts = raycast_scan(self.plan.grid, np.array([pm[0], pm[1], yaw]), 30.0, 7, r_min=0.05, r_max=4.0, noise=0.0)
        return float(np.min(np.hypot(pts[:, 0], pts[:, 1]))) if len(pts) else math.inf

    def _in_dark(self, pm) -> bool:
        return any(lo[0] - 0.05 <= pm[0] <= hi[0] + 0.05 and lo[1] - 0.05 <= pm[1] <= hi[1] + 0.05
                   for lo, hi in self.dark)

    # ------------------------------------------------------------ callbacks
    def _on_odom(self, m: Odometry):
        now = self._now()
        t = stamp_to_sec(m.header.stamp)
        if t <= 0.0:
            self._warn_once("odom_unstamped", "odometry has no header.stamp: using receipt time; map fixes "
                                              "and line fixes cannot be time-synchronised")
            t = now
        fid = getattr(m.header, "frame_id", "")
        if fid and fid != self.get_parameter("odom_frame").value:
            self._warn_once("odom_frame", "odometry frame_id %r != expected %r" % (fid, self.get_parameter("odom_frame").value))
        if self.odom is not None and t < self.odom[2]:
            return                                             # out-of-order odometry: ignore
        p = m.pose.pose.position
        xyz = np.array([p.x, p.y, p.z])
        yaw = quat_to_yaw(m.pose.pose.orientation)
        cov = list(getattr(m.pose, "covariance", []) or [])
        pos_std = math.sqrt(max(cov[0], cov[7])) if len(cov) == 36 and max(cov[0], cov[7]) > 0 else None
        self.health.on_odom(now, t, xyz, pos_std, v_max=max(1.5, 1.5 * self.fprm.v_cap))
        self.odom = (xyz, yaw, t, now)
        self.odom_hist.add(t, (xyz[0], xyz[1], xyz[2], yaw))
        if not self.align.valid and self.get_parameter("align_mode").value == "first_odom":
            self._align_at_launch()

    def _on_clearance(self, m: Range):
        self.fc.update(stamp_to_sec(m.header.stamp), self._now(), float(m.range))

    def _align_at_launch(self):
        """The drone is sitting on the launch mark with heading `launch_yaw_map`."""
        p, yaw = self.odom[0], self.odom[1]
        launch = np.array([self.course.launch[0], self.course.launch[1], self.ground_z])
        self.align.init_from_pose(p, yaw, launch, self.get_parameter("launch_yaw_map").value)
        self.get_logger().info("aligned odom -> course frame at the launch mark")

    def _map_pose(self):
        return self.align.to_map(self.odom[0], self.odom[1])

    def _map_pose_at(self, t: float):
        """Course-frame pose at time t under the CURRENT alignment, from odometry history (None if unknown)."""
        od = self.odom_hist.at(t, future_tol=0.03)
        if od is None:
            return None
        return self.align.to_map(od[:3], od[3])

    def _on_fix(self, m: PoseWithCovarianceStamped):
        """Absolute course pose measured at the scan's capture time -> alignment correction."""
        if not self.align.valid or self.odom is None:
            return
        t = stamp_to_sec(m.header.stamp)
        if t <= 0.0 or t < self.last_fix_stamp:
            self.counters["fix_rejected"] += 1                 # unstamped or older than an applied fix
            return
        at = self._map_pose_at(t)
        if at is None:
            self.counters["fix_rejected"] += 1                 # no odometry for that time (stale / gap / clock)
            return
        pm_t, yaw_t = at
        q = m.pose.pose
        d = np.array([q.position.x - pm_t[0], q.position.y - pm_t[1]])
        dyaw = wrap(quat_to_yaw(q.orientation) - yaw_t)
        self.align.correct(d, dyaw, pm_t[:2], self.get_parameter("correction_gain").value)
        self.last_fix_stamp = t
        self.last_fix_t = self._now()
        self.health.on_map_fix(self.last_fix_t, float(np.linalg.norm(d)))
        self.counters["fix_applied"] += 1

    def _on_line(self, m: Vector3Stamped):
        """Lateral fix from the floor guideline, evaluated against the pose at the image capture time."""
        gl = self.course.guideline
        if gl is None or not self.align.valid or self.odom is None or m.vector.z < 0.5:
            return
        at = self._map_pose_at(stamp_to_sec(m.header.stamp))
        if at is None:
            return
        pm, yaw = at
        best = None
        for a, b in zip(gl[:-1], gl[1:]):
            ab = b - a
            L2 = float(ab @ ab)
            if L2 < 1e-9:
                continue
            tt = float(np.clip((pm[:2] - a) @ ab / L2, 0, 1))
            q = a + tt * ab
            d = float(np.linalg.norm(pm[:2] - q))
            if best is None or d < best[0]:
                best = (d, q, ab / math.sqrt(L2))
        if best is None or best[0] > 0.6:
            return
        _, q, u = best
        if abs(wrap(math.atan2(u[1], u[0]) - yaw)) > 0.6 and abs(wrap(math.atan2(-u[1], -u[0]) - yaw)) > 0.6:
            return   # only use the line when flying along it
        c, s = math.cos(yaw), math.sin(yaw)
        rel = q - pm[:2]
        y_exp = -s * rel[0] + c * rel[1]          # expected lateral offset (body left +)
        y_meas = float(m.vector.x)
        if abs(y_meas - y_exp) > self.get_parameter("line_max_innovation").value:
            return   # the guideline is in zone 1 (before the tube) where drift is small: a big jump is a false line
        err_body_y = -(y_meas - y_exp)             # drone is actually displaced the other way
        d_map = np.array([-s * err_body_y, c * err_body_y])
        n = np.array([-u[1], u[0]])
        d_map = (d_map @ n) * n                    # only the line-normal component is observable
        self.align.correct(d_map, 0.0, pm[:2], self.get_parameter("line_gain").value)
        self.counters["line_applied"] += 1

    # -------------------------------------------------------------- mission
    def _start(self):
        if self.state != "WAIT_START" or self.odom is None:
            self.get_logger().warn("start ignored (state=%s, odom=%s)" % (self.state, self.odom is not None))
            return
        if not self.align.valid:
            self._align_at_launch()
        self.get_logger().info("start command received")
        if self.dry_run:
            self.backend.grant_authority()
        if self.backend.can_arm and self.get_parameter("takeoff_on_start").value:
            self.backend.hold()
            self.backend.request_offboard()
            self.backend.arm()
        self._goto("TAKEOFF")

    def _goto(self, state: str):
        self.get_logger().info("state %s -> %s" % (self.state, state))
        self.state = state
        self.state_t0 = self._now()
        self.follower = None                   # (re)built when the state first gets command authority
        if state in ("ABORTED", "DONE"):
            return

    def _make_follower(self):
        pm, _ = self._map_pose()
        if self.state == "TAKEOFF":
            start = self.plan.legs[0].traj.p[0]
            return self._vertical(pm, np.array([start[0], start[1], self.rp.cruise_z]))
        if self.state == "COURSE":
            return TrajectoryFollower(self.plan.legs[0].traj, self.fprm)
        if self.state == "RETURN":
            # re-plan from where the drone actually landed; descend to just above the
            # floor and let LAND (auto-land + auto-disarm) finish the touchdown
            tr = return_leg(np.array([pm[0], pm[1], max(pm[2], 0.0)]), self.course.launch[:2], self.rp, land_z=0.4)
            return TrajectoryFollower(tr, FollowerParams(v_cap=self.fprm.v_cap, face_travel=False, finish_tol=0.15))
        return None

    def _vertical(self, a, b):
        # PX4 MPC_Z_VEL_MAX_UP is 1.0 m/s on the bench drone
        tr = time_parameterize(np.vstack([a + (b - a) * k / 20 for k in range(21)]), 0.9, 1.5, 1.0)
        return TrajectoryFollower(tr, FollowerParams(v_cap=self.fprm.v_cap, face_travel=False, finish_tol=0.12))

    def _abort(self, reason: str):
        if self.state == "ABORTED":
            return
        self.abort_reason = reason
        self.get_logger().error("ABORT: %s -> no further commands%s" % (
            reason, "; requesting interface hold" if self.backend.offboard_active() else ""))
        if self.backend.offboard_active():
            self.backend.hold()                # the interface's own freeze-at-last-safe-position
        self._goto("ABORTED")

    def _send(self, v, yr):
        vo = self.align.vec_to_odom(v)
        self.backend.send_velocity(vo[0], vo[1], vo[2], yr)
        self.counters["commands"] += 1

    def _tick(self):
        now = self._now()
        dt = max(0.0, now - self.last_tick)
        self.last_tick = now
        self.backend.tick()
        self.pub_state.publish(String(data=self.state))
        if self.odom is not None and self.align.valid and now - self.odom[3] <= self.get_parameter("odom_timeout").value:
            pm0, yaw0 = self._map_pose()
            self._publish_pose(pm0, yaw0)      # also while waiting: stationary localization checks + localizer prior
        if self.odom is None or self.state in ("WAIT_START", "ABORTED", "DONE"):
            if self.state == "WAIT_START" and self.odom is not None and \
                    self.get_parameter("autostart_on_offboard").value and self.backend.offboard_active():
                self._start()
            return
        if self.state in ("LAND", "RETURN_LAND", "LANDED") and self.backend.can_land:
            # raw-PX4 landing runs in PX4's own LAND mode (not Offboard): no Offboard gating here
            pm, _ = self._map_pose()
            if self.state == "LANDED":
                self._landed_tick(now)
            else:
                self._land_step(pm)
            return
        active_state = self.state in FLIGHT_STATES or self.state in ("LAND", "RETURN_LAND")

        # ---- arming (raw PX4 backend only; SITL) ------------------------------------------------
        if self.state in ("TAKEOFF", "RETURN") and self.backend.can_arm and self.backend.armed() is False:
            # (re)arm for takeoff / the return: retry every 0.5 s, give up after 6 s
            # (a failed return never costs the main run - rule 7.10)
            if now - self.state_t0 > 6.0:
                self.get_logger().error("could not arm for %s; stopping" % self.state)
                self._goto("DONE")
                return
            if now - getattr(self, "_last_arm_try", 0.0) > 0.5:
                self._last_arm_try = now
                self.backend.hold()
                self.backend.request_offboard()
                self.backend.arm()
            self.backend.send_velocity(0.0, 0.0, 0.0, 0.0)     # PX4 needs a setpoint stream before Offboard
            return

        # ---- command authority ---------------------------------------------------------------------
        if active_state and not self.backend.offboard_active():
            if self.backend.requires_setpoint_stream:            # raw PX4 (SITL): stream zeros, ask for Offboard
                if now - getattr(self, "_last_off_try", 0.0) > 0.5:
                    self._last_off_try = now
                    self.backend.request_offboard()
                self.backend.send_velocity(0.0, 0.0, 0.0, 0.0)
                return
            if self.was_active and self.state in FLIGHT_STATES + ("LAND",):
                # Sky Beta: Offboard was active and is now gone (pilot took over / interface left Offboard).
                # Under R5 human input other than abort ends the run: never resume on our own.
                self._abort("Offboard lost during %s" % self.state)
                return
            self.get_logger().warn("waiting for Offboard before commanding (no setpoints published)",
                                   throttle_duration_sec=2.0)
            return                                                # Sky Beta: publish NOTHING before Offboard
        if active_state and not self.was_active:
            self.was_active = True
            self.get_logger().info("Offboard active: taking command authority")

        # ---- health -------------------------------------------------------------------------------
        pm, yaw = self._map_pose()
        in_dark = self._in_dark(pm)
        self.health.set_in_dark(now, in_dark and self.state == "COURSE")
        mode, h_scale, reason = self.health.evaluate(now)
        fc_scale, fc_state = self.fc.speed_scale(now, expected_blind=in_dark, expected_range=self._map_range(pm, yaw))
        self.pub_health.publish(String(data="%s fc=%s %s" % (mode, fc_state, reason)))
        if mode == ABORT:
            self._abort(reason)
            return
        if mode == HOVER:
            self.counters["hover_ticks"] += 1
            self.get_logger().warn("HOVER: " + reason, throttle_duration_sec=1.0)
            self._send(np.zeros(3), 0.0)                          # explicit zero velocity; clock frozen
            return

        if self.state in FLIGHT_STATES:
            if self.follower is None:
                self.follower = self._make_follower()
            blackout = self._blackout_governor(pm, now) if self.state == "COURSE" else 1.0
            clock_scale = h_scale * blackout * (fc_scale if self.state != "TAKEOFF" else 1.0)
            self.follower.speed_scale = clock_scale
            v, yr, p_ref, err = self.follower.step(pm, yaw, dt)
            v = self._safety(v, pm, fc_scale)
            self._send(v, yr)
            if self.follower.done:
                nxt = {"TAKEOFF": "COURSE", "COURSE": "LAND", "RETURN": "RETURN_LAND"}[self.state]
                self._goto(nxt)
        elif self.state in ("LAND", "RETURN_LAND"):
            self._land_step(pm, fc_scale)
        elif self.state == "LANDED":
            self._landed_tick(now)

    def _landed_tick(self, now):
        armed = self.backend.armed()
        if armed is False and self.get_parameter("enable_return").value and self.backend.can_arm \
                and len(self.plan.legs) > 1 and now - self.state_t0 > 1.0:
            self.backend.hold()
            self.backend.request_offboard()
            self.backend.arm()
            self.was_active = False               # authority is re-acquired for the return leg
            self._goto("RETURN")
        elif now - self.state_t0 > 3.0 and not self.get_parameter("enable_return").value:
            self._goto("DONE")

    def _blackout_governor(self, pm, now) -> float:
        """After leaving a dark zone, creep until the localizer delivers a fresh fix (returns a clock factor).

        Inside the tube visual odometry coasts; the first metres after the exit are
        where an uncorrected offset would put the drone into the chicane walls.
        Disabled when post_dark_scale >= 1 or no localizer is running.
        """
        if self._in_dark(pm):
            self._dark_exit_t = None
            self._was_dark = True
            return 1.0
        if getattr(self, "_was_dark", False):
            self._was_dark = False
            self._dark_exit_t = now
        t_exit = getattr(self, "_dark_exit_t", None)
        scale = self.get_parameter("post_dark_scale").value
        if t_exit is not None and scale < 1.0:
            fixed = self.last_fix_t > t_exit
            if fixed or now - t_exit > self.get_parameter("post_dark_timeout").value:
                self._dark_exit_t = None
                return 1.0
            return scale
        return 1.0

    def _land_step(self, pm, fc_scale: float = 1.0):
        now = self._now()
        if self.backend.can_land:
            if now - self.state_t0 < 0.2:
                self.backend.land()
            landed = bool(self.backend.landed())
            if landed and self._landed_since is None:
                self._landed_since = now
            elif not landed:
                self._landed_since = None
            # The scored clock stops at disarm (R7.5): disarm as soon as the land
            # detector is confident instead of waiting for COM_DISARM_LAND (2 s).
            if landed and self.backend.armed() and now - self._landed_since > \
                    self.get_parameter("disarm_after_landed_s").value:
                self.backend.disarm()
            if landed and self.backend.armed() is False:
                self._landed_since = None
                self._goto("LANDED" if self.state == "LAND" else "DONE")
        else:
            # Organizer interface cannot land: hover low over the pad centre and wait.
            target = np.array([self.course.pad_center[0], self.course.pad_center[1],
                               self.get_parameter("land_hover_z").value])
            v = 1.0 * (target - pm)
            n = math.hypot(v[0], v[1])
            if n > 0.4:
                v[:2] *= 0.4 / n
            v[:2] *= fc_scale
            v[2] = float(np.clip(v[2], -0.4, 0.4))
            self._send(v, 0.0)
            self.get_logger().warn("backend cannot land autonomously; holding over pad", throttle_duration_sec=2.0)

    def _safety(self, v, pm, fc_scale: float = 1.0):
        v = v.copy()
        if self.state != "TAKEOFF":
            v[:2] *= fc_scale                       # forward-clearance governor (stereo depth validity aware)
        # software geofence inside the net + altitude ceiling below the walls (except return)
        m = self.get_parameter("geofence_margin").value
        lo, hi = self.course.bounds_min[:2] + m, self.course.bounds_max[:2] - m
        for k in range(2):
            if (pm[k] < lo[k] and v[k] < 0) or (pm[k] > hi[k] and v[k] > 0):
                v[k] = 0.0
        ceiling = self.course.wall_height - 0.30 if self.state != "RETURN" else self.rp.return_z + 0.2
        if pm[2] > ceiling and v[2] > 0:
            v[2] = -0.2
        return v

    # -------------------------------------------------------------- output
    def _publish_pose(self, pm, yaw):
        m = PoseStamped()
        sec = self.odom[2]
        m.header.stamp.sec = int(sec)
        m.header.stamp.nanosec = int(round((sec - int(sec)) * 1e9)) % 1000000000
        m.header.frame_id = "course"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, pm)
        m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.pub_pose.publish(m)

    def _publish_plan(self):
        path = Path()
        path.header.frame_id = "course"
        for leg in self.plan.legs:
            for q in leg.traj.p[::5]:
                ps = PoseStamped()
                ps.header.frame_id = "course"
                ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, q)
                path.poses.append(ps)
        self.pub_path.publish(path)


def main():
    rclpy.init()
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop commanding: the Sky Beta twist expires after 1 s and the interface holds. Ask for its hold
        # explicitly only if we had command authority.
        try:
            if node.was_active and node.state not in ("ABORTED", "DONE") and node.backend.offboard_active():
                node.backend.hold()
        except Exception:  # noqa: BLE001 - best effort during shutdown
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
