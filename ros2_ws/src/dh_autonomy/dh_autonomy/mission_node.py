"""DroneHacks mission node: plan from the course model, align frames, fly, land, return.

Pipeline (all generated at start-up from the course file, no hand-typed waypoints):
  course OBJ -> occupancy slice at cruise height -> clearance-aware A* (+ optional
  forced pass through the wall aperture) -> shortcut + corner fillets ->
  time-optimal speed profile -> trajectory follower -> backend setpoints.

Topics
  sub  /visual_slam/tracking/odometry  nav_msgs/Odometry (ENU, odom frame)
  sub  /dh/start                        std_msgs/Empty   (the single start command)
  sub  /dh/map_correction               geometry_msgs/Vector3Stamped (dx, dy, dyaw) from localizer
  sub  /dh/line_obs                     std_msgs/Float32MultiArray [lateral_m, heading_rad, confidence]
  sub  /dh/forward_clearance            std_msgs/Float32 (m) from stereo_scan_node
  pub  /dh/pose_map                     geometry_msgs/PoseStamped (frame "course")
  pub  /dh/plan                         nav_msgs/Path (frame "course")
  pub  /dh/mission_state                std_msgs/String
"""
from __future__ import annotations

import math
import time
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Empty, Float32, Float32MultiArray, String

from dh_core.course import Course
from dh_core.route import RouteParams, dark_zones, plan_mission, return_leg
from dh_core.timing import time_parameterize
from dh_core.tracker import FollowerParams, MapAlignment, TrajectoryFollower, wrap

from .backends import Px4Backend, SkybetaBackend


def quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MissionNode(Node):
    def __init__(self):
        super().__init__("dh_mission")
        P = self.declare_parameter
        P("course_obj", "/root/team_ws/course/dronehacks-course-v3.obj")
        P("backend", "skybeta")                 # skybeta | px4
        P("route", "window")                    # gate | window | auto
        P("cruise_z", 0.85)
        P("robot_radius", 0.27)
        P("v_max", 0.85)                        # planned speed (leave headroom below PX4 cap)
        P("v_cap", 1.0)                         # PX4 MPC_XY_VEL_MAX
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
        P("odom_timeout", 0.4)
        P("correction_gain", 0.5)
        P("corner", "clothoid")              # fillet corners: clothoid (continuous curvature) | arc
        P("a_total", 1.5)                    # combined acceleration limit for the speed profile (0 = separate)
        P("window_straight", 0.6)            # m of straight path required each side of the window
        P("planner", "fillet")            # fillet | sfc (safe flight corridor + min-jerk QP, Liu et al. RA-L 2017)
        P("line_gain", 0.3)
        P("line_max_innovation", 0.20)   # m; reject line fixes further than this from the expected line
        P("min_forward_clearance", 0.45)
        P("px4_yaw_offset", 0.0)
        P("land_hover_z", 0.35)                 # skybeta interface floor is 0.3 m
        P("geofence_margin", 0.25)
        P("rate_hz", 30.0)
        P("disarm_after_landed_s", 0.4)
        P("post_dark_scale", 0.25)           # speed factor after the tube until a fresh map fix (1.0 = off)
        P("post_dark_timeout", 2.5)
        g = lambda n: self.get_parameter(n).value

        self.course = Course.from_obj(g("course_obj"))
        self.rp = RouteParams(route=g("route"), cruise_z=g("cruise_z"), robot_radius=g("robot_radius"),
                              v_max=g("v_max"), v_narrow=g("v_narrow"), v_dark=g("v_dark"),
                              a_max=g("a_max"), a_lat=g("a_lat"), return_z=g("return_z"), planner=g("planner"),
                              corner=g("corner"), a_total=g("a_total"), window_straight=g("window_straight"))
        t0 = time.time()
        self.plan = plan_mission(self.course, self.rp, with_return=g("enable_return"))
        main = self.plan.legs[0].traj
        self.get_logger().info("planned route=%s planner=%s length=%.2f m time=%.1f s (%.1fs to plan) lengths=%s"
                               % (self.plan.route_used, self.plan.info.get("planner"), main.length, main.duration, time.time() - t0,
                                  self.plan.info["route_lengths"]))
        self.fprm = FollowerParams(v_cap=g("v_cap"))
        self.backend = (Px4Backend(self, g("px4_yaw_offset")) if g("backend") == "px4" else SkybetaBackend(self))
        self.align = MapAlignment()
        self.odom = None              # (p (3,), yaw, stamp_sec)
        self.state = "WAIT_START"
        self.follower: Optional[TrajectoryFollower] = None
        self.state_t0 = time.time()
        self.fwd_clear = math.inf
        self._landed_since = None
        self.last_fix_t = -1.0
        self.dark = [b[:, :2] for b in dark_zones(self.course, self.rp.dark_zone_hints)]
        self.ground_z = g("ground_z")

        self.create_subscription(Odometry, g("odom_topic"), self._on_odom, 10)
        self.create_subscription(Empty, "/dh/start", lambda _: self._start(), 10)
        self.create_subscription(Vector3Stamped, "/dh/map_correction", self._on_correction, 10)
        self.create_subscription(Float32MultiArray, "/dh/line_obs", self._on_line, 10)
        self.create_subscription(Float32, "/dh/forward_clearance", lambda m: setattr(self, "fwd_clear", m.data), 10)
        self.pub_pose = self.create_publisher(PoseStamped, "/dh/pose_map", 10)
        self.pub_state = self.create_publisher(String, "/dh/mission_state", 10)
        self.pub_path = self.create_publisher(Path, "/dh/plan", 1)
        self._publish_plan()
        self.timer = self.create_timer(1.0 / g("rate_hz"), self._tick)
        self.last_tick = time.time()

    # ------------------------------------------------------------ callbacks
    def _on_odom(self, m: Odometry):
        p = m.pose.pose.position
        self.odom = (np.array([p.x, p.y, p.z]), quat_to_yaw(m.pose.pose.orientation), time.time())
        if not self.align.valid and self.get_parameter("align_mode").value == "first_odom":
            self._align_at_launch()

    def _align_at_launch(self):
        """The drone is sitting on the launch mark with heading `launch_yaw_map`."""
        p, yaw, _ = self.odom
        launch = np.array([self.course.launch[0], self.course.launch[1], self.ground_z])
        self.align.init_from_pose(p, yaw, launch, self.get_parameter("launch_yaw_map").value)
        self.get_logger().info("aligned odom -> course frame at the launch mark")

    def _map_pose(self):
        p, yaw, _ = self.odom
        return self.align.to_map(p, yaw)

    def _on_correction(self, m: Vector3Stamped):
        if not self.align.valid or self.odom is None:
            return
        pm, _ = self._map_pose()
        self.align.correct((m.vector.x, m.vector.y), m.vector.z, pm[:2], self.get_parameter("correction_gain").value)
        self.last_fix_t = time.time()

    def _on_line(self, m: Float32MultiArray):
        """Lateral fix from the floor guideline: compare the measured line offset with the map."""
        gl = self.course.guideline
        if gl is None or not self.align.valid or self.odom is None or len(m.data) < 3 or m.data[2] < 0.5:
            return
        pm, yaw = self._map_pose()
        best = None
        for a, b in zip(gl[:-1], gl[1:]):
            ab = b - a
            L2 = float(ab @ ab)
            if L2 < 1e-9:
                continue
            t = float(np.clip((pm[:2] - a) @ ab / L2, 0, 1))
            q = a + t * ab
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
        y_meas = float(m.data[0])
        if abs(y_meas - y_exp) > self.get_parameter("line_max_innovation").value:
            return   # the guideline is in zone 1 (before the tube) where drift is small: a big jump is a false line
        err_body_y = -(y_meas - y_exp)             # drone is actually displaced the other way
        d_map = np.array([-s * err_body_y, c * err_body_y])
        n = np.array([-u[1], u[0]])
        d_map = (d_map @ n) * n                    # only the line-normal component is observable
        self.align.correct(d_map, 0.0, pm[:2], self.get_parameter("line_gain").value)

    # -------------------------------------------------------------- mission
    def _start(self):
        if self.state != "WAIT_START" or self.odom is None:
            self.get_logger().warn("start ignored (state=%s, odom=%s)" % (self.state, self.odom is not None))
            return
        if not self.align.valid:
            self._align_at_launch()
        self.get_logger().info("start command received")
        if self.backend.can_arm and self.get_parameter("takeoff_on_start").value:
            self.backend.hold()
            self.backend.request_offboard()
            self.backend.arm()
        self._goto("TAKEOFF")

    def _goto(self, state: str):
        self.get_logger().info("state %s -> %s" % (self.state, state))
        self.state = state
        self.state_t0 = time.time()
        if state == "TAKEOFF":
            pm, _ = self._map_pose()
            start = self.plan.legs[0].traj.p[0]
            self.follower = self._vertical(pm, np.array([start[0], start[1], self.rp.cruise_z]))
        elif state == "COURSE":
            self.follower = TrajectoryFollower(self.plan.legs[0].traj, self.fprm)
        elif state == "RETURN":
            # re-plan from where the drone actually landed; descend to just above the
            # floor and let LAND (auto-land + auto-disarm) finish the touchdown
            pm, _ = self._map_pose()
            tr = return_leg(np.array([pm[0], pm[1], max(pm[2], 0.0)]), self.course.launch[:2], self.rp, land_z=0.4)
            self.follower = TrajectoryFollower(tr, FollowerParams(v_cap=self.fprm.v_cap, face_travel=False,
                                                                  finish_tol=0.15))

    def _vertical(self, a, b):
        # PX4 MPC_Z_VEL_MAX_UP is 1.0 m/s on the bench drone
        tr = time_parameterize(np.vstack([a + (b - a) * k / 20 for k in range(21)]), 0.9, 1.5, 1.0)
        return TrajectoryFollower(tr, FollowerParams(v_cap=self.fprm.v_cap, face_travel=False, finish_tol=0.12))

    def _tick(self):
        now = time.time()
        dt = now - self.last_tick
        self.last_tick = now
        self.backend.tick()
        self.pub_state.publish(String(data=self.state))
        if self.odom is None:
            return
        if self.state == "WAIT_START":
            if self.get_parameter("autostart_on_offboard").value and self.backend.offboard_active():
                self._start()
            return
        if now - self.odom[2] > self.get_parameter("odom_timeout").value:
            self.backend.hold()
            self.get_logger().warn("odometry stale -> hold", throttle_duration_sec=1.0)
            return
        pm, yaw = self._map_pose()
        self._publish_pose(pm, yaw)
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
            self.backend.send_velocity(0.0, 0.0, 0.0, 0.0)
            return
        if self.state in ("TAKEOFF", "COURSE", "RETURN") and not self.backend.offboard_active():
            # Flight-interface contract: do not command until Offboard is active. The trajectory clock is
            # frozen (the follower is not stepped) so the reference cannot run ahead of the drone.
            if self.backend.can_arm and now - getattr(self, "_last_off_try", 0.0) > 0.5:
                self._last_off_try = now
                self.backend.request_offboard()           # px4 backend only; skybeta waits for the pilot/organizer
            self.backend.send_velocity(0.0, 0.0, 0.0, 0.0)
            self.get_logger().warn("waiting for Offboard before commanding", throttle_duration_sec=2.0)
            return
        if self.state == "COURSE":
            self._blackout_governor(pm, now)
        if self.state in ("TAKEOFF", "COURSE", "RETURN"):
            v, yr, p_ref, err = self.follower.step(pm, yaw, dt)
            v = self._safety(v, pm)
            vo = self.align.vec_to_odom(v)
            self.backend.send_velocity(vo[0], vo[1], vo[2], yr)
            if self.follower.done:
                nxt = {"TAKEOFF": "COURSE", "COURSE": "LAND", "RETURN": "RETURN_LAND"}[self.state]
                self._goto(nxt)
        elif self.state in ("LAND", "RETURN_LAND"):
            self._land_step(pm)
        elif self.state == "LANDED":
            armed = self.backend.armed()
            if armed is False and self.get_parameter("enable_return").value and self.backend.can_arm \
                    and len(self.plan.legs) > 1 and now - self.state_t0 > 1.0:
                self.backend.hold()
                self.backend.request_offboard()
                self.backend.arm()
                self._goto("RETURN")
            elif now - self.state_t0 > 3.0 and not self.get_parameter("enable_return").value:
                self._goto("DONE")
        elif self.state == "DONE":
            pass

    def _blackout_governor(self, pm, now):
        """After leaving a dark zone, creep until the localizer delivers a fresh fix.

        Inside the tube visual odometry coasts; the first metres after the exit are
        where an uncorrected offset would put the drone into the chicane walls.
        Disabled when post_dark_scale >= 1 or no localizer is running.
        """
        in_dark = any(lo[0] - 0.05 <= pm[0] <= hi[0] + 0.05 and lo[1] - 0.05 <= pm[1] <= hi[1] + 0.05
                      for lo, hi in self.dark)
        if in_dark:
            self._dark_exit_t = None
            self._was_dark = True
            self.follower.speed_scale = 1.0
            return
        if getattr(self, "_was_dark", False):
            self._was_dark = False
            self._dark_exit_t = now
        t_exit = getattr(self, "_dark_exit_t", None)
        scale = self.get_parameter("post_dark_scale").value
        if t_exit is not None and scale < 1.0:
            fixed = self.last_fix_t > t_exit
            if fixed or now - t_exit > self.get_parameter("post_dark_timeout").value:
                self._dark_exit_t = None
                self.follower.speed_scale = 1.0
            else:
                self.follower.speed_scale = scale
        else:
            self.follower.speed_scale = 1.0

    def _land_step(self, pm):
        if self.backend.can_land:
            if now_elapsed(self.state_t0) < 0.2:
                self.backend.land()
            landed = bool(self.backend.landed())
            if landed and self._landed_since is None:
                self._landed_since = time.time()
            elif not landed:
                self._landed_since = None
            # The scored clock stops at disarm (R7.5): disarm as soon as the land
            # detector is confident instead of waiting for COM_DISARM_LAND (2 s).
            if landed and self.backend.armed() and time.time() - self._landed_since > \
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
            vo = self.align.vec_to_odom(v)
            self.backend.send_velocity(vo[0], vo[1], float(np.clip(vo[2], -0.4, 0.4)), 0.0)
            self.get_logger().warn("backend cannot land autonomously; holding over pad", throttle_duration_sec=2.0)

    def _safety(self, v, pm):
        # forward-clearance speed governor (stereo depth)
        mc = self.get_parameter("min_forward_clearance").value
        if self.fwd_clear < 2 * mc:
            scale = max(0.0, (self.fwd_clear - mc) / mc)
            v = v.copy(); v[:2] *= scale
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
        m.header.stamp = self.get_clock().now().to_msg()
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


def now_elapsed(t0: float) -> float:
    return time.time() - t0


def main():
    rclpy.init()
    node = MissionNode()
    try:
        rclpy.spin(node)
    finally:
        node.backend.hold()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
