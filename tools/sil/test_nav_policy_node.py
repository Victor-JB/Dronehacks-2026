"""nav_policy node on fake ROS: endpoints, organizer handoff rules, final-command safety. No DDS, no PX4.
The drone does not move here (outputs are checked, not flown); closed-loop behaviour is in dh_core tests."""
import json
import math
import os
import sys
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
for p in (HERE, ROOT, os.path.join(ROOT, "ros2_ws", "src", "dh_core"), os.path.join(ROOT, "ros2_ws", "src", "dh_autonomy")):
    if p not in sys.path:
        sys.path.insert(0, p)
import fake_ros  # noqa: E402
fake_ros.install()
from fake_ros import BUS, Node, TfBuffer  # noqa: E402

sys.modules.setdefault("isaac_ros_visual_slam_interfaces", types.ModuleType("isaac_ros_visual_slam_interfaces"))
sys.modules["isaac_ros_visual_slam_interfaces.msg"] = types.SimpleNamespace(VisualSlamStatus=type("VisualSlamStatus", (), {}))

from dh_autonomy.nav_policy_node import NavPolicyNode  # noqa: E402
from dh_core.shadow.synth import (CAM_FRAME, bag_tree, depth_msg, info_msg, odom_msg, quat_from_rpy,  # noqa: E402
                                  render_depth)

WALL = (3.0, 3.5, -4.0, 4.0, 0.0, 5.0)
FENCE = [-1.0, 4.0, -2.0, 2.0]


def fresh(**over):
    BUS.t = 10.0
    BUS.subs.clear(); BUS.timers.clear(); BUS.published.clear()
    BUS.service_calls.clear(); BUS.services_ready.clear()
    R, t = bag_tree().lookup("base_link", CAM_FRAME)
    w = math.sqrt(1 + np.trace(R)) / 2
    q = ((R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w)
    TfBuffer.transforms = {("base_link", CAM_FRAME): types.SimpleNamespace(transform=types.SimpleNamespace(
        rotation=types.SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3]),
        translation=types.SimpleNamespace(x=t[0], y=t[1], z=t[2])))}
    Node.PARAM_OVERRIDES = dict(over)


class World:
    """Feeds the node like the drone would: odometry 30 Hz, depth 15 Hz, state 5 Hz, VO 30 Hz."""
    def __init__(self, node, pos=(0.0, 0.0, 0.75), yaw=0.0):
        self.node, self.pos, self.yaw = node, np.array(pos, float), yaw
        self.offboard, self.armed, self.vo = False, True, 1
        self.depth_on, self.odom_on = True, True
        self.img = render_depth(self.pos, quat_from_rpy(0, 0, yaw), [WALL])

    def run(self, seconds):
        n = int(round(seconds / 0.01))
        for _ in range(n):
            BUS.t = round(BUS.t + 0.01, 6)
            k = int(round(BUS.t * 100))
            if k % 3 == 0 and self.odom_on:
                BUS.publish("/visual_slam/tracking/odometry", odom_msg(BUS.t - 0.05, self.pos, quat_from_rpy(0, 0, self.yaw), [0, 0, 0]))
            if k % 3 == 0:
                BUS.publish("/visual_slam/status", types.SimpleNamespace(vo_state=self.vo))
                BUS.publish("/oak/stereo/camera_info", info_msg(BUS.t - 0.01))
            if k % 7 == 0 and self.depth_on:
                BUS.publish("/oak/stereo/image_raw", depth_msg(BUS.t - 0.06, self.img))
            if k % 20 == 0:
                BUS.publish("/skybeta/state", types.SimpleNamespace(data=json.dumps(
                    {"stamp": BUS.t, "px4_link": True, "armed": self.armed, "offboard_active": self.offboard,
                     "frozen": False, "landed": False, "clamped": []})))
            if k % 10 == 0:
                self.node._tick()


def cmds(topic="/skybeta/setpoint/twist", since=None):
    out = BUS.published.get(topic, [])
    return [m for t, m in out if since is None or t >= since]


def twist(m):
    return np.array([m.twist.linear.x, m.twist.linear.y, m.twist.linear.z, m.twist.angular.z])


def test_dry_run_is_default_and_isolated():
    fresh()
    n = NavPolicyNode()
    topics = {p.topic_name for p in n.publishers}
    assert n.dry and topics == {"/dh/dry_run/cmd_vel", "/dh/dry_run/events"} and n.clients == []


def test_live_endpoints_are_only_the_organizer_interface():
    fresh(dry_run=False, geofence=FENCE)
    n = NavPolicyNode()
    assert {p.topic_name for p in n.publishers} == {"/skybeta/setpoint/twist", "/skybeta/setpoint/pose"}
    assert all(not c.srv_name.startswith("/fmu") for c in n.clients)
    assert not any(t.startswith("/fmu/in") for t in BUS.subs)


def test_before_offboard_a_zero_twist_every_tick_and_nothing_else():
    fresh(dry_run=False, geofence=FENCE)
    n = NavPolicyNode(); w = World(n)
    w.run(2.0)
    tw = cmds()
    assert 15 <= len(tw) <= 21                                         # 10 Hz
    assert all(np.all(twist(m) == 0) for m in tw) and cmds("/skybeta/setpoint/pose") == []
    assert all(m.header.frame_id == "odom" for m in tw)
    assert n.phase == "WAIT"


def test_unset_geofence_never_navigates():
    fresh(dry_run=False)                                               # geofence left unset
    n = NavPolicyNode(); w = World(n)
    w.run(1.0); w.offboard = True; w.run(3.0)
    assert n.phase == "WAIT" and cmds("/skybeta/setpoint/pose") == []
    assert all(np.all(twist(m) == 0) for m in cmds())


def test_takeoff_pose_then_bounded_nav_twists():
    fresh(dry_run=False, geofence=FENCE)
    n = NavPolicyNode(); w = World(n)
    w.run(1.0); w.offboard = True; t_on = BUS.t
    w.run(14.0)
    poses = cmds("/skybeta/setpoint/pose", t_on)
    assert poses, "takeoff pose expected"
    p0 = poses[0].pose.position
    assert abs(p0.z - 0.75) < 1e-9 and abs(p0.x) < 0.05 and abs(p0.y) < 0.05     # start xy, cruise height (ENU)
    nav = [twist(m) for m in cmds(since=t_on)]
    assert nav and all(np.all(np.isfinite(v)) for v in nav)
    assert max(np.linalg.norm(v[:3]) for v in nav) <= 0.35 + 1e-9               # v_max, under the 1.5 m/s cap
    assert n.phase in ("NAV", "HOLD")


def test_stale_depth_gives_zero_motion():
    fresh(dry_run=False, geofence=FENCE)
    n = NavPolicyNode(); w = World(n)
    w.run(1.0); w.offboard = True; w.run(12.0)
    w.depth_on = False; w.run(0.5); t0 = BUS.t; w.run(2.0)
    late = [twist(m) for m in cmds(since=t0)]
    assert all(np.linalg.norm(v[:3]) == 0.0 for v in late)


def test_localization_loss_brakes_then_stops_commanding_and_never_resumes():
    fresh(dry_run=False, geofence=FENCE)
    n = NavPolicyNode(); w = World(n)
    w.run(1.0); w.offboard = True; w.run(10.0)
    w.vo = 2; t0 = BUS.t; w.run(4.0)
    after = cmds(since=t0 + 0.05)                                        # first tick after the VO-lost message
    assert n.phase == "LOC_LOST"
    assert all(np.all(twist(m) == 0) for m in after)
    assert all(t <= t0 + 1.5 + 0.35 for t, m in BUS.published["/skybeta/setpoint/twist"] if t >= t0)   # brake, then silent
    w.vo = 1; t1 = BUS.t; w.run(3.0)
    assert n.phase == "LOC_LOST" and cmds(since=t1) == [] and cmds("/skybeta/setpoint/pose", t1) == []


def test_takeover_latches_until_disarm():
    fresh(dry_run=False, geofence=FENCE)
    n = NavPolicyNode(); w = World(n)
    w.run(1.0); w.offboard = True; w.run(5.0)
    w.offboard = False; t0 = BUS.t; w.run(2.0)
    assert n.phase == "YIELDED" and cmds(since=t0 + 0.3) == [] and cmds("/skybeta/setpoint/pose", t0 + 0.3) == []
    w.offboard = True; t1 = BUS.t; w.run(2.0)                          # Offboard again without re-arming: no
    assert n.phase == "YIELDED" and cmds(since=t1) == []
    w.offboard, w.armed = False, False; w.run(1.0)
    assert n.phase == "WAIT"
