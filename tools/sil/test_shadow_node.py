"""Shadow node on fake ROS: endpoint creation, read-only guarantee, and parity with the pure replay.
No DDS, no hardware. Real rclpy also creates implicit endpoints (/rosout, /parameter_events, parameter
services); guard.check_endpoints allows exactly those, tested in dh_core/test/test_shadow.py."""
import json
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
for p in (HERE, ROOT, os.path.join(ROOT, "ros2_ws", "src", "dh_core"), os.path.join(ROOT, "ros2_ws", "src", "dh_autonomy")):
    if p not in sys.path:
        sys.path.insert(0, p)
import fake_ros  # noqa: E402
fake_ros.install()
from fake_ros import BUS, Node, TfBuffer  # noqa: E402

# VisualSlamStatus exists only on the drone; provide a stand-in so the VO subscription is exercised
sys.modules.setdefault("isaac_ros_visual_slam_interfaces", types.ModuleType("isaac_ros_visual_slam_interfaces"))
sys.modules["isaac_ros_visual_slam_interfaces.msg"] = types.SimpleNamespace(VisualSlamStatus=type("VisualSlamStatus", (), {}))

from dh_autonomy.shadow_node import ShadowNode  # noqa: E402
from dh_core.shadow.monitor import ShadowMonitor, deterministic  # noqa: E402
from dh_core.shadow.replay import replay  # noqa: E402
from dh_core.shadow.synth import BAG_TF, CAM_FRAME, Scenario, bag_extrinsic, bag_tree, straight  # noqa: E402
from dh_core.shadow.guard import FORBIDDEN_TOPIC_PREFIXES  # noqa: E402

TOPIC = {"odom": "/visual_slam/tracking/odometry", "state": "/skybeta/state", "camera_info": "/oak/stereo/camera_info",
         "depth": "/oak/stereo/image_raw", "vo": "/visual_slam/status"}
WALL = (3.0, 3.5, -4.0, 4.0, 0.0, 5.0)


def fresh(**over):
    BUS.t = 1.0
    BUS.subs.clear(); BUS.timers.clear(); BUS.published.clear()
    BUS.service_calls.clear(); BUS.services_ready.clear()
    R, t = bag_tree().lookup("base_link", CAM_FRAME)
    from dh_core.shadow.synth import quat_from_rpy  # noqa
    import numpy as np
    # rotation matrix -> quaternion (w>0 branch is fine for this transform)
    w = np.sqrt(1 + np.trace(R)) / 2
    q = ((R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w)
    TfBuffer.transforms = {("base_link", CAM_FRAME): types.SimpleNamespace(transform=types.SimpleNamespace(
        rotation=types.SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3]),
        translation=types.SimpleNamespace(x=t[0], y=t[1], z=t[2])))}
    Node.PARAM_OVERRIDES = dict(over)


def drive(node, events, eval_times):
    """Deliver events through the fake bus in receipt order, ticking the node at eval_times."""
    from dh_core.shadow.replay import KINDS
    ev = sorted(events, key=lambda e: (e[0], KINDS.index(e[1])))
    out, k = [], 0
    for te in eval_times:
        while k < len(ev) and ev[k][0] <= te + 1e-12:
            rx, kind, msg = ev[k]
            BUS.t = rx
            BUS.publish(TOPIC[kind], types.SimpleNamespace(vo_state=msg) if kind == "vo" else msg)
            k += 1
        BUS.t = te
        node._tick()
        out.append(deterministic(node.mon.last_report))
    return out


def same(a, b, path="r"):
    if isinstance(a, dict):
        assert isinstance(b, dict) and set(a) == set(b), path
        for k in a:
            same(a[k], b[k], path + "." + str(k))
    elif isinstance(a, (list, tuple)):
        assert isinstance(b, (list, tuple)) and len(a) == len(b), path
        for i, (x, y) in enumerate(zip(a, b)):
            same(x, y, "%s[%d]" % (path, i))
    elif isinstance(a, float) and isinstance(b, (int, float)) and not isinstance(b, bool):
        assert a == b or abs(a - b) <= max(1e-6, 1e-4 * abs(b)) or (a != a and b != b), (path, a, b)
    else:
        assert a == b, (path, a, b)


def test_node_creates_only_subscriptions():
    fresh()
    n = ShadowNode()
    assert n.publishers == [] and n.clients == []
    assert n.endpoint_violations() == []
    assert set(TOPIC.values()) <= set(BUS.subs)
    for topic in BUS.subs:
        assert not topic.startswith(FORBIDDEN_TOPIC_PREFIXES)


def test_node_matches_pure_replay_and_publishes_nothing():
    fresh()
    node = ShadowNode()
    sc = Scenario(straight(v=0.6), boxes=[WALL], duration=3.0)
    ev = sc.events()
    mon = ShadowMonitor(); mon.set_extrinsic(bag_extrinsic())
    ref = replay(ev, mon)
    got = drive(node, ev, [r["t"] for r in ref])
    same(got, ref)        # numbers within 1e-6 abs or 1e-4 rel (ROS clock is ns-quantized); verdicts, reasons, states exact
    assert {r["verdict"] for r in got} >= {"SAFE", "UNKNOWN"}
    # the bus holds only the sensor messages this test injected: the node added nothing, called nothing
    assert set(BUS.published) == set(TOPIC.values()) and BUS.service_calls == []
    t_last = ref[-1]["t"]                                             # drive() delivers up to the last tick
    n_in = {tp: sum(1 for e in ev if TOPIC[e[1]] == tp and e[0] <= t_last + 1e-12) for tp in TOPIC.values()}
    assert {tp: len(v) for tp, v in BUS.published.items()} == n_in


def test_node_refuses_to_run_with_a_command_endpoint():
    import pytest
    from std_msgs.msg import String

    class Sneaky(ShadowNode):
        def __init__(self):
            super().__init__()
            self.create_publisher(String, "/skybeta/setpoint/twist", 10)
            self.assert_read_only()

    class Calls(ShadowNode):
        def __init__(self):
            super().__init__()
            self.create_client(object, "/skybeta/arm")
            self.assert_read_only()

    for cls in (Sneaky, Calls):
        fresh()
        with pytest.raises(RuntimeError, match="refuses to run"):
            cls()


def test_node_without_tf_reports_unknown(tmp_path):
    fresh(log_path=str(tmp_path / "shadow.jsonl"), log_period_s=0.5)
    TfBuffer.transforms = {}
    node = ShadowNode()
    ev = Scenario(straight(v=0.3), boxes=[WALL], duration=2.0).events()
    t0 = min(e[0] for e in ev)
    got = drive(node, ev, [t0 + 0.1 * i for i in range(20)])
    assert all(r["max_speed"] == 0.0 for r in got)
    assert got[-1]["reason"] == "DEPTH_NO_TF"
    lines = (tmp_path / "shadow.jsonl").read_text().splitlines()
    assert len(lines) >= 3 and json.loads(lines[-1])["reason"] == "DEPTH_NO_TF"
    node._log_file.close()
