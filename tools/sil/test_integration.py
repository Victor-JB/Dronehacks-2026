"""Integration tests of the REAL dh_autonomy nodes (mission, localizer, stereo_scan) on the fake ROS bus.

Mocked, and only at the documented hardware/organizer boundary:
  * odometry (stands in for cuVSLAM /visual_slam/tracking/odometry),
  * /skybeta/state (stands in for the organizers' flight_interface),
  * camera images (rendered; see test_stereo_node.py for the renderer).
Everything between those inputs and the velocity commands is the real node code.
These are NOT ROS 2 tests (no DDS, QoS or executor); the ROS-graph test for the Jetson is
ros2_ws/src/dh_autonomy/test/test_ros_graph_dry_run.py.

  python tools/sil/test_integration.py          (or pytest tools/sil/test_integration.py)
"""
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "dh_core"))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "dh_autonomy"))
import fake_ros  # noqa: E402

fake_ros.install()
from fake_ros import (BUS, Empty, LaserScan, Node, Odometry, Range, String, TfBuffer,  # noqa: E402
                      make_time)
from dh_core.localize import raycast_scan  # noqa: E402

COURSE = os.environ.get("DH_COURSE_OBJ", "C:/dhdl/obj/dronehacks-course-v3.obj")
DT = 0.01


def fresh(**over):
    BUS.t = 1.0
    BUS.subs.clear(); BUS.timers.clear(); BUS.published.clear()
    BUS.service_calls.clear(); BUS.services_ready.clear()
    TfBuffer.transforms = {}
    p = {"course_obj": COURSE, "rate_hz": 30.0, "require_vo_status": False, "mount_source": "parameters"}
    p.update(over)
    Node.PARAM_OVERRIDES = p


def run(seconds, each=None):
    n = int(round(seconds / DT))
    for _ in range(n):
        if each is not None:
            each(BUS.t)
        for tm in BUS.timers:
            tm[1] += DT
            if tm[1] >= tm[0] - 1e-9:
                tm[1] = 0.0
                tm[2]()
        BUS.t = round(BUS.t + DT, 6)


def odom_msg(t, xyz, yaw, stamp=None):
    o = Odometry()
    o.header.stamp = make_time(t if stamp is None else stamp)
    o.header.frame_id = "odom"
    o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = map(float, xyz)
    o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return o


def state_msg(active):
    return String(data=json.dumps({"offboard_active": bool(active), "armed": True}))


def clear_msg(t, rng):
    r = Range()
    r.header.stamp = make_time(t); r.header.frame_id = "base_link"
    r.min_range, r.max_range = 0.2, 4.0
    r.range = rng
    return r


def published(topic):
    return BUS.published.get(topic, [])


def mission(**over):
    import importlib
    import dh_autonomy.mission_node as mn
    importlib.reload(mn)
    return mn.MissionNode()


def stationary_feed(node, xyz, yaw, active=None, fc=math.inf):
    """30 Hz odometry, 10 Hz /skybeta/state and forward clearance."""
    def each(t):
        k = int(round(t * 100))
        if k % 3 == 0:
            BUS.publish("/visual_slam/tracking/odometry", odom_msg(t, xyz, yaw))
        if k % 10 == 0:
            if active is not None:
                BUS.publish("/skybeta/state", state_msg(active(t) if callable(active) else active))
            BUS.publish("/dh/forward_clearance", clear_msg(t, fc))
    return each


# ------------------------------------------------------------------------------------------- tests
def test_dry_run_isolation_enforced_in_code():
    fresh(dry_run=True)
    node = mission()
    names = node.endpoints
    assert not any(n.startswith("/skybeta") or n.startswith("/fmu") for n in names), names
    launch = node.course.launch
    run(1.0, stationary_feed(node, (launch[0], launch[1], 0.0), math.pi / 2))
    BUS.publish("/dh/start", Empty())
    run(3.0, stationary_feed(node, (launch[0], launch[1], 0.0), math.pi / 2))
    assert len(published("/dh/dry_run/cmd_vel")) > 50
    assert not [k for k in BUS.published if k.startswith("/skybeta/setpoint") or k.startswith("/fmu/in")]
    # a backend that tried to create a flight-control publisher must make the node refuse to start
    import dh_autonomy.backends as be
    orig = be.DryRunBackend.__init__

    def bad_init(self, n, out_ns="/dh/dry_run"):
        orig(self, n, out_ns)
        n.create_publisher(object, "/skybeta/setpoint/twist", 10)
    be.DryRunBackend.__init__ = bad_init
    try:
        fresh(dry_run=True)
        import dh_autonomy.mission_node as mn
        mn.DryRunBackend = be.DryRunBackend
        raised = False
        try:
            mn.MissionNode()
        except RuntimeError as e:
            raised = "ISOLATION" in str(e)
        assert raised
    finally:
        be.DryRunBackend.__init__ = orig
    try:
        be.DryRunBackend(Node("x"), out_ns="/skybeta")
        assert False, "forbidden output namespace accepted"
    except RuntimeError:
        pass


def test_skybeta_only_zero_readiness_before_offboard_and_clock_waits():
    fresh(dry_run=False)
    node = mission()
    L = node.course.launch
    feed = stationary_feed(node, (L[0], L[1], 0.0), math.pi / 2, active=lambda t: t >= 5.0)
    run(1.0, feed)
    BUS.publish("/dh/start", Empty())
    run(3.0, feed)                                      # started, but Offboard not active
    assert node.state == "TAKEOFF"
    assert len(published("/skybeta/setpoint/twist")) > 0
    for _, m in published("/skybeta/setpoint/twist"):
        assert [m.twist.linear.x, m.twist.linear.y, m.twist.linear.z, m.twist.angular.z] == [0.0] * 4
    assert node.follower is None                        # trajectory clock has not started
    run(1.5, feed)                                      # Offboard becomes active at t = 5 s
    assert len(published("/skybeta/setpoint/twist")) > 10
    assert node.follower is not None and node.was_active


def test_offboard_loss_aborts_and_stops_commanding():
    fresh(dry_run=False)
    node = mission()
    L = node.course.launch
    feed = stationary_feed(node, (L[0], L[1], 0.0), math.pi / 2, active=lambda t: t < 4.0)
    run(1.0, feed)
    BUS.publish("/dh/start", Empty())
    run(2.5, feed)
    n_before = len(published("/skybeta/setpoint/twist"))
    assert n_before > 10
    run(2.0, feed)                                      # Offboard lost at t = 4 s (pilot took over)
    assert node.state == "ABORTED", node.state
    n_after = len(published("/skybeta/setpoint/twist"))
    run(2.0, feed)
    assert len(published("/skybeta/setpoint/twist")) == n_after   # nothing more after the abort
    assert not [c for c in BUS.service_calls if c[1].endswith("request_offboard")]


def test_stale_skybeta_state_is_not_offboard():
    fresh(dry_run=False)
    node = mission()
    L = node.course.launch
    stop_state = {"t": 99.0}
    feed = stationary_feed(node, (L[0], L[1], 0.0), math.pi / 2, active=True)

    def each(t):
        k = int(round(t * 100))
        if k % 3 == 0:
            BUS.publish("/visual_slam/tracking/odometry", odom_msg(t, (L[0], L[1], 0.0), math.pi / 2))
        if k % 10 == 0:
            if t < stop_state["t"]:
                BUS.publish("/skybeta/state", state_msg(True))
            BUS.publish("/dh/forward_clearance", clear_msg(t, math.inf))
    run(1.0, each)
    BUS.publish("/dh/start", Empty())
    run(1.0, each)
    assert node.was_active
    stop_state["t"] = BUS.t                             # the interface stops publishing its state
    run(1.5, each)
    assert node.state == "ABORTED"


def test_odometry_loss_hovers_then_aborts():
    fresh(dry_run=True)
    node = mission()
    L = node.course.launch
    alive = {"until": 99.0}

    def each(t):
        k = int(round(t * 100))
        if k % 3 == 0 and t < alive["until"]:
            BUS.publish("/visual_slam/tracking/odometry", odom_msg(t, (L[0], L[1], 0.0), math.pi / 2))
        if k % 10 == 0:
            BUS.publish("/dh/forward_clearance", clear_msg(t, math.inf))
    run(1.0, each)
    BUS.publish("/dh/start", Empty())
    run(1.0, each)
    alive["until"] = BUS.t
    run(1.0, each)                                      # stale > 0.4 s -> HOVER (zero velocity)
    last = published("/dh/dry_run/cmd_vel")[-1][1].twist.linear
    assert node.state == "TAKEOFF" and abs(last.x) + abs(last.y) + abs(last.z) < 1e-9
    assert any(m.data.startswith("HOVER") for _, m in published("/dh/health")[-20:])
    run(1.5, each)                                      # > 2 s -> ABORTED
    assert node.state == "ABORTED"


def test_forward_clearance_missing_freezes_course_progress():
    fresh(dry_run=True)
    node = mission()
    L = node.course.launch
    feed_ok = stationary_feed(node, (L[0], L[1], 0.0), math.pi / 2)

    def feed_no_fc(t):
        k = int(round(t * 100))
        if k % 3 == 0:
            BUS.publish("/visual_slam/tracking/odometry", odom_msg(t, (L[0], L[1], 0.85), math.pi / 2))
    run(1.0, feed_ok)
    BUS.publish("/dh/start", Empty())
    node._goto("COURSE")                                # straight into the course leg at cruise height
    run(0.7, feed_no_fc)                                # last clearance reading ages past fc_timeout (0.5 s)
    tau0 = node.follower.tau if node.follower else 0.0
    run(1.5, feed_no_fc)                                # stereo node silent: fc STALE
    assert node.follower is None or abs(node.follower.tau - tau0) < 1e-6, (tau0, node.follower.tau)
    v = published("/dh/dry_run/cmd_vel")[-1][1].twist.linear
    assert math.hypot(v.x, v.y) < 1e-6


def build_localizer():
    import importlib
    import dh_autonomy.localizer_node as ln
    importlib.reload(ln)
    return ln.LocalizerNode()


def _delayed_scan_run(stamp_at_capture: bool, latency: float = 0.3):
    """Drone moves north from the launch mark toward the end wall of the first corridor at 0.8 m/s with
    PERFECT odometry (odom frame = course frame). Scans are captured at t and delivered at t+latency.
    Correct handling must leave the estimate untouched."""
    fresh(dry_run=True, align_mode="first_odom", ground_z=0.85, correction_gain=0.5)
    node = mission()
    loc = build_localizer()
    L = node.course.launch
    grid = loc.matcher.grid if hasattr(loc.matcher, "grid") else None
    from dh_core.course import Course
    scan_grid = node.course.occupancy(0.025, 0.58, 1.12)
    queue = []
    truth = {"p": np.array([L[0], L[1], 0.85])}

    def pos(t):
        return np.array([L[0], L[1] + 0.8 * max(0.0, t - 1.5), 0.85])

    def each(t):
        k = int(round(t * 100))
        p = pos(t)
        truth["p"] = p
        if k % 3 == 0:
            BUS.publish("/visual_slam/tracking/odometry", odom_msg(t, p, math.pi / 2))
        if k % 10 == 0 and t > 1.2:
            sc = raycast_scan(scan_grid, np.array([p[0], p[1], math.pi / 2]), 80.0, 64, r_max=4.0, noise=0.0)
            if len(sc):
                ls = LaserScan()
                ls.header.frame_id = "base_link"
                r = np.hypot(sc[:, 0], sc[:, 1]); a = np.arctan2(sc[:, 1], sc[:, 0])
                order = np.argsort(a)
                ls.angle_min = float(a[order][0]); n = len(order)
                ls.angle_increment = float((a[order][-1] - a[order][0]) / max(n - 1, 1))
                # resample onto a uniform grid of beams
                grid_a = ls.angle_min + np.arange(n) * ls.angle_increment
                ls.ranges = list(np.interp(grid_a, a[order], r[order]))
                ls.range_min, ls.range_max = 0.2, 4.0
                queue.append((t + latency, t, ls))
        while queue and queue[0][0] <= t + 1e-9:
            due, tc, ls = queue.pop(0)
            ls.header.stamp = make_time(tc if stamp_at_capture else due)
            BUS.publish("/dh/stereo_scan", ls)
    run(3.2, each)
    pm, _ = node._map_pose()
    return float(np.linalg.norm(pm[:2] - truth["p"][:2])), node, loc


def test_delayed_scans_do_not_cause_false_corrections():
    err_ok, node, loc = _delayed_scan_run(stamp_at_capture=True)
    assert node.counters["fix_applied"] > 3, (node.counters, loc.rejected)
    assert err_ok < 0.03, err_ok
    # control: the same data stamped at DELIVERY time (the old behaviour) corrupts the estimate
    err_bad, node2, _ = _delayed_scan_run(stamp_at_capture=False)
    assert err_bad > 3 * max(err_ok, 0.02), (err_ok, err_bad)
    print("    delayed scans: estimate error %.3f m with capture stamps vs %.3f m when stamped at delivery"
          % (err_ok, err_bad))


def test_localizer_rejects_unstamped_stale_future_and_wrong_frame():
    fresh(dry_run=True, align_mode="first_odom")
    node = mission()
    loc = build_localizer()
    L = node.course.launch
    run(3.0, stationary_feed(node, (L[0], L[1], 0.85), math.pi / 2))
    base = LaserScan()
    base.angle_min, base.angle_increment = -0.6, 0.02
    base.ranges = [1.0] * 61
    base.range_min, base.range_max = 0.2, 4.0

    def send(stamp, frame="base_link"):
        m = LaserScan(); m.angle_min, m.angle_increment, m.ranges = base.angle_min, base.angle_increment, list(base.ranges)
        m.range_min, m.range_max = 0.2, 4.0
        m.header.stamp = make_time(stamp) if stamp is not None else make_time(0.0)
        m.header.frame_id = frame
        BUS.publish("/dh/stereo_scan", m)
    send(None)
    send(BUS.t - 1.5)          # older than max_scan_latency (0.5 s) but stamped
    send(BUS.t + 1.0)          # in the future
    send(BUS.t - 0.05, frame="oak_left_camera_optical_frame")   # no TF for this frame
    r = loc.rejected
    assert r["unstamped"] == 1 and r["stale"] == 1 and r["future"] == 1 and r["frame"] == 1, r
    # with a TF base_link <- camera frame the same frame is accepted (it gets matched)
    import types
    TfBuffer.transforms[("base_link", "cam")] = types.SimpleNamespace(transform=types.SimpleNamespace(
        translation=types.SimpleNamespace(x=0.1, y=0.0, z=0.0),
        rotation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))
    before = dict(r)
    send(BUS.t - 0.05, frame="cam")
    assert loc.rejected["frame"] == before["frame"]


def test_out_of_order_fix_is_rejected():
    from fake_ros import PoseWithCovarianceStamped
    fresh(dry_run=True, align_mode="first_odom")
    node = mission()
    L = node.course.launch
    run(1.0, stationary_feed(node, (L[0], L[1], 0.85), math.pi / 2))

    def fix(stamp, dx):
        m = PoseWithCovarianceStamped()
        m.header.stamp = make_time(stamp)
        m.header.frame_id = "course"
        m.pose.pose.position.x, m.pose.pose.position.y = L[0] + dx, L[1]
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(math.pi / 4), math.cos(math.pi / 4)
        BUS.publish("/dh/map_fix", m)
    fix(BUS.t - 0.1, 0.02)
    fix(BUS.t - 0.3, 0.50)                              # older than the applied fix: must be ignored
    assert node.counters["fix_applied"] == 1 and node.counters["fix_rejected"] == 1, node.counters
    fix(0.0, 0.5)                                       # unstamped
    fix(BUS.t - 3.0, 0.5)                               # older than the odometry history
    assert node.counters["fix_applied"] == 1 and node.counters["fix_rejected"] == 3, node.counters


def test_visual_tracking_loss_hovers_outside_dark_zone():
    fresh(dry_run=True)
    node = mission()
    L = node.course.launch
    feed = stationary_feed(node, (L[0], L[1], 0.0), math.pi / 2)
    run(1.0, feed)
    BUS.publish("/dh/start", Empty())
    run(1.0, feed)
    node.health.on_vo_status(BUS.t, 2)                  # cuVSLAM reports tracking lost (vo_state 2)
    run(0.5, feed)
    v = published("/dh/dry_run/cmd_vel")[-1][1].twist.linear
    assert abs(v.x) + abs(v.y) + abs(v.z) < 1e-9
    node.health.on_vo_status(BUS.t, 1)                  # recovered
    run(0.5, feed)
    assert node.state == "TAKEOFF"
    node.health.on_vo_status(BUS.t, 2)
    run(3.5, feed)
    assert node.state == "ABORTED"


def test_startup_without_any_sensor_is_silent():
    fresh(dry_run=False)
    node = mission()
    loc = build_localizer()
    run(3.0)
    BUS.publish("/dh/start", Empty())                   # ignored: no odometry
    run(1.0)
    assert node.state == "WAIT_START"
    assert not published("/skybeta/setpoint/twist") and not published("/dh/map_fix")


def test_stereo_node_reports_unknown_not_clear():
    """Real stereo_scan_node on rendered pairs: wall -> finite, textureless/dark/unstamped -> never +inf."""
    import importlib
    import test_stereo_node as tsn                      # renderer + camera model (no ROS)
    fresh(rectify_source="auto", tf_wait_s=0.0)
    import dh_autonomy.stereo_scan_node as ssn
    importlib.reload(ssn)
    node = ssn.StereoScanNode()
    K = tsn.K
    from dh_core.camera_geom import stereo_rectify
    R1, R2, P1, P2, _ = stereo_rectify(K, np.zeros(5), K, np.zeros(5), (tsn.W, tsn.H), np.eye(3), np.array([-0.075, 0, 0]))
    node._on_info("l", tsn.make_info("l", R1, P1)); node._on_info("r", tsn.make_info("r", R2, P2))
    sync = fake_ros.ApproximateTimeSynchronizer.last

    def pair(left, right, t=5.0, tr=None):
        BUS.t = max(t, 5.0)
        a, b = tsn.make_image(left, "l"), tsn.make_image(right, "r")
        a.header.stamp = make_time(t); b.header.stamp = make_time(t if tr is None else tr)
        sync.cb(a, b)
        out = published("/dh/forward_clearance")
        return out[-1][1].range if out else None
    q = (0.0, 0.0, 0.0, 1.0)
    left = tsn.render(np.zeros(3), np.eye(3), 1.2)
    right = tsn.render(np.array([0.075, 0, 0]), np.eye(3), 1.2)
    r_wall = pair(left, right)
    assert r_wall is not None and abs(r_wall - 1.2) < 0.15, r_wall
    flat = np.full_like(left, 128)
    assert math.isnan(pair(flat, flat, t=5.1))           # textureless: unknown, NOT clear
    dark = (left * 0.03).astype(np.uint8)
    assert math.isnan(pair(dark, dark, t=5.2))           # too dark (the tube): unknown
    assert math.isnan(pair(left, right, t=5.3, tr=5.35)) # left/right not from the same capture
    n = len(published("/dh/forward_clearance"))
    pair(left, right, t=0.0)                             # unstamped: dropped, nothing published
    assert len(published("/dh/forward_clearance")) == n
    far_l = tsn.render(np.zeros(3), np.eye(3), 8.0)
    far_r = tsn.render(np.array([0.075, 0, 0]), np.eye(3), 8.0)
    r_far = pair(far_l, far_r, t=5.4)
    assert r_far == math.inf, r_far                      # measured: nothing within 4 m (valid clear)


if __name__ == "__main__":
    import traceback
    tests = [(n, f) for n, f in list(globals().items()) if n.startswith("test_") and callable(f)]
    passed = failed = 0
    for name, fn in tests:
        t0 = time.time()
        try:
            fn()
            passed += 1
            print("PASS %s (%.1fs)" % (name, time.time() - t0))
        except Exception:  # noqa: BLE001
            failed += 1
            print("FAIL %s" % name)
            traceback.print_exc()
    print("integration tests: %d passed, %d failed" % (passed, failed))
    sys.exit(1 if failed else 0)
