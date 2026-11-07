"""Node regressions on fake ROS, including real OpenCV. No DDS or hardware."""
import math
import types

import numpy as np

from test_integration import (BUS, Node, build_localizer, clear_msg, fresh, mission,
                              odom_msg, published, run, stationary_feed)
from fake_ros import Empty, PoseStamped, PoseWithCovarianceStamped, String, make_time
from dh_core.health import ClearanceParams, ForwardClearanceMonitor, LocalizationHealth, LocHealthParams


def zero_command():
    m = published("/dh/dry_run/cmd_vel")[-1][1].twist
    return [m.linear.x, m.linear.y, m.linear.z, m.angular.z] == [0.0] * 4


def test_clearance_cannot_be_waived_by_legacy_tuning_or_tube():
    m = ForwardClearanceMonitor(ClearanceParams(invalid_scale=1, invalid_floor_scale=1))
    m.update(1, 1, math.nan)
    assert m.speed_scale(1.1, expected_blind=True) == (0.0, "INVALID")
    m.update(2, 2, math.inf)
    m.update(2, 2.4, math.inf)  # replay must not refresh receipt
    assert m.state(2.6) == "STALE"
    m.update(20, 3, math.inf)
    assert m.speed_scale(3)[0] == 0
    m = ForwardClearanceMonitor()
    m.update(4, 4, -math.inf)
    assert m.speed_scale(4.1)[0] == 0


def test_health_uses_capture_age_and_detects_vertical_and_yaw_jumps():
    h = LocalizationHealth()
    h.on_odom(10, 8, (0, 0, 1))
    assert h.evaluate(10.1)[0] == "ABORT"
    h = LocalizationHealth()
    h.on_odom(1, 1, (0, 0, 1), yaw=0)
    h.on_odom(1.03, 1.03, (0, 0, 1.5), yaw=0)
    assert h.evaluate(1.04)[0] == "HOVER"
    h.on_odom(2, 2, (0, 0, 1.5), yaw=0)
    h.on_odom(2.03, 2.03, (0, 0, 1.5), yaw=0.8)
    assert h.evaluate(2.04)[0] == "HOVER"
    h.on_odom(2.8, 2.8, (0, 0, 1.5), yaw=0.8)
    assert h.evaluate(2.8)[0] == "HOVER"  # time alone cannot repair a frame reset
    for t in (2.81, 2.82, 2.83):
        h.on_map_fix(t, 0.02)
    assert h.evaluate(2.84)[0] == "DEGRADED"


def test_lit_tube_never_suppresses_tracking_faults_or_causes_false_hold():
    h = LocalizationHealth(LocHealthParams(require_vo_status=True))
    for t in np.arange(1, 7, 0.1):
        h.set_in_dark(t, True)
        h.on_odom(t, t, (0, 0, 1))
        h.on_vo_status(t, 1)
        assert h.evaluate(t)[0] == "OK"
    h.on_vo_status(7, 2)
    h.on_odom(7, 7, (0, 0, 1))
    assert h.evaluate(7)[0] == "HOVER"
    h.on_vo_status(8, 0)  # unknown is not tracking
    h.on_odom(8, 8, (0, 0, 1))
    assert h.evaluate(8)[0] == "HOVER"
    h.on_vo_status(8.1, 1)
    h.on_odom(9.2, 9.2, (0, 0, 1))
    assert h.evaluate(9.2)[0] == "HOVER"  # stale status despite fresh odom


def test_invalid_clearance_freezes_all_commands_and_recovers_on_fresh_data():
    fresh(dry_run=True)
    node = mission()
    L = node.course.launch
    feed = stationary_feed(node, (*L[:2], 0.0), math.pi / 2, fc=math.nan)
    run(1, feed)
    BUS.publish("/dh/start", Empty())
    run(1, feed)
    assert zero_command() and node.follower is None
    good = stationary_feed(node, (*L[:2], 0.0), math.pi / 2)
    run(0.3, good)
    assert node.follower is not None and node.follower.tau > 0
    tau = node.follower.tau
    run(0.3, feed)
    tau = node.follower.tau
    run(2, feed)
    assert zero_command() and node.follower.tau == tau
    assert published("/dh/health")[-1][1].data.startswith("HOVER fc=INVALID")


def test_replayed_or_invalid_odometry_cannot_keep_mission_alive():
    fresh(dry_run=True)
    node = mission()
    L = node.course.launch
    feed = stationary_feed(node, (*L[:2], 0), math.pi / 2)
    run(1, feed)
    BUS.publish("/dh/start", Empty())
    last_stamp = node.odom[2]
    def replay(t):
        BUS.publish("/visual_slam/tracking/odometry", odom_msg(t, (*L[:2], 0), math.pi / 2, stamp=last_stamp))
        node._on_clearance(clear_msg(t, math.inf))
    run(1, replay)
    assert zero_command()
    run(1.5, replay)
    assert node.state == "ABORTED"
    before = node.odom[2]
    for frame, x, q in [("wrong", 0, 1), ("odom", math.nan, 1), ("odom", 0, 0)]:
        o = odom_msg(BUS.t, (x, 0, 0), 0)
        o.header.frame_id = frame
        o.pose.pose.orientation.w = q
        node._on_odom(o)
        assert node.odom[2] == before


def test_skybeta_rejects_string_authority_and_does_not_unfreeze_hold():
    from dh_autonomy.backends import SkybetaBackend
    fresh()
    backend = SkybetaBackend(Node("backend_test"))
    backend._on_state(String(data='{"offboard_active":"false"}'))
    backend.send_velocity(1, 0, 0, 0)
    assert not published("/skybeta/setpoint/twist")
    backend._on_state(String(data='{"offboard_active":false,"frozen":false}'))
    backend.send_velocity(1, 0, 0, 0)
    backend.send_velocity(0, 0, 0, 0)
    assert len(published("/skybeta/setpoint/twist")) == 1
    backend._on_state(String(data='{"offboard_active":true,"frozen":true}'))
    backend.send_velocity(0, 0, 0, 0)
    assert len(published("/skybeta/setpoint/twist")) == 1


def test_fix_frame_duplicate_and_nonfinite_rejected():
    fresh(dry_run=True, align_mode="first_odom")
    node = mission()
    L = node.course.launch
    run(1, stationary_feed(node, (*L[:2], 0), math.pi / 2))
    m = PoseWithCovarianceStamped()
    m.header.stamp = make_time(BUS.t - 0.1)
    m.header.frame_id = "wrong"
    m.pose.pose.position.x, m.pose.pose.position.y = L[:2]
    m.pose.pose.orientation.z = m.pose.pose.orientation.w = math.sqrt(0.5)
    node._on_fix(m)
    m.header.frame_id = "course"
    m.pose.pose.position.x = math.nan
    node._on_fix(m)
    assert node.counters["fix_applied"] == 0
    m.pose.pose.position.x = L[0]
    node._on_fix(m)
    node._on_fix(m)
    assert node.counters["fix_applied"] == 1 and node.counters["fix_rejected"] == 3
    m.header.stamp = make_time(BUS.t - 0.05)
    m.pose.pose.position.x = L[0] + 100.0
    before = node.align.t.copy()
    node._on_fix(m)
    assert np.array_equal(before, node.align.t) and node.counters["fix_rejected"] == 4


def test_line_projects_using_capture_height_not_latest_height():
    import dh_autonomy.line_node as ln
    import test_line_node as renderer
    from fake_ros import CameraInfo, Image
    fresh()
    BUS.t = 2
    node = ln.LineNode()
    ci = CameraInfo()
    ci.k, ci.d = list(renderer.K.ravel()), [0.0] * 5
    ci.width, ci.height, ci.header.frame_id = renderer.W, renderer.H, "optical"
    node._on_info(ci)
    node.hist.add(1.8, (0, 0, 0.85, 0))
    node.hist.add(2, (0, 0, 1.4, 0))  # height changed after capture
    arr = renderer.render(0.85, 0.2, 0, 0.48, 200, 75, 215, np.random.default_rng(0))
    m = Image()
    m.height, m.width, m.step, m.encoding, m.data = renderer.H, renderer.W, renderer.W, "mono8", arr.tobytes()
    m.header.frame_id, m.header.stamp = "optical", make_time(1.8)
    node._on_img(m)
    out = published("/dh/line_obs")[-1][1]
    assert abs(out.vector.x - 0.2) < 0.01 and node.height == 0.85


def test_real_opencv_unknown_flows_through_to_mission_stop():
    import dh_autonomy.stereo_scan_node as ssn
    import test_stereo_node as renderer
    from fake_ros import ApproximateTimeSynchronizer
    from dh_core.camera_geom import stereo_rectify
    fresh(dry_run=True)
    node = mission()
    L = node.course.launch
    run(1, stationary_feed(node, (*L[:2], 0), math.pi / 2))
    BUS.publish("/dh/start", Empty())
    stereo = ssn.StereoScanNode()
    K = renderer.K
    R1, R2, P1, P2, _ = stereo_rectify(K, np.zeros(5), K, np.zeros(5), (renderer.W, renderer.H), np.eye(3), [-0.075, 0, 0])
    stereo._on_info("l", renderer.make_info("l", R1, P1))
    stereo._on_info("r", renderer.make_info("r", R2, P2))
    flat = np.full((renderer.H, renderer.W), 128, np.uint8)
    ml, mr = renderer.make_image(flat, "l"), renderer.make_image(flat, "r")
    ml.header.stamp = mr.header.stamp = make_time(BUS.t)
    ApproximateTimeSynchronizer.last.cb(ml, mr)  # actual OpenCV SGBM, not a synthetic Range
    node._tick()
    assert math.isnan(published("/dh/forward_clearance")[-1][1].range)
    assert zero_command() and node.follower is None
    # Calibration absence stays unknown; auto mode cannot silently assume 75 mm.
    stereo.maps = None
    stereo._on_info("r", renderer.make_info("r"))
    BUS.t += 0.1
    ml.header.stamp = mr.header.stamp = make_time(BUS.t)
    ApproximateTimeSynchronizer.last.cb(ml, mr)
    assert stereo.maps is None and math.isnan(published("/dh/forward_clearance")[-1][1].range)


def test_height_aware_scene_preserves_space_above_low_obstacle():
    from scene import VoxelScene
    occ = np.zeros((30, 30, 70), bool)
    occ[:5, :, 20:22] = True  # 25 cm high obstacle, x=1 m
    occ[:, :, 60:62] = True   # full-height wall, x=3 m
    scene = VoxelScene(occ, (0, 0, 0), 0.05)
    points, hit, floor = scene.raycast((0.1, 0.7, 0.85), [[1, 0, 0]])
    assert hit[0] and not floor[0] and abs(points[0, 0] - 3) < 0.06
    points, hit, _ = scene.raycast((0.1, 0.7, 0.1), [[1, 0, 0]])
    assert hit[0] and abs(points[0, 0] - 1) < 0.06


def test_capture_timestamp_tf_and_image_memory_layout():
    from dh_autonomy.stereo_scan_node import img_to_gray
    from fake_ros import Image, TfBuffer
    from dh_core.health import stamp_to_sec
    fresh()
    loc = build_localizer()
    requested = []
    tr = types.SimpleNamespace(transform=types.SimpleNamespace(
        translation=types.SimpleNamespace(x=0.1, y=0., z=0.),
        rotation=types.SimpleNamespace(x=0., y=0., z=0., w=1.)))
    loc.tf_buffer.lookup_transform = lambda target, source, stamp: (requested.append(stamp) or tr)
    stamp = make_time(1.25)
    assert loc._scan_to_base("laser", stamp) == (0.1, 0., 0.)
    assert stamp_to_sec(requested[0]) == 1.25
    im = Image()
    im.width, im.height, im.step, im.encoding = 2, 2, 4, "mono8"
    im.data = bytes([1, 2, 99, 99, 3, 4, 99, 99])
    assert img_to_gray(im).tolist() == [[1, 2], [3, 4]]
    im.step = 1
    try:
        img_to_gray(im)
        assert False, "invalid stride accepted"
    except ValueError:
        pass


def test_real_opencv_with_timestamped_optical_to_body_tf():
    import test_stereo_node as renderer
    result = renderer.run_case("camera_info", 1.5, (0., 0., 0., 1.), mount_source="tf")
    assert result is not None and abs(result["x_med"] - 1.5) < 0.06


def test_stationary_entrypoint_cannot_enable_physical_backends():
    from dh_autonomy.mission_node import StationaryMissionNode
    from dh_autonomy.backends import DryRunBackend
    for requested in ("skybeta", "px4"):
        fresh(dry_run=False, backend=requested)
        node = StationaryMissionNode()
        assert node.dry_run and isinstance(node.backend, DryRunBackend)
        assert not any(name.startswith(("/fmu", "/skybeta")) for name in node.endpoints)
        L = node.course.launch
        run(1, stationary_feed(node, (*L[:2], 0), math.pi / 2))
        BUS.publish("/dh/start", Empty())
        run(0.3, stationary_feed(node, (*L[:2], 0), math.pi / 2))
        assert published("/dh/dry_run/cmd_vel")
        assert not published("/skybeta/setpoint/twist")
        assert not BUS.service_calls


def test_isolation_checker_requires_positive_mission_evidence():
    import importlib.util
    from pathlib import Path
    path = Path(__file__).resolve().parents[1] / "hil" / "check_command_isolation.py"
    spec = importlib.util.spec_from_file_location("isolation_checker", path)
    checker = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(checker)
    assert checker.result_code([("dh_line", "/")], [], []) == 2
    assert checker.result_code([("dh_mission", "/")], [], []) == 2
    assert checker.result_code([("dh_mission", "/")], [], ["dh_mission"]) == 0
    assert checker.result_code([("dh_mission", "/")], [("dh_mission", "publisher", "/fmu/in/x")], []) == 1


def test_line_waits_for_delayed_odometry_without_extrapolation():
    import dh_autonomy.line_node as ln
    import test_line_node as renderer
    from fake_ros import CameraInfo, Image
    fresh()
    BUS.t = 2.0
    node = ln.LineNode()
    ci = CameraInfo()
    ci.k, ci.d = list(renderer.K.ravel()), [0.] * 5
    ci.width, ci.height, ci.header.frame_id = renderer.W, renderer.H, "optical"
    node._on_info(ci)
    node.hist.add(1.6, (0, 0, 0.85, 0))
    arr = renderer.render(0.85, 0.2, 0, 0.48, 200, 75, 215, np.random.default_rng(0))
    m = Image()
    m.height, m.width, m.step, m.encoding, m.data = renderer.H, renderer.W, renderer.W, "mono8", arr.tobytes()
    m.header.frame_id, m.header.stamp = "optical", make_time(1.8)
    node._on_img(m)
    assert len(node.waiting_images) == 1 and not published("/dh/line_obs")
    pose = PoseStamped()
    pose.header.frame_id, pose.header.stamp = "course", make_time(1.8)
    pose.pose.position.z = 0.85
    node._on_pose(pose)
    assert abs(published("/dh/line_obs")[-1][1].vector.x - 0.2) < 0.01


def test_unit02_observed_calibration_selects_factory_tf():
    import dh_autonomy.stereo_scan_node as ssn
    from dh_core.camera_geom import quat_to_rot
    from fake_ros import CameraInfo
    fresh(rectify_source="auto")
    node = ssn.StereoScanNode()
    for side,k,p03 in [("l",[595.195068359375,0,487.35308837890625,0,595.3043212890625,305.8476867675781,0,0,1],-44.8249474391155),("r",[597.6473388671875,0,481.54364013671875,0,597.5056762695312,298.5870361328125,0,0,1],0)]:
        ci = CameraInfo()
        ci.width,ci.height = 960,600
        ci.header.frame_id = "oak_"+("left" if side=="l" else "right")+"_camera_optical_frame"
        ci.k,ci.d,ci.r = k,[0.]*14,list(np.eye(3).ravel())
        ci.p = [k[0],0,k[2],p03,0,k[4],k[5],0,0,0,1,0]
        node._on_info(side,ci)
    R=quat_to_rot(-0.0011172620574881663,0.00011568005459286838,-0.00159471026885414916,0.9999980976195801)
    T=np.array([-0.07531135559082058,0.000600247979164914,0.00023905953392322488])
    node._extrinsics_from_tf=lambda:(R,T)
    assert node._rectification((960,600))[-1]=="tf"
    assert node._build_maps(960,600)
    assert abs(node.B - np.linalg.norm(T))<1e-6


if __name__ == "__main__":
    import traceback
    failed = 0
    tests = [(n, f) for n, f in list(globals().items()) if n.startswith("test_")]
    for name, fn in tests:
        try:
            fn()
            print("PASS", name)
        except Exception:
            failed += 1
            print("FAIL", name)
            traceback.print_exc()
    print("safety regressions: %d passed, %d failed" % (len(tests) - failed, failed))
    raise SystemExit(bool(failed))
