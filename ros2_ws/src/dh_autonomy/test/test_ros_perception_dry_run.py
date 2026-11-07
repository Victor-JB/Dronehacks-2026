"""Actual ROS 2 launch + DDS + OpenCV contract test. Requires Humble, not fake_ros.

Runs the shipped launch, synthetic calibrated image publishers and a static optical mount.
Isolate with ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1. No PX4/flight interface is launched.
"""
import math
import os
import time
import unittest

import launch
import launch.launch_description_sources
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory


pytestmark = pytest.mark.skipif(
    os.environ.get("ROS_DOMAIN_ID") != "77" or os.environ.get("ROS_LOCALHOST_ONLY") != "1",
    reason="synthetic ROS publishers require ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1")

@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory("dh_autonomy")
    stack = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(share, "launch", "autonomy.launch.py")),
        launch_arguments={
            "course_obj": os.environ["DH_COURSE_OBJ"], "dry_run": "true",
            "align_mode": "first_odom", "odom_topic": "/dh_test/odom",
            # cuVSLAM is outside this test: synthetic odometry, no GPU estimator.
            "require_vo_status": "false",
        }.items())
    mount = launch_ros.actions.Node(
        package="tf2_ros", executable="static_transform_publisher",
        arguments=["--x", "0.1", "--y", "0", "--z", "0",
                   "--qx", "-0.5", "--qy", "0.5", "--qz", "-0.5", "--qw", "0.5",
                   "--frame-id", "base_link", "--child-frame-id", "dh_test_optical"])
    return launch.LaunchDescription([mount, stack, launch_testing.actions.ReadyToTest()])


class TestPerceptionContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("dh_test_images")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_real_images_clearance_and_isolation(self):
        from geometry_msgs.msg import TwistStamped
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data as sensor
        from sensor_msgs.msg import CameraInfo, Image, LaserScan, Range
        from std_msgs.msg import Empty
        n = self.node
        scans, clearances, commands = [], [], []
        n.create_subscription(LaserScan, "/dh/stereo_scan", scans.append, sensor)
        n.create_subscription(Range, "/dh/forward_clearance", clearances.append, sensor)
        n.create_subscription(TwistStamped, "/dh/dry_run/cmd_vel", commands.append, 10)
        images = [n.create_publisher(Image, "/oak/" + side + "/image_raw", sensor) for side in ("left", "right")]
        infos = [n.create_publisher(CameraInfo, "/oak/" + side + "/camera_info", sensor) for side in ("left", "right")]
        odom = n.create_publisher(Odometry, "/dh_test/odom", sensor)
        start = n.create_publisher(Empty, "/dh/start", 10)
        width, height, f, baseline, disparity = 640, 400, 350.0, 0.075, 20
        rng = np.random.default_rng(123)
        left = rng.integers(20, 235, (height, width), dtype=np.uint8)
        right = np.empty_like(left)
        right[:, :-disparity], right[:, -disparity:] = left[:, disparity:], left[:, -disparity:]
        stamps = set()

        def feed(flat=False):
            stamp = n.get_clock().now().to_msg()
            stamps.add((stamp.sec, stamp.nanosec))
            o = Odometry()
            o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, "odom", "base_link"
            o.pose.pose.orientation.w = 1.0
            odom.publish(o)
            for i in range(2):
                ci = CameraInfo()
                ci.header.stamp, ci.header.frame_id = stamp, "dh_test_optical"
                ci.width, ci.height = width, height
                ci.k = [f, 0., width/2, 0., f, height/2, 0., 0., 1.]
                ci.d, ci.r = [0.] * 5, [1., 0., 0., 0., 1., 0., 0., 0., 1.]
                ci.p = [f, 0., width/2, -f*baseline if i else 0., 0., f, height/2, 0., 0., 0., 1., 0.]
                infos[i].publish(ci)
                im = Image()
                im.header = ci.header
                im.width, im.height, im.step, im.encoding = width, height, width, "mono8"
                im.data = (np.full_like(left, 128) if flat else (right if i else left)).tobytes()
                images[i].publish(im)

        deadline = time.monotonic() + 30
        while time.monotonic() < deadline and not any(math.isfinite(m.range) for m in clearances):
            feed()
            rclpy.spin_once(n, timeout_sec=0.05)
        finite = [m for m in clearances if math.isfinite(m.range)]
        self.assertTrue(finite, "no depth through real DDS, TF and OpenCV")
        self.assertTrue(scans)
        self.assertEqual(scans[-1].header.frame_id, "base_link")
        self.assertIn((scans[-1].header.stamp.sec, scans[-1].header.stamp.nanosec), stamps)
        self.assertAlmostEqual(finite[-1].range, f*baseline/disparity + 0.1, delta=0.18)
        self.assertGreater(n.count_subscribers("/dh/start"), 0)
        start.publish(Empty())
        # Flat images make SGBM unknown. Drive that through DDS to explicit all-axis hold.
        begin = time.monotonic()
        while time.monotonic() - begin < 2.0:
            feed(flat=True)
            rclpy.spin_once(n, timeout_sec=0.05)
        self.assertTrue(any(math.isnan(m.range) for m in clearances))
        self.assertTrue(commands, "mission did not receive the dry-run start")
        v = commands[-1].twist
        self.assertEqual([v.linear.x, v.linear.y, v.linear.z, v.angular.z], [0.] * 4)
        names = {name for name, _ in n.get_node_names_and_namespaces()}
        self.assertTrue({"dh_mission", "dh_localizer", "dh_stereo_scan", "dh_line"} <= names)
        for name, namespace in n.get_node_names_and_namespaces():
            if name not in ("dh_mission", "dh_localizer", "dh_stereo_scan", "dh_line"):
                continue
            endpoints = n.get_publisher_names_and_types_by_node(name, namespace)
            endpoints += n.get_client_names_and_types_by_node(name, namespace)
            self.assertFalse([t for t, _ in endpoints if t.startswith(("/skybeta", "/fmu"))])
