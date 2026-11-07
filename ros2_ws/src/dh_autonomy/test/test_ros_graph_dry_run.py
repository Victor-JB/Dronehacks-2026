"""ROS 2 launch test (real DDS): the mission node in DRY RUN never touches the flight-control path.

NOT RUN LOCALLY (no ROS 2 on the development laptop). Run on the Jetson / in the organizers' container:
    export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1      # isolate from the organizers' stack and other drones
    export DH_COURSE_OBJ=/root/team_ws/course/dronehacks-course-v3.obj
    colcon test --packages-select dh_autonomy --pytest-args -k ros_graph && colcon test-result --verbose
The test feeds odometry on a TEST topic (/dh_test/odom), so even a misconfigured domain cannot mix it with
real cuVSLAM, and it asserts over the live ROS graph that:
  * /dh/dry_run/cmd_vel receives commands after /dh/start;
  * no dh_* node has a publisher or a service client on /skybeta/* or /fmu/*;
  * nothing at all is published on /skybeta/setpoint/twist during the test.
"""
import math
import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy

COURSE = os.environ.get("DH_COURSE_OBJ", "/root/team_ws/course/dronehacks-course-v3.obj")


pytestmark = pytest.mark.skipif(
    os.environ.get("ROS_DOMAIN_ID") != "77" or os.environ.get("ROS_LOCALHOST_ONLY") != "1",
    reason="synthetic ROS publishers require ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1")

@pytest.mark.launch_test
def generate_test_description():
    mission = launch_ros.actions.Node(
        package="dh_autonomy", executable="mission", name="dh_mission", output="screen",
        parameters=[{"course_obj": COURSE, "dry_run": True, "odom_topic": "/dh_test/odom",
                     "align_mode": "first_odom", "require_forward_clearance": False, "require_vo_status": False}])
    return launch.LaunchDescription([mission, launch_testing.actions.ReadyToTest()]), {"mission": mission}


class TestDryRunGraph(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("dh_test_driver")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_commands_only_on_dry_run_topic(self):
        from geometry_msgs.msg import TwistStamped
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Empty
        n = self.node
        got, forbidden_seen = [], []
        n.create_subscription(TwistStamped, "/dh/dry_run/cmd_vel", lambda m: got.append(m), 10)
        n.create_subscription(TwistStamped, "/skybeta/setpoint/twist", lambda m: forbidden_seen.append(m), 10)
        pub_odom = n.create_publisher(Odometry, "/dh_test/odom", 10)
        pub_start = n.create_publisher(Empty, "/dh/start", 10)
        t0 = time.time()
        started = False
        while time.time() - t0 < 25.0:                 # planning takes a few seconds on the Orin
            o = Odometry()
            o.header.stamp = n.get_clock().now().to_msg()
            o.header.frame_id = "odom"
            o.child_frame_id = "base_link"
            o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(0.785), math.cos(0.785)
            pub_odom.publish(o)
            if not started and time.time() - t0 > 12.0:
                pub_start.publish(Empty()); started = True
            rclpy.spin_once(n, timeout_sec=0.03)
            if len(got) > 20:
                break
        self.assertGreater(len(got), 20, "no dry-run commands received")
        self.assertEqual(len(forbidden_seen), 0, "something was published on /skybeta/setpoint/twist")
        bad = []
        for name, ns in n.get_node_names_and_namespaces():
            if not name.startswith("dh_") or name == "dh_test_driver":
                continue
            for topic, _ in n.get_publisher_names_and_types_by_node(name, ns):
                if topic.startswith("/skybeta") or topic.startswith("/fmu"):
                    bad.append((name, topic))
            for srv, _ in n.get_client_names_and_types_by_node(name, ns):
                if srv.startswith("/skybeta") or srv.startswith("/fmu"):
                    bad.append((name, srv))
        self.assertEqual(bad, [], "dry-run node owns flight-control endpoints")


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
