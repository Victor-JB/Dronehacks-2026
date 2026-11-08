"""Actual Humble/DDS startup test, deliberately separate from frozen Phase2.

NOT executed on the Windows host. Requires installed team dronehacks_nav and
Isaac status interface. Source/test stamps and TF stay on synthetic test topics.
"""
import json
import os
import time
import unittest
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy

pytestmark=pytest.mark.skipif(os.environ.get('ROS_DOMAIN_ID')!='78' or os.environ.get('ROS_LOCALHOST_ONLY')!='1',
    reason='Require isolated ROS_DOMAIN_ID=78 ROS_LOCALHOST_ONLY=1')


@pytest.mark.launch_test
def generate_test_description():
    node=launch_ros.actions.Node(package='dh_autonomy',executable='local_policy_stationary',
        name='dh_local_policy_test',output='screen',parameters=[dict(dry_run=False,
            depth_geometry_verified=True,below_wall_height_verified=True,
            legal_min_altitude=.35,legal_max_altitude=1.5,cruise_z=.8,
            voxel_m=.2,size_xy_m=6.,integrate_hz=10.,
            depth_topic='/dh_policy_test/depth',depth_info_topic='/dh_policy_test/info',
            odometry_topic='/dh_policy_test/odom',vo_status_topic='/dh_policy_test/vo')],
        remappings=[('/dh/start','/dh_policy_test/start'),
                    ('/dh/local_policy/status','/dh_policy_test/status'),
                    ('/dh/dry_run/cmd_vel','/dh_policy_test/cmd')])
    return launch.LaunchDescription([node,launch_testing.actions.ReadyToTest()])


class TestLocalPolicyROS(unittest.TestCase):
    def test_native_depth_hold_and_stationary_endpoints(self):
        import numpy as np
        from sensor_msgs.msg import Image,CameraInfo
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String,Empty
        from geometry_msgs.msg import TransformStamped
        from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
        from rclpy.qos import qos_profile_sensor_data as sensor
        from rclpy.time import Time
        from tf2_ros import StaticTransformBroadcaster
        rclpy.init(); driver=rclpy.create_node('dh_policy_driver')
        try:
            statuses=[]
            driver.create_subscription(String,'/dh_policy_test/status',lambda m:statuses.append(json.loads(m.data)),10)
            depth_pub=driver.create_publisher(Image,'/dh_policy_test/depth',sensor)
            info_pub=driver.create_publisher(CameraInfo,'/dh_policy_test/info',sensor)
            odom_pub=driver.create_publisher(Odometry,'/dh_policy_test/odom',10)
            vo_pub=driver.create_publisher(VisualSlamStatus,'/dh_policy_test/vo',sensor)
            start_pub=driver.create_publisher(Empty,'/dh_policy_test/start',10)
            tf=TransformStamped(); tf.header.frame_id='base_link'; tf.child_frame_id='dh_policy_depth_optical'
            tf.transform.rotation.x=-.5; tf.transform.rotation.y=.5; tf.transform.rotation.z=-.5; tf.transform.rotation.w=.5
            tf.transform.translation.x=.1053; tf.transform.translation.z=.0304
            broadcaster=StaticTransformBroadcaster(driver); broadcaster.sendTransform(tf)
            end=time.monotonic()+12.
            while time.monotonic()<end and not any(s.get('startup_reason')=='WAIT_MEASURED_FULL_START_VOLUME' for s in statuses):
                now=driver.get_clock().now(); stamp=now.to_msg()
                odom=Odometry(); odom.header.stamp=stamp; odom.header.frame_id='odom'; odom.child_frame_id='base_link'
                odom.pose.pose.position.z=.8; odom.pose.pose.orientation.w=1.; odom_pub.publish(odom)
                vo=VisualSlamStatus(); vo.header.stamp=stamp; vo.vo_state=1; vo_pub.publish(vo)
                capture=Time(nanoseconds=now.nanoseconds-50_000_000).to_msg()
                info=CameraInfo(); info.header.stamp=capture; info.header.frame_id='dh_policy_depth_optical'
                info.width=160; info.height=100; info.k=[100.,0.,80.,0.,100.,50.,0.,0.,1.]; info_pub.publish(info)
                depth=Image(); depth.header=info.header; depth.width=160; depth.height=100
                depth.encoding='16UC1'; depth.step=320; depth.is_bigendian=0
                depth.data=np.full((100,160),3000,dtype='<u2').tobytes(); depth_pub.publish(depth)
                start_pub.publish(Empty())  # must be refused with an unobserved starting volume
                rclpy.spin_once(driver,timeout_sec=.03)
            self.assertTrue(any(s.get('startup_reason')=='WAIT_MEASURED_FULL_START_VOLUME' for s in statuses),statuses[-5:])
            self.assertFalse(any(s.get('startup_ready') for s in statuses))
            pubs=driver.get_publisher_names_and_types_by_node('dh_local_policy_test','/')
            clients=driver.get_client_names_and_types_by_node('dh_local_policy_test','/')
            self.assertTrue(any(name=='/dh_policy_test/cmd' for name,_ in pubs))
            self.assertFalse([name for name,_ in pubs+clients if name.startswith(('/skybeta','/fmu'))])
        finally:
            driver.destroy_node(); rclpy.shutdown()
