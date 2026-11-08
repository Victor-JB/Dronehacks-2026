"""One mapper/policy; existing sensors/VO/stereo clearance providers are reused."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file',default_value=os.path.join(
            get_package_share_directory('dh_autonomy'),'config','local_policy.yaml')),
        Node(package='dh_autonomy',executable='local_policy_stationary',name='dh_local_policy',
             output='screen',parameters=[LaunchConfiguration('params_file')])])
