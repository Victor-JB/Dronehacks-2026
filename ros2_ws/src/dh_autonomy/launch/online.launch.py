"""CAD-free stationary-only pipeline; no driver/SLAM/flight-interface startup."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('planner',default_value='clothoid',choices=['clothoid','sfc','mpl']),
        DeclareLaunchArgument('params_file',default_value=os.path.join(get_package_share_directory('dh_autonomy'),'config','online.yaml')),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        Node(package='dh_autonomy',executable='online_stationary',name='dh_mission',output='screen',
             parameters=[LaunchConfiguration('params_file'),{'planner':LaunchConfiguration('planner'),
                'use_sim_time':PythonExpression(["'",LaunchConfiguration('use_sim_time'),"'.lower() == 'true'"])}])])
