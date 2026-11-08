"""REVIEW ONLY: explicit selection, no activation by default.

Reuse the already running organizer flight_interface. Only its owner may select
start_flight_interface:=true after confirming there is no existing instance.
The legacy mission and permanently dry local_policy.launch.py are untouched.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def selected_policy(context):
    if LaunchConfiguration('activate_physical').perform(context).lower()!='true':
        raise RuntimeError('Physical policy disabled: separate approval and reviewed configuration required')
    official=os.path.join(get_package_share_directory('competitor_example'),'launch','competitor.launch.py')
    interface=IncludeLaunchDescription(PythonLaunchDescriptionSource(official),
        condition=IfCondition(LaunchConfiguration('start_flight_interface')),
        launch_arguments={'competitor':'false','allow_mode_request':'false',
            'max_altitude_m':LaunchConfiguration('interface_max_altitude_m'),
            'min_altitude_m':LaunchConfiguration('interface_min_altitude_m')}.items())
    policy=Node(package='dh_autonomy',executable='local_policy',name='dh_local_policy',
        output='screen',parameters=[LaunchConfiguration('params_file'),{'dry_run':False}])
    return [interface,policy]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('activate_physical',default_value='false'),
        DeclareLaunchArgument('start_flight_interface',default_value='false'),
        # Organizer interface limits, NOT competition/legal navigation height.
        DeclareLaunchArgument('interface_min_altitude_m',default_value='0.3'),
        DeclareLaunchArgument('interface_max_altitude_m',default_value='3.0'),
        DeclareLaunchArgument('params_file',default_value=os.path.join(
            get_package_share_directory('dh_autonomy'),'config','local_policy.physical-review.yaml')),
        OpaqueFunction(function=selected_policy)])
