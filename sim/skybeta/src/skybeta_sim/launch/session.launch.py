"""One flight, as `fly.sh start` runs it on the drone, plus the scripted pilot.

    ros2 launch skybeta_sim session.launch.py params_file:=<drone.yaml with use_sim_time: true>

Includes the team's own dronehacks_nav mission.launch.py (skybeta_flight_interface + mission node)
unchanged; only use_sim_time is set, because PX4 SITL runs on Gazebo's clock.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', description='mission ROS parameters, with use_sim_time: true'),
        GroupAction([
            SetParameter(name='use_sim_time', value=True),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [FindPackageShare('dronehacks_nav'), 'launch', 'mission.launch.py'])),
                launch_arguments={'params_file': LaunchConfiguration('params_file')}.items()),
            Node(package='skybeta_sim', executable='pilot', name='pilot', output='screen'),
        ]),
    ])
