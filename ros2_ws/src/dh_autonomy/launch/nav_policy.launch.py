"""CAD-free navigation policy on the organizers' interface (autonomy_pipeline.md, step 3).

ONE launch file starts exactly one flight_interface (auto_offboard_on_arm) and exactly one command
source (nav_policy). Use it INSTEAD of the team's current launch file, never alongside it.

  ros2 launch dh_autonomy nav_policy.launch.py                  # dry run (default): nothing reaches PX4
  ros2 launch dh_autonomy nav_policy.launch.py dry_run:=false   # live: needs team approval + geofence in the yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("dh_autonomy"), "config", "nav_policy.yaml")
    dry = LaunchConfiguration("dry_run")
    return LaunchDescription([
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("with_flight_interface", default_value="true",
                              description="false only if another launch already runs THE flight_interface"),
        Node(package="skybeta_flight_interface", executable="flight_interface", name="flight_interface",
             output="screen", emulate_tty=True, parameters=[{"auto_offboard_on_arm": True}],
             condition=IfCondition(LaunchConfiguration("with_flight_interface"))),
        Node(package="dh_autonomy", executable="nav_policy", name="nav_policy", output="screen", emulate_tty=True,
             parameters=[cfg, {"dry_run": dry}]),
    ])
