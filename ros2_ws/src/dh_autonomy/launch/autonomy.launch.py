"""Launch the DroneHacks autonomy stack next to the organizers' Sky Beta stack.

DEFAULT IS DRY RUN: dry_run:=true routes every velocity command to /dh/dry_run/cmd_vel and the mission node
refuses to start if it owns any publisher/client on /skybeta/* or /fmu/* (enforced in code).

  # stationary hardware-in-the-loop (props off, flight battery out):
  ros2 launch dh_autonomy autonomy.launch.py course_obj:=/path/course.obj align_mode:=first_odom
  # recorded-data playback (ros2 bag play <bag> --clock in another shell):
  ros2 launch dh_autonomy autonomy.launch.py course_obj:=/path/course.obj use_sim_time:=true
  # flight, ONLY when flying is authorised:
  ros2 launch dh_autonomy autonomy.launch.py course_obj:=/path/course.obj dry_run:=false
Then the single start command:  ros2 topic pub --once /dh/start std_msgs/msg/Empty {}

Start order does not matter: every node tolerates missing inputs (the mission publishes nothing to the flight
interface until odometry, alignment and Offboard are all valid; the localizer waits for /dh/pose_map; the
stereo node publishes NaN clearance until camera_info and rectification are ready).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _bool(name):
    return PythonExpression(["'", LaunchConfiguration(name), "'.lower() == 'true'"])


def generate_launch_description(mission_executable="mission"):
    cfg = LaunchConfiguration("params_file")
    common = {"course_obj": LaunchConfiguration("course_obj"), "use_sim_time": _bool("use_sim_time")}
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=os.path.join(
            get_package_share_directory("dh_autonomy"), "config", "competition.yaml")),
        DeclareLaunchArgument("course_obj", default_value="/root/team_ws/course/dronehacks-course-v3.obj"),
        DeclareLaunchArgument("dry_run", default_value="true",
                              description="true: NO publisher to /skybeta or /fmu; commands -> /dh/dry_run/cmd_vel"),
        DeclareLaunchArgument("backend", default_value="skybeta", description="skybeta | px4 (SITL only)"),
        DeclareLaunchArgument("route", default_value="window"),
        DeclareLaunchArgument("align_mode", default_value="start",
                              description="start | first_odom (stationary tests: align as soon as odometry arrives)"),
        DeclareLaunchArgument("use_stereo", default_value="true"),
        DeclareLaunchArgument("require_forward_clearance", default_value="true",
                              description="Keep true; false only for explicit offline fixtures without stereo"),
        DeclareLaunchArgument("require_vo_status", default_value="true"),
        DeclareLaunchArgument("use_line", default_value="true"),
        DeclareLaunchArgument("use_localizer", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="true for ros2 bag play --clock"),
        DeclareLaunchArgument("odom_topic", default_value="/visual_slam/tracking/odometry"),
        DeclareLaunchArgument("enable_return", default_value="false"),
        Node(package="dh_autonomy", executable=mission_executable, name="dh_mission", output="screen",
             parameters=[cfg, common, {
                 "dry_run": _bool("dry_run"),
                 "backend": LaunchConfiguration("backend"),
                 "route": LaunchConfiguration("route"),
                 "align_mode": LaunchConfiguration("align_mode"),
                 "odom_topic": LaunchConfiguration("odom_topic"),
                 "enable_return": _bool("enable_return"),
                 # without the stereo node there is NO forward obstacle information: make that explicit
                 "require_forward_clearance": _bool("require_forward_clearance"),
                 "require_vo_status": _bool("require_vo_status")}]),
        Node(package="dh_autonomy", executable="localizer", name="dh_localizer", output="screen",
             parameters=[cfg, common], condition=IfCondition(LaunchConfiguration("use_localizer"))),
        Node(package="dh_autonomy", executable="stereo_scan", name="dh_stereo_scan", output="screen",
             parameters=[cfg, {"use_sim_time": _bool("use_sim_time")}],
             condition=IfCondition(LaunchConfiguration("use_stereo"))),
        Node(package="dh_autonomy", executable="line", name="dh_line", output="screen",
             parameters=[cfg, {"use_sim_time": _bool("use_sim_time")}],
             condition=IfCondition(LaunchConfiguration("use_line"))),
    ])
