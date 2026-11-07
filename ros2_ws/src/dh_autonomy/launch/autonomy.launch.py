"""Launch the DroneHacks autonomy stack next to the organizers' skybeta stack.

ros2 launch dh_autonomy autonomy.launch.py course_obj:=/path/course.obj backend:=skybeta
Then send the single start command:  ros2 topic pub --once /dh/start std_msgs/msg/Empty {}
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("dh_autonomy"), "config", "competition.yaml")
    course = LaunchConfiguration("course_obj")
    over = {"course_obj": course}
    return LaunchDescription([
        DeclareLaunchArgument("course_obj", default_value="/root/team_ws/course/dronehacks-course-v3.obj"),
        DeclareLaunchArgument("backend", default_value="skybeta"),
        DeclareLaunchArgument("route", default_value="window"),
        DeclareLaunchArgument("use_stereo", default_value="true"),
        DeclareLaunchArgument("use_line", default_value="true"),
        DeclareLaunchArgument("odom_topic", default_value="/visual_slam/tracking/odometry"),
        DeclareLaunchArgument("enable_return", default_value="false"),
        Node(package="dh_autonomy", executable="mission", name="dh_mission", output="screen",
             parameters=[cfg, over, {"backend": LaunchConfiguration("backend"), "route": LaunchConfiguration("route"),
                                     "odom_topic": LaunchConfiguration("odom_topic"),
                                     "enable_return": PythonExpression(["'", LaunchConfiguration("enable_return"), "' == 'true'"])}]),
        Node(package="dh_autonomy", executable="localizer", name="dh_localizer", output="screen",
             parameters=[cfg, over]),
        Node(package="dh_autonomy", executable="stereo_scan", name="dh_stereo_scan", output="screen",
             parameters=[cfg], condition=IfCondition(LaunchConfiguration("use_stereo"))),
        Node(package="dh_autonomy", executable="line", name="dh_line", output="screen",
             parameters=[cfg], condition=IfCondition(LaunchConfiguration("use_line"))),
    ])
