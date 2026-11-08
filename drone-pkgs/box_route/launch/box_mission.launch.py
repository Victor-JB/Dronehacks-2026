from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Exactly ONE flight_interface may run. This launch file is the one the stack starts
        # (COMPETITOR_PKG=box_route, COMPETITOR_LAUNCH=box_mission.launch.py in /etc/skybeta/drone.env).
        Node(package='skybeta_flight_interface', executable='flight_interface',
             name='flight_interface', output='screen', emulate_tty=True,
             parameters=[{'auto_offboard_on_arm': True,      # without this: no takeover
                          # setpoints re-anchored on PX4's local position (cancels the
                          # VIO-vs-PX4 height offset), same as test_files
                          'anchor_setpoints': True}]),
        Node(package='box_route', executable='box_mission',
             name='box_mission', output='screen', emulate_tty=True),
    ])
