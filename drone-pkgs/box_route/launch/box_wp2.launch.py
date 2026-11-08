from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Short mode: climb, waypoint 1, waypoint 2, then settle and land AT waypoint 2.
        # Same as box_mission.launch.py except last_waypoint. The stack starts it with
        # COMPETITOR_PKG=box_route, COMPETITOR_LAUNCH=box_wp2.launch.py in /etc/skybeta/drone.env.
        # Exactly ONE flight_interface may run.
        Node(package='skybeta_flight_interface', executable='flight_interface',
             name='flight_interface', output='screen', emulate_tty=True,
             parameters=[{'auto_offboard_on_arm': True,      # without this: no takeover
                          # setpoints re-anchored on PX4's local position (cancels the
                          # VIO-vs-PX4 height offset), same as test_files
                          'anchor_setpoints': True}]),
        Node(package='box_route', executable='box_mission',
             name='box_mission', output='screen', emulate_tty=True,
             parameters=[{'last_waypoint': 2}]),
    ])
