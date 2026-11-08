from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='skybeta_flight_interface', executable='flight_interface',
             name='flight_interface', output='screen', emulate_tty=True,
             parameters=[{'auto_offboard_on_arm': True,      # without this: no takeover
                          # Setpoints become displacements from the vision origin,
                          # re-anchored on PX4's local position. Cancels the
                          # VIO-vs-PX4 height offset (competitor_guide.md, frame caveat).
                          'anchor_setpoints': True}]),
        Node(package='test_files', executable='my_mission',
             name='my_mission', output='screen', emulate_tty=True),
    ])
