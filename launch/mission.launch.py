"""Launch skybeta_flight_interface plus the team mission (dronehacks_nav).

Modelled on ~/skybeta/src/competitor_example/launch/competitor.launch.py: the
``flight_interface`` node is kept exactly as there (it owns the Offboard
heartbeat, the safety envelope and the arm -> Position -> Offboard handover),
and ``square_mission`` is swapped for ours.

    ros2 launch dronehacks_nav mission.launch.py
    ros2 launch dronehacks_nav mission.launch.py max_altitude_m:=2.0 \
        route_file:=/root/team_ws/src/dronehacks/route.json

Set ``mission:=false`` to bring up only the flight interface. Tighten the
envelope from here; do not loosen it without asking the organisers.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

SHARE = FindPackageShare('dronehacks_nav')

# name -> (default, help). Envelope values are the flight interface's own
# defaults, repeated here so they can be tightened from one command line.
ARGUMENTS = [
    ('max_altitude_m', '3.0', 'ceiling above the odometry origin, metres'),
    ('min_altitude_m', '0.3', 'floor above the odometry origin, metres'),
    ('max_speed_mps', '1.5', 'cap on the norm of a twist setpoint, m/s'),
    ('setpoint_rate_hz', '10.0',
     'OffboardControlMode heartbeat rate -- PX4 needs > 2 Hz'),
    ('allow_mode_request', 'false',
     'enable /skybeta/request_offboard. SIM ONLY: on a real vehicle Offboard '
     'follows the pilot arming, then switching to Position'),
    ('auto_offboard_on_arm', 'true',
     'enter Offboard once when the pilot arms, then switches to Position. false: the vehicle '
     'stays in Position under the pilot'),
    ('odometry_topic', '/visual_slam/tracking/odometry',
     'source of the ENU pose the altitude band is measured against'),
    ('route_file', PathJoinSubstitution([SHARE, 'route.json']),
     'mission route and params (refnav format)'),
    ('params_file', PathJoinSubstitution([SHARE, 'config', 'drone.yaml']),
     'mission ROS parameters for this drone'),
    ('mission', 'true', 'also start the mission node'),
]


def generate_launch_description():
    declared = [
        DeclareLaunchArgument(name, default_value=default, description=help_)
        for name, default, help_ in ARGUMENTS
    ]

    flight_interface = Node(
        package='skybeta_flight_interface',
        executable='flight_interface',
        name='flight_interface',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'max_altitude_m': LaunchConfiguration('max_altitude_m'),
            'min_altitude_m': LaunchConfiguration('min_altitude_m'),
            'max_speed_mps': LaunchConfiguration('max_speed_mps'),
            'setpoint_rate_hz': LaunchConfiguration('setpoint_rate_hz'),
            'allow_mode_request': LaunchConfiguration('allow_mode_request'),
            'auto_offboard_on_arm': LaunchConfiguration('auto_offboard_on_arm'),
            'odometry_topic': LaunchConfiguration('odometry_topic'),
        }],
    )

    mission = Node(
        package='dronehacks_nav',
        executable='mission',
        name='dronehacks_nav',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('mission')),
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'route_file': LaunchConfiguration('route_file'),
                'odometry_topic': LaunchConfiguration('odometry_topic'),
            },
        ],
    )

    return LaunchDescription(declared + [flight_interface, mission])
