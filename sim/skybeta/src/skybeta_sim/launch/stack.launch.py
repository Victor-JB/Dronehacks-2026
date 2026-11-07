"""What skybeta-stack.service runs on the drone, with the OAK driver and cuVSLAM replaced by Gazebo.

  ros_gz_bridge      /clock, ground truth, rendered OAK  (sim only)
  oak_sim            /oak/stereo/image_raw + camera_info, /oak/left/image_mono, OAK TFs
  vio_sim            /visual_slam/tracking/odometry + /visual_slam/status  (cuVSLAM's outputs)
  px4_odom_bridge    the drone's own bridge -> /fmu/in/vehicle_visual_odometry

MicroXRCEAgent is started next to this by sim/skybeta/start.sh, as on the drone. Every node runs
on Gazebo's clock: PX4 SITL keeps sim time, so all timestamps share one time base.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_share = FindPackageShare('skybeta_sim')
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge', output='screen',
             parameters=[{'config_file': PathJoinSubstitution([sim_share, 'config', 'gz_bridge.yaml'])}]),
        Node(package='skybeta_sim', executable='oak_sim', name='oak_sim', output='screen'),
        Node(package='skybeta_sim', executable='vio_sim', name='vio_sim', output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('oakd_vslam_bringup'), 'launch', 'px4_bridge.launch.py'])),
            launch_arguments={'pose_frame': 'FRD'}.items()),
    ])
