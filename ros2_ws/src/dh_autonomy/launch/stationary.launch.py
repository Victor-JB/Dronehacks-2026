"""Stationary pipeline with a mission executable that cannot construct physical command backends.

No physical node is started by this launch. Sensor providers must already be running.
Do not deploy or run this on hardware without a separate controlled-deployment review.
"""
import os
import runpy
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    source = os.path.join(get_package_share_directory("dh_autonomy"), "launch", "autonomy.launch.py")
    return runpy.run_path(source)["generate_launch_description"](mission_executable="stationary_mission")
