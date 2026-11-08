from glob import glob
from setuptools import setup

pkg = "dh_autonomy"
setup(
    name=pkg,
    version="0.1.0",
    packages=[pkg],
    data_files=[("share/ament_index/resource_index/packages", ["resource/" + pkg]),
                ("share/" + pkg, ["package.xml"]),
                ("share/" + pkg + "/launch", glob("launch/*.py")),
                ("share/" + pkg + "/config", glob("config/*.yaml"))],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="DroneHacks team",
    description="DroneHacks ROS 2 nodes.",
    license="MIT",
    entry_points={"console_scripts": [
        "mission = dh_autonomy.mission_node:main",
        "stationary_mission = dh_autonomy.mission_node:stationary_main",
        "localizer = dh_autonomy.localizer_node:main",
        "stereo_scan = dh_autonomy.stereo_scan_node:main",
        "line = dh_autonomy.line_node:main",
        "sitl_odom = dh_autonomy.sitl_odom_node:main",
        "nav_policy = dh_autonomy.nav_policy_node:main",
    ]},
)
