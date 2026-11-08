from glob import glob
from setuptools import setup

package_name = 'box_route'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='dronehacks',
    maintainer_email='sakethdesu@tamu.edu',
    description='Course route: climb 1.08 m, 13 odom-map waypoints (walk_1107), land.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'box_mission = box_route.box_mission:main',
            'live_view = box_route.live_view:main',
        ],
    },
)
