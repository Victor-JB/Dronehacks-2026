from glob import glob
from setuptools import setup

package_name = 'box_route_v2'

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
    description='Course route v2: corrected waypoints from the 2026-11-08 data (v1 = box_route).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'box_mission = box_route_v2.box_mission:main',
            'live_view = box_route_v2.live_view:main',
        ],
    },
)
