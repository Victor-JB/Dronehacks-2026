from glob import glob
from setuptools import setup

package_name = 'dronehacks_nav'

setup(
    name=package_name,
    version='0.1.0',
    # refnav is the mission logic shared with the simulator branch (main);
    # dronehacks_nav is the skybeta-specific ROS shell around it.
    packages=['refnav', package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'route.json']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Victor Joulin-Batejat',
    maintainer_email='victorjb2015@gmail.com',
    description='refnav mission on the skybeta drone.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mission = dronehacks_nav.skybeta_node:main',
            'camera_web = dronehacks_nav.camera_web:main',
            'map_node = dronehacks_nav.map_node:main',
        ],
    },
)
