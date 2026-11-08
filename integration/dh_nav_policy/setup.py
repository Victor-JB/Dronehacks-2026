from glob import glob
from setuptools import setup

package_name = 'dh_nav_policy'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.navstack', package_name + '.navstack.nav',
              package_name + '.navstack.shadow'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='DroneHacks team',
    maintainer_email='sakethdesu@tamu.edu',
    description='CAD-free exploration policy on the skybeta interface.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'nav_policy = dh_nav_policy.node:main',
        ],
    },
)
