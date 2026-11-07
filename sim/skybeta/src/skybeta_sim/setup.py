from glob import glob
from setuptools import setup

setup(
    name='skybeta_sim',
    version='0.1.0',
    packages=['skybeta_sim'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/skybeta_sim']),
        ('share/skybeta_sim', ['package.xml']),
        ('share/skybeta_sim/launch', glob('launch/*.py')),
        ('share/skybeta_sim/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'vio_sim = skybeta_sim.vio_sim:main',
        'oak_sim = skybeta_sim.oak_sim:main',
        'pilot = skybeta_sim.pilot:main',
    ]},
)
