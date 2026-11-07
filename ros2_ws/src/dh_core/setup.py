from setuptools import setup

setup(
    name="dh_core",
    version="0.1.0",
    packages=["dh_core"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/dh_core"]),
                ("share/dh_core", ["package.xml"])],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="DroneHacks team",
    description="DroneHacks core library (numpy only).",
    license="MIT",
    tests_require=["pytest"],
)
