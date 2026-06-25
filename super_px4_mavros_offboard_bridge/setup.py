from glob import glob
from setuptools import find_packages, setup


package_name = "super_px4_mavros_offboard_bridge"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="super_ws maintainer",
    maintainer_email="user@example.com",
    description="ROS 2 MAVROS bridge for PX4 offboard control with SUPER planner setpoints.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "offboard_bridge_node = super_px4_mavros_offboard_bridge.offboard_bridge_node:main",
        ],
    },
)
