import os
from glob import glob

from setuptools import find_packages, setup

package_name = "robot_launch"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml") + glob("config/*.rviz"),
        ),
        (os.path.join("share", package_name, "maps"), glob("maps/*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="todo",
    maintainer_email="todo@example.com",
    description="Nav2-related launch and config for this workspace.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "auto_set_gp_origin = robot_launch.auto_set_gp_origin:main",
            "lifecycle_service_relay = robot_launch.lifecycle_service_relay:main",
            "lifecycle_startup_watchdog = robot_launch.lifecycle_startup_watchdog:main",
        ],
    },
)
