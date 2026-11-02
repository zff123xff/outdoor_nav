from setuptools import find_packages, setup

package_name = "mavros_odom_relay"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/mavros_odom_relay.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dev",
    maintainer_email="dev@example.com",
    description="Relay /mavros/local_position/odom (BEST_EFFORT) -> /odom (RELIABLE) for Nav2.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            f"odom_relay = {package_name}.mavros_odom_relay:main",
        ],
    },
)
