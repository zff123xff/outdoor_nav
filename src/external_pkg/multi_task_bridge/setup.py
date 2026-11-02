from setuptools import find_packages, setup

package_name = "multi_task_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/multi_task_bridge.launch.py"]),
    ],
    install_requires=["setuptools", "numpy", "shapely", "pymap3d"],
    zip_safe=True,
    maintainer="dev",
    maintainer_email="dev@example.com",
    description="Bridge MultiTask backend messages to agent ExecuteTask service",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            f"multi_task_bridge = {package_name}.node:main",
        ],
    },
)
