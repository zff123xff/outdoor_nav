import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'task_pipeline'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@example.com',
    description='Task pipeline senders and shared YAML task goals.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'send_point_goal = task_pipeline.send_point_goal:main',
            'send_patrol = task_pipeline.send_patrol:main',
            'pipeline_dispatcher = task_pipeline.dispatcher:main',
            'mqtt_task_gateway = task_pipeline.mqtt_task_gateway:main',
            'send_area_search = task_pipeline.send_area_search:main',
            'area_search = task_pipeline.area_search:main',
        ],
    },
)
