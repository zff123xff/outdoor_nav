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
        *[(os.path.join('share', package_name, os.path.dirname(path)), [path])
          for path in glob('config/**/*.yaml', recursive=True)],
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@example.com',
    description='FIFO primitive dispatch, independent target inspection, MQTT ingress and example publishers.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'send_point_goal = task_pipeline.examples.sender:send_point_goal',
            'multi_point_executor = task_pipeline.multi_point_executor:main',
            'send_multi_point = task_pipeline.examples.sender:send_multi_point',
            'send_patrol = task_pipeline.examples.sender:send_patrol',
            'pipeline_dispatcher = task_pipeline.dispatcher:main',
            'mqtt_task_gateway = task_pipeline.mqtt_task_gateway:main',
            'inspection_udp_bridge = task_pipeline.inspection_udp_bridge:main',
            'target_inspection_executor = task_pipeline.inspection_executor:main',
            'search_inspection_executor = task_pipeline.search_inspection_executor:main',
            'send_target_inspection = task_pipeline.examples.sender:send_target_inspection',
            'send_area_search = task_pipeline.examples.sender:send_area_search',
        ],
    },
)
