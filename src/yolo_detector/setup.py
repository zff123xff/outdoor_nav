from glob import glob
from setuptools import find_packages, setup

setup(
    name='yolo_detector',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/yolo_detector']),
        ('share/yolo_detector', ['package.xml']),
        ('share/yolo_detector/config', glob('config/*.yaml')),
        ('share/yolo_detector/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@example.com',
    description='Latest-frame TensorRT YOLO detector.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'yolo_detector = yolo_detector.yolo_node:main',
        'simulation_target_marker = yolo_detector.target_marker:main',
    ]},
)
