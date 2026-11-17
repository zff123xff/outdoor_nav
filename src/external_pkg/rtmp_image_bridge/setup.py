from glob import glob
from setuptools import find_packages, setup

setup(
    name='rtmp_image_bridge', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/rtmp_image_bridge']),
        ('share/rtmp_image_bridge', ['package.xml', 'README.md']),
        ('share/rtmp_image_bridge/config', glob('config/*.yaml')),
        ('share/rtmp_image_bridge/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='dev', maintainer_email='dev@example.com', license='Apache-2.0',
    description='RTMP video to ROS Image with reconnect and a latest-frame buffer.',
    entry_points={'console_scripts': ['rtmp_image_bridge = rtmp_image_bridge.node:main']},
)
