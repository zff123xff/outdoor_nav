from glob import glob
from setuptools import find_packages, setup

package_name = 'servo_triger_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='Hardware PWM servo switch service with a timer-driven state machine.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'servo_triger_node = servo_triger_control.servo_triger_node:main',
    ]},
)
