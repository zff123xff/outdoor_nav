"""Test fixture composing the two independently owned workspaces."""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('px4_gps_imu'), 'launch', 'go2_px4_sim.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('robot_launch'), 'launch', 'simulation.launch.py')), launch_arguments={'rviz': 'false'}.items()),
    ])
