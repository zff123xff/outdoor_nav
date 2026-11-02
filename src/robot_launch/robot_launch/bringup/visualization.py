"""RViz configuration for the common navigation interfaces."""
import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def namespaced_rviz_config(source, namespace):
    """Resolve RViz's default robot frames and topics for this robot instance."""
    with open(source) as stream:
        config = yaml.safe_load(stream)

    def rewrite(value):
        if isinstance(value, dict):
            return {key: rewrite(item) for key, item in value.items()}
        if isinstance(value, list):
            return [rewrite(item) for item in value]
        if isinstance(value, str):
            if value == 'robot':
                return namespace
            if value.startswith('robot/'):
                return namespace + value[len('robot'):]
            if value.startswith('/robot/'):
                return '/' + namespace + value[len('/robot'):]
        return value

    with tempfile.NamedTemporaryFile(
            mode='w', prefix='south_lake_rviz_' + namespace + '_',
            suffix='.rviz', delete=False) as output:
        yaml.safe_dump(rewrite(config), output, sort_keys=False)
        return output.name


def bringup(context):
    namespace = LaunchConfiguration('robot_namespace').perform(context).strip('/')
    source = os.path.join(get_package_share_directory('robot_launch'), 'rviz', 'go2_sim.rviz')
    return [Node(package='robot_launch', executable='visualization_tf', namespace=namespace,
                 parameters=[{'use_sim_time': False}],
                 condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
            Node(package='rviz2', executable='rviz2',
                 arguments=['-d', namespaced_rviz_config(source, namespace)],
                 remappings=[('/tf', '/' + namespace + '/visualization/tf'),
                             ('/tf_static', '/' + namespace + '/visualization/tf_static')],
                 parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
                 condition=IfCondition(LaunchConfiguration('rviz')), output='screen')]
