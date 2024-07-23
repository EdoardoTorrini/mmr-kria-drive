from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():

    config_node = os.path.join(
        get_package_share_directory('apps_actuator'),
        'config',
        'apps_actuator_conf.yaml'
    )

    node=Node(
        package='apps_actuator',
        name='apps_actuator_node',
        executable='apps_actuator_node',
        parameters=[
            config_node,
        ]
    )

    return LaunchDescription([
        node
    ])