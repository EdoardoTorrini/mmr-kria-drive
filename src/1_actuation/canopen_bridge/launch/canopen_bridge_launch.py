from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():

    config_node = os.path.join(
        get_package_share_directory('canopen_bridge'),
        'config',
        'canopen_bridge_conf.yaml'
    )

    node=Node(
        package='canopen_bridge',
        name='canopen_bridge_node',
        executable='canopen_bridge_node',
        parameters=[
            config_node,
        ]
    )

    return LaunchDescription([
        node
    ])