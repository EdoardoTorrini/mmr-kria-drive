from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():

    config_node = os.path.join(
        get_package_share_directory('canbus_bridge'),
        'config',
        'canbus_bridge_conf.yaml'
    )

    node=Node(
        package='canbus_bridge',
        name='canbus_bridge_node',
        executable='canbus_bridge_node',
        parameters=[
            config_node,
        ]
    )

    return LaunchDescription([
        node
    ])