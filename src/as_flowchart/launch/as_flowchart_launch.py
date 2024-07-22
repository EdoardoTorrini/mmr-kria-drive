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
        'as_flowchart_conf.yaml'
    )

    node=Node(
        package='as_flowchart',
        name='as_flowchart_node',
        executable='as_flowchart_node',
        parameters=[
            config_node,
        ]
    )

    return LaunchDescription([
        node
    ])