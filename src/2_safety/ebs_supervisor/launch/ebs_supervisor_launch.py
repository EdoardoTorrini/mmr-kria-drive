from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():

    config_node = os.path.join(
        get_package_share_directory('ebs_supervisor'),
        'config',
        'ebs_supervisor_conf.yaml'
    )

    node=Node(
        package='ebs_supervisor',
        name='ebs_supervisor_node',
        executable='ebs_supervisor_node',
        parameters=[
            config_node,
        ]
    )

    return LaunchDescription([
        node
    ])