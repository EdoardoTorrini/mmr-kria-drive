from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
def generate_launch_description():


    config = os.path.join(get_package_share_directory('pure_pursuit'),'config','track_pure_pursuit.yaml')


    pure_pursuit_node = Node(
        #namespace="car",
        name="pure_pursuit_node",
        package="pure_pursuit",
        executable="pure_pursuit_node",
        parameters=[config]
    )

    return LaunchDescription([
        pure_pursuit_node,

    ])