import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    hybridpath_controller_node = Node(
            package='hybridpath_controller',
            executable='hybridpath_controller_node',
            name='hybridpath_controller_node',
            output='screen',
        )
    return LaunchDescription([
        hybridpath_controller_node
    ])
