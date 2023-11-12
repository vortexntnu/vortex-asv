import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    thruster_interface_node = Node(
            package='thruster_interface',
            executable='thruster_interface_node',
            name='thruster_interface_node',
            parameters=[],
            output='screen',
        )
    return LaunchDescription([
        thruster_interface_node
    ])