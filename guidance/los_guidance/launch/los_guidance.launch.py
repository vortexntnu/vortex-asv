import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    los_guidance_node = Node(
            package='los_guidance',
            executable='los_guidance_node',
            name='los_guidance_node',
            parameters=[],
            output='screen',
        )
    return LaunchDescription([
        los_guidance_node
    ])
