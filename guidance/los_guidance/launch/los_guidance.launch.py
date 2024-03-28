import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    los_guidance_node = Node(
            package='los_guidance',
            executable='los_guidance_node',
            name='los_guidance_node',
            parameters=[os.path.join(get_package_share_directory('los_guidance'),'config','los_guidance_config.yaml')],
            output='screen',
        )
    return LaunchDescription([
        los_guidance_node
    ])
