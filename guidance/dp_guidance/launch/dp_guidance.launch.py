import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    dp_guidance_node = Node(
            package='dp_guidance',
            executable='dp_guidance_node.py',
            name='dp_guidance_node',
            parameters=[os.path.join(get_package_share_directory('dp_guidance'),'config','dp_guidance_config.yaml')],
            output='screen',
        )
    return LaunchDescription([
        dp_guidance_node
    ])

