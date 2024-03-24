import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    hybridpath_guidance_node = Node(
            package='hybridpath_guidance',
            executable='hybridpath_guidance_node',
            name='hybridpath_guidance_node',
            parameters=[os.path.join(get_package_share_directory('hybridpath_guidance'),'config','hybridpath_guidance_config.yaml')],
            output='screen',
        )
    return LaunchDescription([
        hybridpath_guidance_node
    ])
