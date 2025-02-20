import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

hp_config = os.path.join(
    get_package_share_directory('hybridpath_guidance'),
    'config',
    'hybridpath_guidance_config.yaml',
)

freya_config = os.path.join(
    get_package_share_directory('asv_setup'),
    'config',
    'robots',
    'freya.yaml',
)


def generate_launch_description():
    hybridpath_guidance_node = Node(
        package='hybridpath_guidance',
        executable='hybridpath_guidance_node.py',
        name='hybridpath_guidance_node',
        namespace='freya',
        parameters=[
            hp_config,
            freya_config,
        ],
        output='screen',
    )
    return LaunchDescription([hybridpath_guidance_node])
