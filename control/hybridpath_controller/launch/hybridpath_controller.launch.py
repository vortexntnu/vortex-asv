import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    hybridpath_controller_node = Node(
        package='hybridpath_controller',
        executable='hybridpath_controller_node.py',
        name='hybridpath_controller_node',
        parameters=[
            os.path.join(
                get_package_share_directory('hybridpath_controller'),
                'config',
                'hybridpath_controller_config.yaml',
            ),
            os.path.join(
                get_package_share_directory('asv_setup'),
                'config',
                'robots',
                'freya.yaml',
            ),
        ],
        output='screen',
    )
    return LaunchDescription([hybridpath_controller_node])
