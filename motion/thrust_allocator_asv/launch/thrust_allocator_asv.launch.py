from os import path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = path.join(
        get_package_share_directory('asv_setup'), 'config', 'robots', 'freya.yaml'
    )

    thrust_allocator_asv_node = Node(
        package='thrust_allocator_asv',
        executable='thrust_allocator_asv_node',
        name='thrust_allocator_asv_node',
        namespace='freya',
        parameters=[config],
        output='screen',
    )

    return LaunchDescription([thrust_allocator_asv_node])
