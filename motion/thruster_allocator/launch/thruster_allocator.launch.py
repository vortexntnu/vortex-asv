import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    trushter_allocator_node = Node(
            package='thruster_allocator',
            executable='thruster_allocator_node',
            name='thruster_allocator_node',
            parameters=[os.path.join(get_package_share_directory('asv_setup'),'config','robots','freya.yaml')],
            output='screen',
        )
    return LaunchDescription([
        trushter_allocator_node
    ])