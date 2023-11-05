from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='thruster_allocator',
            executable='thruster_allocator_node',
            name='thruster_allocator_node'
        )
    ])