from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='blackbox',
            namespace='blackbox',
            executable='blackbox_node',
            name='blackbox_node'
        ),
    ])