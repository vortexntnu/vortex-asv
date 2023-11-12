from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bms',
            executable='freya_bms_publisher_node'
        )
    ])