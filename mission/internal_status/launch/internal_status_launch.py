from launch import LaunchDescription
from launch_ros.actions import Node 

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'internal_status',
            executable = 'power_sense_module_publisher'
        )
    ])