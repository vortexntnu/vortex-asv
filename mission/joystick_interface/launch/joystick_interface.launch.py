import os
from launch import LaunchDescription 
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    joystick_interface_node = Node(
        package='joystick_interface',
        executable='joystick_interface.py',
        name='joystick_interface',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('joystick_interface'), 'config/param_joystick_interface.yaml')]
    )
 
    return LaunchDescription([
        joystick_interface_node
    ])

