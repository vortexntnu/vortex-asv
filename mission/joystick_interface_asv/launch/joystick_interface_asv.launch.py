import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

joystick_config = os.path.join(
    get_package_share_directory('joystick_interface_asv'),
    'config',
    'param_joystick_interface.yaml',
)

freya_config = os.path.join(
    get_package_share_directory('asv_setup'),
    'config',
    'robots',
    'freya.yaml',
)

def generate_launch_description():
    joystick_interface_node = Node(
        package='joystick_interface_asv',
        executable='joystick_interface_asv_node.py',
        name='joystick_interface_asv',
        namespace='freya',
        output='screen',
        parameters=[
            joystick_config,
            freya_config,
        ],
    )

    return LaunchDescription([joystick_interface_node])
