from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('system_monitor'),
        'config',
        'system_monitor_config.yaml'
    )

    system_monitor_node = Node(
        package='system_monitor',
        executable='system_monitor_node.py',
        name='system_monitor',
        output='screen',
        emulate_tty=True,
        parameters=[config]
    )

    return LaunchDescription([
        system_monitor_node
    ])