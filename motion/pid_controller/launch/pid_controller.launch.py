import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pid_controller_node = Node(
            package='pid_controller',
            executable='pid_controller_node.py',
            name='pid_controller_node',
            parameters=[
                os.path.join(get_package_share_directory('pid_controller'),'config','pid_config.yaml'),
                os.path.join(get_package_share_directory('asv_setup'), 'config', 'robots', 'freya.yaml')
            ],
            output='screen',
        )
    return LaunchDescription([
        pid_controller_node
    ])
