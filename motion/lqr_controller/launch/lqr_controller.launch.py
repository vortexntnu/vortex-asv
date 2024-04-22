import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    lqr_controller_node = Node(
            package='lqr_controller',
            executable='lqr_controller_node.py',
            name='lqr_controller_node',
            parameters=[
                os.path.join(get_package_share_directory('lqr_controller'),'config','lqr_config.yaml'),
                os.path.join(get_package_share_directory('asv_setup'), 'config', 'robots', 'freya.yaml')
            ],
            output='screen',
        )
    return LaunchDescription([
        lqr_controller_node
    ])
