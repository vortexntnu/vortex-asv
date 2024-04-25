import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    map_manager_node = Node(
            package='map_manager',
            executable='map_manager_node',
            name='map_manager_node',
            parameters=[os.path.join(get_package_share_directory('map_manager'),'params','land_manager_params.yaml')],
            output='screen',
        )
    return LaunchDescription([
        map_manager_node
    ])