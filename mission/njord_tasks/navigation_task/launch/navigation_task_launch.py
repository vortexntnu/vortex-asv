import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    navigation_task_node = Node(
            package='navigation_task',
            executable='navigation_task_node',
            name='navigation_task_node',
            parameters=[os.path.join(get_package_share_directory('navigation_task'),'params','navigation_task_params.yaml')],
            # arguments=['--ros-args', '--log-level', 'DEBUG'],
            output='screen',
        )
    return LaunchDescription([
        navigation_task_node
    ])