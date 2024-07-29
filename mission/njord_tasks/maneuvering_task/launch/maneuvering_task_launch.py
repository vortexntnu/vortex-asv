import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    maneuvering_task_node = Node(
            package='maneuvering_task',
            executable='maneuvering_task_node',
            name='maneuvering_task_node',
            parameters=[os.path.join(get_package_share_directory('maneuvering_task'),'params','maneuvering_task_params.yaml')],
            # arguments=['--ros-args', '--log-level', 'DEBUG'],
            output='screen',
        )
    return LaunchDescription([
        maneuvering_task_node
    ])