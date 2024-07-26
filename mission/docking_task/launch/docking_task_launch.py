import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    docking_task_node = Node(
            package='docking_task',
            executable='docking_task_node',
            name='docking_task_node',
            parameters=[os.path.join(get_package_share_directory('docking_task'),'params','docking_task_params.yaml')],
            # arguments=['--ros-args', '--log-level', 'DEBUG'],
            output='screen',
        )
    return LaunchDescription([
        docking_task_node
    ])