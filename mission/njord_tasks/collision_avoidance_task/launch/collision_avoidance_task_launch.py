import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    collision_avoidance_task_node = Node(
            package='collision_avoidance_task',
            executable='collision_avoidance_task_node',
            name='collision_avoidance_task_node',
            parameters=[os.path.join(get_package_share_directory('collision_avoidance_task'),'params','collision_avoidance_task_params.yaml')],
            # arguments=['--ros-args', '--log-level', 'DEBUG'],
            output='screen',
        )
    return LaunchDescription([
        collision_avoidance_task_node
    ])