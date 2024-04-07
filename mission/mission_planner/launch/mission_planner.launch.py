import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    mission_planner_node = Node(
        package='mission_planner',
        executable='mission_planner.py',
        name='mission_planner',
        output='screen'
    )

    return LaunchDescription([
        mission_planner_node
    ])