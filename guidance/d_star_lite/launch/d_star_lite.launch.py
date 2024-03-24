import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    d_star_lite_node = Node(
        package='d_star_lite',
        executable='d_star_lite_node.py',
        name='d_star_lite_node',
        output='screen'
    )

    return LaunchDescription([
        d_star_lite_node
    ])