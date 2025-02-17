from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    d_star_lite_node = Node(
        package='d_star_lite',
        executable='d_star_lite_node.py',
        name='d_star_lite_node',
        output='screen',
    )

    return LaunchDescription([d_star_lite_node])
