import os
from launch import LaunchDescription 
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    colav_node = Node(
        package='colav',
        executable='colav_controller.py',
        name='colav',
        parameters=[
                {'guidance_interface/colav_data': 'guidance/collision_avoidance'}
            ],
        output='screen'
    )

    return LaunchDescription([
        colav_node
    ])