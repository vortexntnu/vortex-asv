import os
from launch import LaunchDescription 
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    colav_sim_node = Node(
        package='colav',
        executable='colav_sim.py',
        name='colav_sim',
        output='screen'
    )

    return LaunchDescription([
        colav_sim_node
    ])