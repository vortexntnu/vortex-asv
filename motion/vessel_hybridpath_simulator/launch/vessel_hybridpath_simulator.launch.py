import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    vessel_hybridpath_simulator_node = Node(
            package='vessel_hybridpath_simulator',
            executable='vessel_hybridpath_simulator_node',
            name='vessel_hybridpath_simulator_node',
            output='screen',
        )
    return LaunchDescription([
        vessel_hybridpath_simulator_node
    ])
