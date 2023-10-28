import os
from launch import LaunchDescription 
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    parameter_manager_node = Node(
        package='asv_setup',
        executable='parameter_manager_node.py',
        name='parameter_manager_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('asv_setup'), 'config','freya.yaml')]
    )
 
    return LaunchDescription([
        parameter_manager_node
    ])
