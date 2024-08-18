import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument)
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    default_params_file = os.path.join(get_package_share_directory('map_manager'),'params','map_manager_params.yaml')
    params_file = LaunchConfiguration('params_file')
    params_file_arg = DeclareLaunchArgument('params_file',
                                            default_value=str(
                                                default_params_file),
                                            description='name or path to the parameters file to use.')
    map_manager_node = Node(
            package='map_manager',
            executable='map_manager_node',
            name='map_manager_node',
            parameters=[params_file],
            # arguments=['--ros-args', '--log-level', 'DEBUG'],
            output='screen',
        )
    return LaunchDescription([
        params_file_arg,
        map_manager_node
    ])