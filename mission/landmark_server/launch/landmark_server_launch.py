import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument)
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    default_params_file = os.path.join(get_package_share_directory('landmark_server'),'params','landmark_server_params.yaml')
    params_file = LaunchConfiguration('params_file')
    params_file_arg = DeclareLaunchArgument('params_file',
                                            default_value=str(
                                                default_params_file),
                                            description='name or path to the parameters file to use.')
    landmark_server_node = Node(
            package='landmark_server',
            executable='landmark_server_node',
            name='landmark_server_node',
            parameters=[params_file],
            # arguments=['--ros-args', '--log-level', 'DEBUG'],
            output='screen',
        )
    return LaunchDescription([
        params_file_arg,
        landmark_server_node
    ])