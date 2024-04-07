from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'plotting_method',
            default_value='matplotlib',
            description='Choose the plotting library: matplotlib or foxglove'
        ),
        Node(
            package='asv_simulator',
            executable='asv_simulator_node.py',
            name='asv_simulator_node',
            output='screen',
            parameters=[{'plotting_method': LaunchConfiguration('plotting_method')}]
        )
    ])
