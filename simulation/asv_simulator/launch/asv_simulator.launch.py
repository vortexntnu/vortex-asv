import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    declare_plotting_method_arg = DeclareLaunchArgument(
        'plotting_method',
        default_value='foxglove',
        description='Plotting method to use: matplotlib or foxglove')

    asv_simulator_node = Node(
            package='asv_simulator',
            executable='asv_simulator_node.py',
            name='asv_simulator_node',
            output='screen',
            parameters=[
                os.path.join(get_package_share_directory('asv_setup'), 'config', 'robots', 'freya.yaml'),
                {'plotting_method': LaunchConfiguration('plotting_method')}]
        )

    return LaunchDescription([
        declare_plotting_method_arg,
        asv_simulator_node
    ])
