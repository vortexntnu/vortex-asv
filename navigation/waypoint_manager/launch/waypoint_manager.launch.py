import os
from launch import LaunchDescription 
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    waypoint_manager_node = Node(
        package='waypoint_manager',
        executable='waypoint_manager.py',
        name='waypoint_manager',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('waypoint_manager'), 'config/param_waypoint_manager.yaml')]
    )
 
    return LaunchDescription([
        waypoint_manager_node
    ])
