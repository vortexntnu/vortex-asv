import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    vessel_simulator_node = Node(
            package='vessel_simulator',
            executable='vessel_simulator_node',
            name='vessel_simulator_node',
            # parameters=[os.path.join(get_package_share_directory('vessel_simulator'),'config','lqr_config.yaml')],
            output='screen',
        )
    return LaunchDescription([
        vessel_simulator_node
    ])

