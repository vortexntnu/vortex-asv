import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    asv_simulator_node = Node(
            package='asv_simulator',
            executable='asv_simulator_node.py',
            name='asv_simulator_node',
            output='screen',
            parameters=[
                os.path.join(get_package_share_directory('asv_setup'), 'config', 'robots', 'freya.yaml'),
                os.path.join(get_package_share_directory('asv_simulator'), 'config', 'asv_sim_config.yaml')]
        )

    return LaunchDescription([
        asv_simulator_node
    ])
