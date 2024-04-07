import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    #Path to the YAML file
    yaml_file_path = os.path.join(
        get_package_share_directory("thruster_interface"),
        "../../../../",                             # Go to the workspace
        "src/vortex-asv/asv_setup/config/robots/",  # Go where the yaml file is located at
        'freya.yaml'
    )
    thruster_interface_node = Node(
            package='thruster_interface',
            executable='thruster_interface_node',
            name='thruster_interface_node',
            output='screen',
            parameters=[yaml_file_path],
        )
    return LaunchDescription([thruster_interface_node])