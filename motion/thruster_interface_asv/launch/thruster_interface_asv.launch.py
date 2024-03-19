from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the YAML file
    yaml_file_path = os.path.join(
        get_package_share_directory("thruster_interface_asv"),
        "../../../../", # Go back to the workspace
        "src/vortex-asv/asv_setup/config/robots/",
        "freya.yaml"
    )

    return LaunchDescription([
        Node(
            package='thruster_interface_asv',
            namespace='thruster_interface_asv',
            executable='thruster_interface_asv_node',
            name='thruster_interface_asv_node',
            output='screen',
            parameters=[yaml_file_path],
        ),
    ])