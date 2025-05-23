import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Path to the YAML file
    yaml_file_path = os.path.join(
        get_package_share_directory("blackbox"),
        "../../../../",  # Go to the workspace
        "src/vortex-asv/asv_setup/config/robots/",  # Go inside where yamal files are located at
        'freya.yaml',
    )

    return LaunchDescription(
        [
            Node(
                package='blackbox',
                namespace='blackbox',
                executable='blackbox_node',
                name='blackbox_node',
                output='screen',
                parameters=[yaml_file_path],
            ),
        ]
    )
