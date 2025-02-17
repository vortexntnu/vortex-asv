from os import path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Path to the YAML file
    config = path.join(
        get_package_share_directory("asv_setup"), 'config', 'robots', "freya.yaml"
    )

    thruster_interface_asv_node = Node(
        package='thruster_interface_asv',
        executable='thruster_interface_asv_node',
        name='thruster_interface_asv_node',
        namespace='freya',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([thruster_interface_asv_node])
