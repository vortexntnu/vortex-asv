from os import path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = [
        path.join(
            get_package_share_directory(package_name="asv_setup"),
            "config",
            "robots",
            "freya.yaml",
        ),
        path.join(
            get_package_share_directory(package_name="thruster_interface_asv"),
            "config",
            "thruster_interface_asv_config.yaml",
        ),
    ]

    thruster_interface_asv_node = Node(
        package="thruster_interface_asv",
        executable="thruster_interface_asv_node",
        name="thruster_interface_asv_node",
        namespace="freya",
        output="screen",
        parameters=config,
    )

    return LaunchDescription([thruster_interface_asv_node])
