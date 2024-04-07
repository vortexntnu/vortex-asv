from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os import path

def generate_launch_description():
    # Path to the YAML file
    config = path.join(get_package_share_directory("asv_setup"),'config', 'robots', "freya.yaml")

    thruste_interface_asv_node = Node(
        package='thruster_interface_asv',
        namespace='thruster_interface_asv',
        executable='thruster_interface_asv_node',
        name='thruster_interface_asv_node',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([thruste_interface_asv_node])