from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory
from os import path

def generate_launch_description():
    config = path.join(get_package_share_directory("asv_setup"), 'config', 'robots', "freya.yaml")
    
    thruster_interface_asv_node = LifecycleNode(
        package='thruster_interface_asv',
        executable='thruster_interface_asv_node',
        name='thruster_interface_asv_node',
        namespace='motion',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([thruster_interface_asv_node])