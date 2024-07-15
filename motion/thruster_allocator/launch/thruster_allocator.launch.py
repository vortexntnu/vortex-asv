from os import path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    config = path.join(get_package_share_directory('asv_setup'), 'config', 'robots', 'freya.yaml')
    
    thruster_allocator_node = LifecycleNode(
        package='thruster_allocator',
        executable='thruster_allocator_node',
        name='thruster_allocator_node',
        namespace='motion',
        parameters=[config],
        output='screen',
    )

    return LaunchDescription([
        thruster_allocator_node
    ])