import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    enable_tf = LaunchConfiguration('enable_tf')
    enable_tf_arg = DeclareLaunchArgument(
        'enable_tf',
        default_value='True',
        description='enable tf launch file',
    )
    enable_seapath = LaunchConfiguration('enable_seapath')
    enable_seapath_arg = DeclareLaunchArgument(
        'enable_seapath',
        default_value='True',
        description='enable seapath launch file',
    )
  
    seapath_ros_driver_node = Node(
            package='seapath_ros_driver',
            executable='seapath_ros_driver_node',
            name='seapath_ros_driver_node',
            parameters=[os.path.join(get_package_share_directory('asv_setup'),'config','sitaw','seapath_params.yaml')],
            output='screen',
            condition=IfCondition(enable_seapath),
            
        )
    map_manager_node = Node(
            package='map_manager',
            executable='map_manager_node',
            name='map_manager_node',
            parameters=[os.path.join(get_package_share_directory('asv_setup'),'config','sitaw','map_manager_params.yaml')],
            output='screen',
        )
    landmark_server_node = Node(
            package='landmark_server',
            executable='landmark_server_node',
            name='landmark_server_node',
            parameters=[os.path.join(get_package_share_directory('asv_setup'),'config','sitaw','landmark_server_params.yaml')],
            # arguments=['--ros-args', '--log-level', 'DEBUG'],
            output='screen',
        )
    tf_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('asv_setup'), 'launch', 'tf.launch.py')),
        condition=IfCondition(enable_tf),
    )
    return LaunchDescription([
        enable_tf_arg,
        tf_launch,
        enable_seapath_arg,
        seapath_ros_driver_node,
        map_manager_node,
        landmark_server_node
    ])