from os import path
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    set_env_var = SetEnvironmentVariable(
        name='ROSCONSOLE_FORMAT',
        value='[${severity}] [${time}] [${node}]: ${message}'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'deadzone': 0.15,
            'autorepeat_rate': 100.0,
            'device_name': 'Wireless Controller',
        }],
        remappings=[
            ('/joy', '/freya/joy'),
        ]
    )

    joystick_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(get_package_share_directory('joystick_interface'),
                      'launch','joystick_interface.launch.py')
        )
    )

    thruster_allocator_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
            path.join(get_package_share_directory('thruster_allocator'),
                      'launch','thruster_allocator.launch.py')
        )
    )

    return LaunchDescription([
        set_env_var,
        joy_node,
        joystick_interface_launch,
        thruster_allocator_launch
    ])
