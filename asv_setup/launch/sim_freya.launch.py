from os import path
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Set environment variable
    set_env_var = SetEnvironmentVariable(
        name='ROSCONSOLE_FORMAT',
        value='[${severity}] [${time}] [${node}]: ${message}'
    )


    # Thruster Allocator launch
    thruster_allocator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(get_package_share_directory('thruster_allocator'), 'launch', 'thruster_allocator.launch.py')
        )
    )

    # Joystick node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joystick_driver',
        output='screen',
        parameters=[
            {'deadzone': 0.15},
            {'autorepeat_rate': 100.0},
        ],
        remappings=[
            ('/joy', '/joystick/joy'),
        ],
    )

    # Joystick interface launch
    joystick_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(get_package_share_directory('joystick_interface'), 'launch', 'joystick_interface.launch.py')
        )
    )

    # Vortex Sim Interface launch
    vortex_sim_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(get_package_share_directory('vortex_sim_interface'), 'launch', 'vortex_sim_interface.launch.py')
        )
    )

    # Controller
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(get_package_share_directory('asv_setup'), 'launch', 'hybridpath.launch.py')
        )
    )

    # Return launch description
    return LaunchDescription([
        set_env_var,
        thruster_allocator_launch,
        joy_node,
        joystick_interface_launch,
        controller_launch,
        vortex_sim_interface_launch, 
    ])