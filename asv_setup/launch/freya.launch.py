from os import path
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

    system_monitor_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
            path.join(get_package_share_directory('system_monitor'),'launch','system_monitor.launch.py')
        )
    )

    thrust_allocator_asv_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
            path.join(get_package_share_directory('thrust_allocator_asv'),'launch','thrust_allocator_asv.launch.py')
        )
    )

    thruster_interface_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
            path.join(get_package_share_directory('thruster_interface_asv'),'launch','thruster_interface_asv.launch.py')
        )
    )

    return LaunchDescription([
        set_env_var,
        system_monitor_launch,
        thrust_allocator_asv_launch,
        thruster_interface_launch,
    ])
