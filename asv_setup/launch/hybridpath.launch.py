import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    hybridpath_controller_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('hybridpath_controller'), 'launch', 'hybridpath_controller.launch.py')
        )
    )

    hybridpath_guidance_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('hybridpath_guidance'), 'launch', 'hybridpath_guidance.launch.py')
        )
    )

    # Return launch description
    return LaunchDescription([
        hybridpath_controller_launch,
        hybridpath_guidance_launch
    ])