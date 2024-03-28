import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # LQR launch
    lqr_controller_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('lqr_controller'), 'launch/lqr_controller.launch.py')
        )
    )

    # LOS_guidance launch
    los_guidance_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('los_guidance'), 'launch/los_guidance.launch.py')
        )
    )

    vessel_simulator_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('vessel_simulator'), 'launch/vessel_simulator.launch.py')
        )
    )


    # Return launch description
    return LaunchDescription([
        lqr_controller_launch,
        los_guidance_launch,
        vessel_simulator_launch
    ])