import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # LQR_controller launch
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
    
    # Vessel_simulator launch
    vessel_simulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('vessel_simulator_realtime'), 'launch/vessel_simulator_realtime.launch.py')
        )
    )

    # Foxglove bridge launch
    # Download using: $ sudo apt install ros-$ROS_DISTRO-foxglove-bridge
    foxglove_bridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(get_package_share_directory('foxglove_bridge'), 'launch/foxglove_bridge_launch.xml')
        )
    )

    # Return launch description
    return LaunchDescription([
        lqr_controller_launch,
        los_guidance_launch,
        vessel_simulator_launch,
        foxglove_bridge_launch
    ])
