from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
# from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    # usb_port_1 = LaunchConfiguration("usb_port_1")
    # usb_port_2 = LaunchConfiguration("usb_port_2")


    return LaunchDescription([
        DeclareLaunchArgument(
            "usb_port_1",
            default_value="",
            description="USB port of first battery package"
        ),
        DeclareLaunchArgument(
            "usb_port_2",
            default_value="",
            description="USB port of second battery package"
        ),
        Node(
            package="bms",
            executable="bms_publisher",
            name="freya_bms1",
            parameters=[
                {"usb_port": LaunchConfiguration("usb_port_1")}
            ]
        ),
        Node(
            package="bms",
            executable="bms_publisher",
            name="freya_bms",
            parameters=[
                {"usb_port": LaunchConfiguration("usb_port_2")}
            ]
        )
        # Node(
        #     package="bms",
        #     namespace="/internal/status",
        #     executable="freya_bms_node",
        #     name="bms2"
        # ),
    ])