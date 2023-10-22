from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="bms",
            namespace="/internal/status",
            executable="talker",
            name="bms1"
        )
        # Node(
        #     package="bms",
        #     namespace="/internal/status",
        #     executable="freya_bms_node",
        #     name="bms2"
        # ),
    ])