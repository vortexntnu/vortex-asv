"""
TODO
From Martynas:
To Rose:
Add data recording for these ROS2 topics:

MOST IMPORTANT:
'/internal/status/bms0',
'/internal/status/bms1',
'/asv/temperature/ESC1',
'/asv/temperature/ESC2',
'/asv/temperature/ESC3',
'/asv/temperature/ESC4',
'/asv/temperature/ambient1',
'/asv/temperature/ambient2',
'/thrust/thruster_forces',
'/pwm',

Nice to have:
'/joy',                               
'/joystick/joy',
'/thrust/wrench_input',
'/controller/lqr/enable',
'/tf',
'/tf_static'
"""


# ROS2 Libraries
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

# ROS2 Topic Libraries
from std_msgs.msg import Float32

# Custom Libraries
from blackbox.blackbox_log_data import BlackBoxLogData



class BlackBoxNode(Node):
    def __init__(self):
        # Start the ROS2 Node ----------
        super().__init__("blackbox_node")

        # Initialize sunscribers ----------
        self.psm_current_subscriber = self.create_subscription(
            Float32,
            "/asv/power_sense_module/current",
            self.psm_current_callback,
            1)
        self.psm_current_data = 0.0

        self.psm_voltage_subscriber = self.create_subscription(
            Float32,
            "/asv/power_sense_module/voltage",
            self.psm_voltage_callback,
            1)
        self.psm_voltage_data = 0.0

        # Initialize logger ----------
        # Get package directory location
        ros2_package_directory_location = get_package_share_directory("blackbox")
        ros2_package_directory_location = ros2_package_directory_location + "/../../../../" # go back to workspace
        ros2_package_directory_location = ros2_package_directory_location + "src/vortex-asv/mission/blackbox/" # Navigate to this package
        
        # Make blackbox loging file
        self.blackbox_log_data = BlackBoxLogData(
            ROS2_PACKAGE_DIRECTORY = ros2_package_directory_location
        )

        # Logs all the newest data 10 times per second
        self.logger_timer = self.create_timer(0.1, self.logger)

        # Debuging ----------
        self.get_logger().info(
            "Startied logging data for topics: \n"
            "/asv/power_sense_module/current [Float32] \n"
            "/asv/power_sense_module/voltage [Float32] \n"
        )

    # Callback Methods ----------
    def psm_current_callback(self, msg):
        self.psm_current_data = msg.data

    def psm_voltage_callback(self, msg):
        self.psm_voltage_data = msg.data

    def logger(self):
        self.blackbox_log_data.log_data_to_csv_file(
            psm_current=self.psm_current_data,
            psm_voltage=self.psm_voltage_data,
        )



def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)

    # Start ROS2 node
    blackbox_node = BlackBoxNode()
    rclpy.spin(blackbox_node)

    # Destroy the node once ROS2 ends
    blackbox_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()