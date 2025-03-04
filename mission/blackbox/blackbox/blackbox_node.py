# ROS2 Libraries
import array

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

# ROS2 Topic Libraries
from std_msgs.msg import Float32, Float32MultiArray, Int16MultiArray

# Custom Libraries
from blackbox.blackbox_log_data import BlackBoxLogData


class BlackBoxNode(Node):
    def __init__(self):
        # Start the ROS2 Node ----------
        super().__init__("blackbox_node")

        # Initialize sunscribers ----------
        self.psm_current_subscriber = self.create_subscription(
            Float32, "/asv/power_sense_module/current", self.psm_current_callback, 1
        )
        self.psm_current_data = 0.0

        self.psm_voltage_subscriber = self.create_subscription(
            Float32, "/asv/power_sense_module/voltage", self.psm_voltage_callback, 1
        )
        self.psm_voltage_data = 0.0

        self.temperature_esc1_subscriber = self.create_subscription(
            Float32, "/asv/temperature/esc1", self.temperature_esc1_callback, 1
        )
        self.temperature_esc1_data = 0.0

        self.temperature_esc2_subscriber = self.create_subscription(
            Float32, "/asv/temperature/esc2", self.temperature_esc2_callback, 1
        )
        self.temperature_esc2_data = 0.0

        self.temperature_esc3_subscriber = self.create_subscription(
            Float32, "/asv/temperature/esc3", self.temperature_esc3_callback, 1
        )
        self.temperature_esc3_data = 0.0

        self.temperature_esc4_subscriber = self.create_subscription(
            Float32, "/asv/temperature/esc4", self.temperature_esc4_callback, 1
        )
        self.temperature_esc4_data = 0.0

        self.temperature_ambient1_subscriber = self.create_subscription(
            Float32, "/asv/temperature/ambient1", self.temperature_ambient1_callback, 1
        )
        self.temperature_ambient1_data = 0.0

        self.temperature_ambient2_subscriber = self.create_subscription(
            Float32, "/asv/temperature/ambient2", self.temperature_ambient2_callback, 1
        )
        self.temperature_ambient2_data = 0.0

        self.bms0_subscriber = self.create_subscription(
            BatteryState, "/internal/status/bms0", self.bms0_callback, 1
        )
        self.bms0_voltage_data = 0.0
        self.bms0_current_data = 0.0
        self.bms0_percentage_data = 0.0
        self.bms0_cell_temperature_data = array.array('f', [0.0, 0.0, 0.0])

        self.bms1_subscriber = self.create_subscription(
            BatteryState, "/internal/status/bms1", self.bms1_callback, 1
        )
        self.bms1_voltage_data = 0.0
        self.bms1_current_data = 0.0
        self.bms1_percentage_data = 0.0
        self.bms1_cell_temperature_data = array.array('f', [0.0, 0.0, 0.0])

        self.thruster_forces = self.create_subscription(
            Float32MultiArray,
            "/thrust/thruster_forces",
            self.thruster_forces_callback,
            1,
        )
        self.thruster_forces_data = array.array('f', [0.0, 0.0, 0.0, 0.0])

        self.pwm = self.create_subscription(
            Int16MultiArray, "/pwm", self.pwm_callback, 1
        )
        self.pwm_data = array.array('i', [0, 0, 0, 0])

        # Initialize logger ----------
        # Get package directory location
        ros2_package_directory_location = get_package_share_directory("blackbox")
        ros2_package_directory_location = (
            ros2_package_directory_location + "/../../../../"
        )  # go back to workspace
        ros2_package_directory_location = (
            ros2_package_directory_location + "src/vortex-asv/mission/blackbox/"
        )  # Navigate to this package

        # Make blackbox logging file
        self.blackbox_log_data = BlackBoxLogData(
            ros2_package_directory=ros2_package_directory_location
        )

        # Logs all the newest data 10 times per second
        self.declare_parameter(
            "blackbox.data_logging_rate", 1.0
        )  # Providing a default value 1.0 => 1 samplings per second, very slow
        data_logging_rate = (
            self.get_parameter("blackbox.data_logging_rate")
            .get_parameter_value()
            .double_value
        )
        timer_period = 1.0 / data_logging_rate
        self.logger_timer = self.create_timer(timer_period, self.logger)

        # Debugging ----------
        self.get_logger().info(
            "Started logging data for topics: \n"
            "/asv/power_sense_module/current [Float32] \n"
            "/asv/power_sense_module/voltage [Float32] \n"
            "/asv/temperature/ESC1 [Float32] \n"
            "/asv/temperature/ESC2 [Float32] \n"
            "/asv/temperature/ESC3 [Float32] \n"
            "/asv/temperature/ESC4 [Float32] \n"
            "/asv/temperature/ambient1 [Float32] \n"
            "/asv/temperature/ambient2 [Float32] \n"
            "/internal/status/bms0 [Float32] \n"
            "/internal/status/bms1 [Float32] \n"
            "/thrust/thruster_forces [Float32] \n"
            "/pwm [Float32] \n"
        )

    # Callback Methods ----------
    def psm_current_callback(self, msg):
        self.psm_current_data = msg.data

    def psm_voltage_callback(self, msg):
        self.psm_voltage_data = msg.data

    def temperature_esc1_callback(self, msg):
        self.temperature_esc1_data = msg.data

    def temperature_esc2_callback(self, msg):
        self.temperature_esc2_data = msg.data

    def temperature_esc3_callback(self, msg):
        self.temperature_esc3_data = msg.data

    def temperature_esc4_callback(self, msg):
        self.temperature_esc4_data = msg.data

    def temperature_ambient1_callback(self, msg):
        self.temperature_ambient1_data = msg.data

    def temperature_ambient2_callback(self, msg):
        self.temperature_ambient2_data = msg.data

    def bms0_callback(self, msg):
        self.bms0_voltage_data = msg.voltage
        self.bms0_current_data = msg.current
        self.bms0_percentage_data = msg.percentage
        self.bms0_cell_temperature_data = msg.cell_temperature

    def bms1_callback(self, msg):
        self.bms1_voltage_data = msg.voltage
        self.bms1_current_data = msg.current
        self.bms1_percentage_data = msg.percentage
        self.bms1_cell_temperature_data = msg.cell_temperature

    def thruster_forces_callback(self, msg):
        self.thruster_forces_data = msg.data

    def pwm_callback(self, msg):
        self.pwm_data = msg.data

    def logger(self):
        self.blackbox_log_data.log_data_to_csv_file(
            psm_current=self.psm_current_data,
            psm_voltage=self.psm_voltage_data,
            temperature_esc1=self.temperature_esc1_data,
            temperature_esc2=self.temperature_esc2_data,
            temperature_esc3=self.temperature_esc3_data,
            temperature_esc4=self.temperature_esc4_data,
            temperature_ambient1=self.temperature_ambient1_data,
            temperature_ambient2=self.temperature_ambient2_data,
            bms0_voltage=self.bms0_voltage_data,
            bms0_current=self.bms0_current_data,
            bms0_percentage=self.bms0_percentage_data,
            bms0_cell_temperature_1=self.bms0_cell_temperature_data[0],
            bms0_cell_temperature_2=self.bms0_cell_temperature_data[1],
            bms0_cell_temperature_3=self.bms0_cell_temperature_data[2],
            bms1_voltage=self.bms1_voltage_data,
            bms1_current=self.bms1_current_data,
            bms1_percentage=self.bms1_percentage_data,
            bms1_cell_temperature_1=self.bms1_cell_temperature_data[0],
            bms1_cell_temperature_2=self.bms1_cell_temperature_data[1],
            bms1_cell_temperature_3=self.bms1_cell_temperature_data[2],
            thruster_forces_1=self.thruster_forces_data[0],
            thruster_forces_2=self.thruster_forces_data[1],
            thruster_forces_3=self.thruster_forces_data[2],
            thruster_forces_4=self.thruster_forces_data[3],
            pwm_1=self.pwm_data[0],
            pwm_2=self.pwm_data[1],
            pwm_3=self.pwm_data[2],
            pwm_4=self.pwm_data[3],
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
