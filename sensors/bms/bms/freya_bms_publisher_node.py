import rclpy
from rclpy.node import Node
import rclpy.logging
from sensor_msgs.msg import BatteryState
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue, DiagnosticArray
from bms.freya_bms import BMS

class FreyaBMSNode(Node):
    """
        Publishes Freya's BMS data to ROS2.

        Methods:
        ---------
        publish_bms_data(self) -> None
            publises BMS data from BMS system to ROS2 node.
        create_key_value_pair(key: str, value) -> KeyValue:
            creates KeyValue object from supplied key and value. The KeyValue object is 
            needed for publishing to /diagnostics topic.
    """
    def __init__(self, usb_ports: list[str]=None) -> None:
        """
            Parameters:
                usb_ports list[str]: The USB ports to look for BMS data from. If no ports are
                passed, the system automatically queries ports and chooses those that respond 
                to the jbdtool command.
            Returns: 
                None
        """
        super().__init__(f'freya_bms')
        rclpy.logging.initialize()

        if usb_ports:
            self._bms_systems = [BMS(usb_port=port) for port in usb_ports]
        else:
            ports = BMS.find_usb_ports(self.get_logger())
            self._bms_systems = [BMS(usb_port=port) for port in ports]
        
        self._batterystate_publishers = [self.create_publisher(BatteryState, f'/internal/status/bms{i}', 10) for i in range(len(self._bms_systems))]
        
        self._diagnostics_publisher = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        self._timer = self.create_timer(2, self.publish_bms_data)

    def publish_bms_data(self) -> None:
        """
            Publishes BMS data to ROS2 topics.

            Parameters: 
                None

            Returns:
                None
        """

        diagnostic_array = DiagnosticArray()

        for i, bms_system in enumerate(self._bms_systems):
            battery_msg = BatteryState()
            diagnostic_status = DiagnosticStatus()

            resp = bms_system.parse_bms_data(
                bms_system.get_bms_data(
                    bms_system.command, 
                    self.get_logger()
                ), 
                self.get_logger()
            )

            if resp:
                battery_msg.voltage = bms_system.voltage
                battery_msg.current = bms_system.current
                battery_msg.cell_temperature = bms_system.temps
                battery_msg.percentage = bms_system.percent_capacity

                self._batterystate_publishers[i].publish(battery_msg)

                diagnostic_status.name = f"Freya battery status {i}"
            
                if bms_system.percent_capacity < 0.01:
                    diagnostic_status.level = DiagnosticStatus.ERROR
                elif bms_system.percent_capacity < 0.2:
                    diagnostic_status.level = DiagnosticStatus.WARN
                else:
                    diagnostic_status.level = DiagnosticStatus.OK

                diagnostic_status.values.extend([
                    FreyaBMSNode.create_key_value_pair("voltage", bms_system.voltage), 
                    FreyaBMSNode.create_key_value_pair("current", bms_system.current), 
                    FreyaBMSNode.create_key_value_pair("temps", bms_system.temps), 
                    FreyaBMSNode.create_key_value_pair("percentage", bms_system.percent_capacity)])

                diagnostic_status.message = "level indicates whether the battery level is above 20 (OK), below 20 (WARN), or below 1 (ERROR)"

                diagnostic_array.status.append(diagnostic_status)
            else:
                self.get_logger().warn(f"No data to be published. Battery {i} may not be connected")

        self._diagnostics_publisher.publish(diagnostic_array)

    @staticmethod
    def create_key_value_pair(key: str, value) -> KeyValue:
        """
            Creates a KeyValue() for ROS2 diagnostics

            Parameters:
                key (str): Key string
                value: value

            Returns:
                A KeyValue object created from the 'key' and 'value' parameters
        """
        kv = KeyValue()
        kv.key = str(key)
        kv.value = str(value)

        return kv

def main(args=None):
    rclpy.init(args=args)

    node = FreyaBMSNode()

    rclpy.spin(node)

    node.destroy_node()    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    
