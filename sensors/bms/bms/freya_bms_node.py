import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from bms.freya_bms import BMS


#TODO: run the node on startup


class FreyaBMSNode(Node):
    """
        Publishes Freya's BMS data to ROS.

        Methods:
        ---------
        publish_bms_data() -> None
            publises BMS data from BMS system to ros2 node.
    """
    def __init__(self, usb_ports: list[str]=None) -> None:
        """
            Parameters:
                usb_port(str): The USB port to look for BMS data from. If no port is
                passed, the system automatically chooses the first port that gives
                an output
            Returns: 
                None
        """
        super().__init__(f'freya_bms')
        
        if usb_ports:
            self._bms_systems = [BMS(usb_port=port) for port in usb_ports]
        else:
            ports = BMS.find_usb_ports()
            self._bms_systems = [BMS(usb_port=port) for port in ports]
        
        self._batterystate_publishers = [self.create_publisher(BatteryState, f'/internal/status/bms{i}', 10) for i in range(len(self._bms_systems))]
        
        self._diagnostics_publishers = [self.create_publisher(DiagnosticStatus, f'/internal/status/bms_diagnostic{i}', 10) for i in range(len(self._bms_systems))]

        self._timer = self.create_timer(2, self.publish_bms_data)
        

    def publish_bms_data(self) -> None:
        """
            Publishes BMS data to ros2 topics.
        """

        for i, bms_system in enumerate(self._bms_systems):
            battery_msg = BatteryState()
            diagnostics_msg = DiagnosticStatus()

            bms_system.parse_bms_data(bms_system.get_bms_data(bms_system.command))

            battery_msg.voltage = bms_system.voltage
            battery_msg.current = bms_system.current
            battery_msg.cell_temperature = bms_system.temps
            battery_msg.percentage = bms_system.percent_capacity

            self._batterystate_publishers[i].publish(battery_msg)

            diagnostics_msg.name = f"Freya battery status {i}"
        
            if bms_system.percent_capacity < 1:
                diagnostics_msg.level = DiagnosticStatus.ERROR
            elif bms_system.percent_capacity < 20:
                diagnostics_msg.level = DiagnosticStatus.WARN
            else:
                diagnostics_msg.level = DiagnosticStatus.OK

            diagnostics_msg.values.extend([
                create_key_value_pair("voltage", bms_system.voltage), 
                create_key_value_pair("current", bms_system.current), 
                create_key_value_pair("temps", bms_system.temps), 
                create_key_value_pair("percentage", bms_system.percent_capacity)])

            diagnostics_msg.message = "level indicates whether the battery level is \
            below 0 (ERROR), below 20 (WARN) or above 20 (OK)"

            self._diagnostics_publishers[i].publish(diagnostics_msg)

def create_key_value_pair(key: str, value) -> KeyValue:
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
    
