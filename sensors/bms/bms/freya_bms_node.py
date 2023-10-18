import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from bms.freya_bms import BMS

class FreyaBMSNode(Node):
    def __init__(self, usb_port=None) -> None:
        super().__init__('freya_bms')
        self._publisher = self.create_publisher(BatteryState, '/internal/status/bms', 10)
        self._timer = self.create_timer(2, self.publish_bms_data)

        if usb_port:
            self._bms_system = BMS(usb_port=usb_port)
        else:
            self._bms_system = BMS()

    def publish_bms_data(self):
        # print("mjau")
        battery_msg = BatteryState()
        
        self._bms_system.parse_bms_data(self._bms_system.get_bms_data())

        battery_msg.voltage = self._bms_system.voltage
        battery_msg.current = self._bms_system.current
        battery_msg.cell_temperature = self._bms_system.temps
        battery_msg.percentage = self._bms_system.percent_capacity

        self._publisher.publish(battery_msg)
        # self.get_logger().info(f"Publishing voltage: {battery_msg.voltage}, current: {battery_msg.current}, cell_temperature: {battery_msg.cell_temperature}, percentage: {battery_msg.percentage}")

def main(args=None):
    rclpy.init(args=args)

    freya_bms_node = FreyaBMSNode()
    rclpy.spin(freya_bms_node)

    freya_bms_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    
