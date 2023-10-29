import internal_status.power_sense_module_lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class MinimalPublisher(Node):


    def __init__(self):
        super().__init__('PSM_publisher')
        self.PSM = internal_status.power_sense_module_lib.PowerSenseModule()
        self.publisher_current = self.create_publisher(Float32, '/asv/power_sense_module/current', 1)
        self.publisher_voltage = self.create_publisher(Float32, '/asv/power_sense_module/voltage', 1)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        current = Float32()
        voltage = Float32()
        #call the two functions of the power_sense_module_lib to get the current and voltage values of the PSM
        current.data = self.PSM.get_current()
        voltage.data = self.PSM.get_voltage()
        self.publisher_current.publish(current)  #publish current value to the "current topic" 
        self.get_logger().info('Publishing PSM current: "%s"' % current.data)
        self.publisher_voltage.publish(voltage)  #publish voltage value to the "voltge topic"
        self.get_logger().info('Publishing PSM voltage: "%s"' % voltage.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


