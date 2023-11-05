import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import temperature.temperature_lib


class TemperaturePublisher(Node):

    def __init__(self):
        super().__init__('temperature_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.Temperature = temperature.temperature_lib.TemperatureModule()

    def timer_callback(self):
        msg = String()
        #msg.data = 'Hello World: %d' % self.i
        msg.data = self.Temperature.get_temperature()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    temperature_publisher = TemperaturePublisher()

    rclpy.spin(temperature_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    temperature_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()