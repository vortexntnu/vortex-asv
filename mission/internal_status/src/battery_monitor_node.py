import rclpy
from rclpy.node import Node

#from std_msgs.msg import String
from std_msgs.msg import Float32

system_voltage = 0.1
system_current = 0.2     #to remove when the lib is ready 

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Float32, 'topic', 1)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        current = Float32()
        voltage = Float32()
        # call the two functions of the lib to get the current and voltage values
        current.data = system_current
        voltage.data = system_voltage
        self.publisher_.publish(current)
        self.get_logger().info('Publishing: "%s"' % current.data)
        self.publisher_.publish(voltage)
        self.get_logger().info('Publishing: "%s"' % voltage.data)
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