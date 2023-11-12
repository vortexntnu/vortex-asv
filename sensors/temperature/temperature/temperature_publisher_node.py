import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32  # Import the Float32MultiArray message type

import temperature.temperature_lib  # Import your custom temperature library

class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__("temperature_publisher")
        # Create a publisher that publishes Float32MultiArray messages on the 'temperature_data' topic
        self.publisher_ESC1_ = self.create_publisher(Float32, "/asv/temperature/ESC1", 10)
        self.publisher_ESC2_ = self.create_publisher(Float32, "/asv/temperature/ESC2", 10)
        self.publisher_ESC3_ = self.create_publisher(Float32, "/asv/temperature/ESC3", 10)
        self.publisher_ESC4_ = self.create_publisher(Float32, "/asv/temperature/ESC4", 10)
        self.publisher_ambient1_ = self.create_publisher(Float32, "/asv/temperature/ambient1", 10)
        self.publisher_ambient2_ = self.create_publisher(Float32, "/asv/temperature/ambient2", 10)
        
        # Create a timer that calls the timer_callback method every 1.0 seconds
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize the TemperatureModule to interact with the temperature sensor
        self.TemperatureObjcet = temperature.temperature_lib.TemperatureModule()

    def timer_callback(self):
        # Get the temperature data from the sensor
        temperature_data_array = self.TemperatureObjcet.get_data()
        if temperature_data_array is not None:
            # If data is received, create a Float32 messages
            msg_ESC1 = Float32()
            msg_ESC2 = Float32()
            msg_ESC3 = Float32()
            msg_ESC4 = Float32()
            msg_ambient1 = Float32()
            msg_ambient2 = Float32()

            # Assign the received temperature data to the correct message
            msg_ESC1.data = temperature_data_array[0]
            msg_ESC2.data = temperature_data_array[1]
            msg_ESC3.data = temperature_data_array[2]
            msg_ESC4.data = temperature_data_array[3]
            msg_ambient1.data = temperature_data_array[5] # Inverse because of conections
            msg_ambient2.data = temperature_data_array[4] # Inverse because of conections

            # Publish the messages
            self.publisher_ESC1_.publish(msg_ESC1)
            self.publisher_ESC2_.publish(msg_ESC2)
            self.publisher_ESC3_.publish(msg_ESC3)
            self.publisher_ESC4_.publish(msg_ESC4)
            self.publisher_ambient1_.publish(msg_ambient1)
            self.publisher_ambient2_.publish(msg_ambient2)

            # Log the published data for debugging purposes
            self.get_logger().info(f"Temperature: {temperature_data_array}")

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS2 Python client library
    temperature_publisher = TemperaturePublisher()  # Create the TemperaturePublisher node
    rclpy.spin(temperature_publisher)  # Spin the node so the callback function is called
    # After shutdown, destroy the node and shutdown rclpy
    temperature_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()  # Run the main function if the script is executed
