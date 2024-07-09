import rclpy
from rclpy.node import Node
from vortex_msgs.srv import DesiredVelocity

class DesiredVelocityClient(Node):
    def __init__(self):
        super().__init__('desired_velocity_client')
        self.client = self.create_client(DesiredVelocity, 'u_desired')

        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting again...')

        self.send_request()

    def send_request(self):
        req = DesiredVelocity.Request()
        req.u = 0.3
        self.get_logger().info(f'Sending request: {req}')
        self.future = self.client.call_async(req)
        self.future.add_done_callback(self.get_response)

    def get_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Received response: {response}')
            if response.success:
                self.destroy_node()
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))

def main(args=None):
    rclpy.init(args=args)

    client = DesiredVelocityClient()

    try:
        rclpy.spin(client)
    except Exception as e:
        client.get_logger().error('Error in DesiredVelocityClient: %r' % (e,))
    finally:
        if rclpy.ok():
            client.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()