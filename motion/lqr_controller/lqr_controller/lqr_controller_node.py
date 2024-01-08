import rclpy
from rclpy.node import Node

class LQRControllerNode(Node):
    def __init__(self):
        super().__init__("lqr_controller_node")
        self.get_logger().info("lqr_controller_node started")

def main(args=None):
    rclpy.init(args=args)
    node = LQRControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()