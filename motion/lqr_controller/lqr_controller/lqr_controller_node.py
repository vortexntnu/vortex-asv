import rclpy
from rclpy.node import Node
from lqr_controller import LQRController

class LQRControllerNode(Node):
    def __init__(self):
        super().__init__("lqr_controller_node")

        # Load parameters from lqr_config.yaml 
        self.declare_parameters(
            namespace='',
            parameters=[
                ('lqr_controller.mass', 1.0),
                ('lqr_controller.D', [0.0, 0.0, 0.0]),
                ('lqr_controller.Q', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                ('lqr_controller.R', [0.0, 0.0, 0.0])
            ])

        m = self.get_parameter('lqr_controller.mass').get_parameter_value().double_value
        D = self.get_parameter('lqr_controller.D').get_parameter_value().double_array_value
        Q = self.get_parameter('lqr_controller.Q').get_parameter_value().double_array_value
        R = self.get_parameter('lqr_controller.R').get_parameter_value().double_array_value

        self.get_logger().info(f"Mass: {m}")
        self.get_logger().info(f"D: {D}")
        self.get_logger().info(f"Q: {Q}")
        self.get_logger().info(f"R: {R}")

        lqr_controller = LQRController(m, D, Q, R)
        lqr_controller.run_ivan_sim()

        self.get_logger().info("lqr_controller_node started")

def main(args=None):
    rclpy.init(args=args)
    node = LQRControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
