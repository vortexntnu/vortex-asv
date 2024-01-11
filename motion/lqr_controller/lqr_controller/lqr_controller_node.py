import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler
from lqr_controller.lqr_controller import LQRController

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
        
        self.wrench_publisher_ = self.create_publisher(Wrench, "thrust/wrench_input", 1)
        self.state_subscriber_ = self.create_subscription(Odometry, "/sensor/seapath/odometry/ned", self.state_cb, 1)
        self.guidance_subscriber_ = self.create_subscription(Odometry, "controller/lqr/reference", self.guidance_cb, 1)

        m = self.get_parameter('lqr_controller.mass').get_parameter_value().double_value
        D = self.get_parameter('lqr_controller.D').get_parameter_value().double_array_value
        Q = self.get_parameter('lqr_controller.Q').get_parameter_value().double_array_value
        R = self.get_parameter('lqr_controller.R').get_parameter_value().double_array_value

        self.get_logger().info(f"Mass: {m}")
        self.get_logger().info(f"D: {D}")
        self.get_logger().info(f"Q: {Q}")
        self.get_logger().info(f"R: {R}")

        self.lqr_controller = LQRController(m, D, Q, R)
        #self.lqr_controller.run_ivan_sim()

        # Using x, y, yaw as reference (1x3)
        self.x_ref = [0, 0, 0]

        self.get_logger().info("lqr_controller_node started")

    def odometrymsg_to_state(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z
        ]

        # Convert quaternion to Euler angles, extract yaw
        yaw = quat2euler(orientation_list)[2]

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vyaw = msg.twist.twist.angular.z

        state = np.array([x, y, yaw, vx, vy, vyaw])
        return state
    
    def guidance_cb(self, msg):
        self.x_ref = self.odometrymsg_to_state(msg)[:3]

    def state_cb(self, msg):

        state = self.odometrymsg_to_state(msg)

        self.lqr_controller.linearize_model(state[2])

        # Run LQR 
        u = self.lqr_controller.calculate_control_input(state, self.x_ref, self.lqr_controller.K_LQR, self.lqr_controller.K_r)

        wrench = Wrench()
        wrench.force.x  = u[0]
        wrench.force.y  = u[1]
        wrench.force.z  = 0.0
        wrench.torque.x = 0.0
        wrench.torque.y = 0.0
        wrench.torque.z = u[2]

        # Publish thrust/wrench_input
        self.wrench_publisher_.publish(wrench)


def main(args=None):
    rclpy.init(args=args)
    node = LQRControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
