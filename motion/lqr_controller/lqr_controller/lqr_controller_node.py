import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler
from lqr_controller.lqr_controller import LQRController
from time import sleep

from rclpy.qos import QoSProfile, qos_profile_sensor_data, QoSHistoryPolicy, QoSReliabilityPolicy
qos_profile = QoSProfile(depth=1, history=qos_profile_sensor_data.history, 
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)

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
        self.state_subscriber_ = self.create_subscription(Odometry, "/sensor/seapath/odom/ned", self.state_cb, qos_profile=qos_profile)
        # self.guidance_subscriber_ = self.create_subscription(Odometry, "controller/lqr/reference", self.guidance_cb, 1)

        m = self.get_parameter('lqr_controller.mass').get_parameter_value().double_value
        D = self.get_parameter('lqr_controller.D').get_parameter_value().double_array_value
        Q = self.get_parameter('lqr_controller.Q').get_parameter_value().double_array_value
        R = self.get_parameter('lqr_controller.R').get_parameter_value().double_array_value

        # self.get_logger().info(f"Mass: {m}")
        # self.get_logger().info(f"D: {D}")
        # self.get_logger().info(f"Q: {Q}")
        # self.get_logger().info(f"R: {R}")

        self.lqr_controller = LQRController(m, D, Q, R)

        # Using x, y, yaw as reference (1x3)
        self.x_ref = [0, 0, 0]
        self.state = [0, 0, 0, 0, 0, 0]

        self.enabled = False

        controller_period = 0.1
        self.controller_timer_ = self.create_timer(controller_period, self.controller_callback)

        self.test_oneMeterAhead()

        self.get_logger().info("lqr_controller_node started")

    def test_oneMeterAhead(self):
        self.get_logger().info("Sleeping 3 secs to get odom state")
        sleep(3)
        self.x_ref = [self.state[0] + 1, self.state[1], self.state[2]]
        self.enabled = True
        self.get_logger().info("LQR enabled")

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
    
    # def guidance_cb(self, msg):
    #     self.x_ref = self.odometrymsg_to_state(msg)[:3]

    #     #wrench = self.run_lqr_to_wrench()
    #     #self.wrench_publisher_.publish(wrench)

    def state_cb(self, msg):
        self.state = self.odometrymsg_to_state(msg)

    def controller_callback(self):
        """
        Callback function for the controller timer. This function calculates the control input and publishes the control input.
        """
        if self.enabled:
            wrench = self.run_lqr_to_wrench()
            self.wrench_publisher_.publish(wrench)

    def run_lqr_to_wrench(self):
        self.lqr_controller.linearize_model(self.state[2])
        u = self.lqr_controller.calculate_control_input(self.state, self.x_ref, self.lqr_controller.K_LQR, self.lqr_controller.K_r)

        wrench = Wrench()
        wrench.force.x  = u[0]
        wrench.force.y  = u[1]
        wrench.force.z  = 0.0
        wrench.torque.x = 0.0
        wrench.torque.y = 0.0
        wrench.torque.z = u[2]

        return wrench


def main(args=None):
    rclpy.init(args=args)
    node = LQRControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
