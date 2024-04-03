#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from hybridpath_controller.adaptive_backstep import AdaptiveBackstep
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from vortex_msgs.msg import HybridpathReference

class HybridPathControllerNode(Node):
    def __init__(self):
        super().__init__("hybridpath_controller_node")
        self.declare_parameters(
            namespace='',
            parameters=[
                ('hybridpath_controller.K1_diag', [10.0, 10.0, 10.0]),
                ('hybridpath_controller.K2_diag', [60.0, 60.0, 60.0]),
                ('physical.inertia_matrix_diag', [50.0, 50.0, 50.0]),
                ('physical.damping_matrix_diag', [10.0, 10.0, 5.0])
            ])
        
        self.state_subscriber_ = self.state_subscriber_ = self.create_subscription(Odometry, '/sensor/seapath/odometry/ned', self.state_callback, 1)
        self.hpref_subscriber_ = self.create_subscription(HybridpathReference, 'guidance/hybridpath/reference', self.reference_callback, 1)
        self.wrench_publisher_ = self.create_publisher(Wrench, 'thrust/wrench_input', 1)

        # Get parameters
        K1_diag = self.get_parameter('hybridpath_controller.K1_diag').get_parameter_value().double_array_value
        K2_diag = self.get_parameter('hybridpath_controller.K2_diag').get_parameter_value().double_array_value
        M_diag = self.get_parameter('physical.inertia_matrix_diag').get_parameter_value().double_array_value
        D_diag = self.get_parameter('physical.damping_matrix_diag').get_parameter_value().double_array_value

        K1_diag = [10.0, 10.0, 10.0]
        K2_diag = [60.0, 60.0, 60.0]
        M_diag = [50.0, 50.0, 50.0]
        D_diag = [10.0, 10.0, 5.0]

        # Transform parameters to diagonal matrices
        K1 = np.diag(K1_diag)
        K2 = np.diag(K2_diag)
        M = np.diag(M_diag)
        D = np.diag(D_diag)

        # Create controller object
        self.AB_controller_ = AdaptiveBackstep(K1, K2, M, D)

        controller_period = 0.1
        self.controller_timer_ = self.create_timer(controller_period, self.controller_callback)

        self.get_logger().info("hybridpath_controller_node started")

    def state_callback(self, msg: Odometry):
        """
        Callback function for the Odometry message. This function saves the state message.
        """
        self.state_odom = msg

    def reference_callback(self, msg: HybridpathReference):
        self.reference = msg

    def controller_callback(self):
        """
        Callback function for the controller timer. This function calculates the control input and publishes the control input.
        """
        if hasattr(self, 'state_odom') and hasattr(self, 'reference'):
            control_input = self.AB_controller_.control_law(self.state_odom, self.reference)
            wrench_msg = Wrench()
            wrench_msg.force.x = control_input[0]
            wrench_msg.force.y = control_input[1]
            wrench_msg.torque.z = control_input[2]
            self.wrench_publisher_.publish(wrench_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HybridPathControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
