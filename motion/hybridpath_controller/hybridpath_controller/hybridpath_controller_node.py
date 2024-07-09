#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from hybridpath_controller.adaptive_backstep import AdaptiveBackstep
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from vortex_msgs.msg import HybridpathReference
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, qos_profile_sensor_data, QoSReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult


qos_profile = QoSProfile(depth=1, history=qos_profile_sensor_data.history, 
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)

class HybridPathControllerNode(Node):
    def __init__(self):
        super().__init__("hybridpath_controller_node")
        self.declare_parameters(
            namespace='',
            parameters=[
                ('hybridpath_controller.K1_diag', [10.0, 10.0, 10.0]),
                ('hybridpath_controller.K2_diag', [60.0, 60.0, 60.0]),
                ('physical.inertia_matrix', [50.0, 50.0, 50.0]),
                ('physical.damping_matrix_diag', [10.0, 10.0, 5.0])
            ])
        
        self.parameters_updated = False
        
        self.state_subscriber_ = self.state_subscriber_ = self.create_subscription(Odometry, '/seapath/odom/ned', self.state_callback, qos_profile=qos_profile)
        self.hpref_subscriber_ = self.create_subscription(HybridpathReference, 'guidance/hybridpath/reference', self.reference_callback, 1)
        self.wrench_publisher_ = self.create_publisher(Wrench, 'thrust/wrench_input', 1)

        # Debug publishers
        self.eta_error_publisher = self.create_publisher(Float64MultiArray, 'eta_error', 10)
        self.z1_publisher = self.create_publisher(Float64MultiArray, 'z1', 10)
        self.alpha1_publisher = self.create_publisher(Float64MultiArray, 'alpha1', 10)
        self.z2_publisher = self.create_publisher(Float64MultiArray, 'z2', 10)
        self.ds_alpha1_publisher = self.create_publisher(Float64MultiArray, 'ds_alpha1', 10)
        self.sigma1_publisher = self.create_publisher(Float64MultiArray, 'sigma1', 10)
        self.tau_publisher = self.create_publisher(Float64MultiArray, 'tau', 10)

        self.AB_controller_ = AdaptiveBackstep()

        self.update_controller_parameters()
        
        controller_period = 0.1
        self.controller_timer_ = self.create_timer(controller_period, self.controller_callback)
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info("hybridpath_controller_node started")

    def update_controller_parameters(self):

        K1_diag = self.get_parameter('hybridpath_controller.K1_diag').get_parameter_value().double_array_value
        K2_diag = self.get_parameter('hybridpath_controller.K2_diag').get_parameter_value().double_array_value
        D_diag = self.get_parameter('physical.damping_matrix_diag').get_parameter_value().double_array_value
        M = self.get_parameter('physical.inertia_matrix').get_parameter_value().double_array_value

        K1 = np.diag(K1_diag)
        K2 = np.diag(K2_diag)
        D = np.diag(D_diag)
        M = np.reshape(M, (3, 3))
        
        self.AB_controller_.update_parameters(K1, K2, M, D)

        self.get_logger().info("Updated controller parameters")

    def parameter_callback(self, params):
        self.parameters_updated = True
        for param in params:
            if param.name == 'hybridpath_controller.K1_diag':
                self.get_logger().info(f"Updated K1_diag to {param.value}")
            elif param.name == 'hybridpath_controller.K2_diag':
                self.get_logger().info(f"Updated K2_diag to {param.value}")
            elif param.name == 'physical.damping_matrix_diag':
                self.get_logger().info(f"Updated damping_matrix_diag to {param.value}")
            elif param.name == 'physical.inertia_matrix':
                self.get_logger().info(f"Updated inertia_matrix to {param.value}")

        # self.update_controller_parameters()
        return SetParametersResult(successful=True)

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
        if self.parameters_updated:
                self.update_controller_parameters()
                self.parameters_updated = False

        if hasattr(self, 'state_odom') and hasattr(self, 'reference'):
            control_input = self.AB_controller_.control_law(self.state_odom, self.reference)
            wrench_msg = Wrench()
            wrench_msg.force.x = control_input[0]
            wrench_msg.force.y = control_input[1]
            wrench_msg.torque.z = control_input[2]
            self.wrench_publisher_.publish(wrench_msg)

            # Debug publishers
            eta_error_msg = Float64MultiArray()
            eta_error_msg.data = self.AB_controller_.get_eta_error().tolist()
            self.eta_error_publisher.publish(eta_error_msg)

            z1_msg = Float64MultiArray()
            z1_msg.data = self.AB_controller_.get_z1().tolist()
            self.z1_publisher.publish(z1_msg)

            alpha1_msg = Float64MultiArray()
            alpha1_msg.data = self.AB_controller_.get_alpha1().tolist()
            self.alpha1_publisher.publish(alpha1_msg)

            z2_msg = Float64MultiArray()
            z2_msg.data = self.AB_controller_.get_z2().tolist()
            self.z2_publisher.publish(z2_msg)

            ds_alpha1_msg = Float64MultiArray()
            ds_alpha1_msg.data = self.AB_controller_.get_ds_alpha1().tolist()
            self.ds_alpha1_publisher.publish(ds_alpha1_msg)

            sigma1_msg = Float64MultiArray()
            sigma1_msg.data = self.AB_controller_.get_sigma1().tolist()
            self.sigma1_publisher.publish(sigma1_msg)

            tau_msg = Float64MultiArray()
            tau_msg.data = control_input.tolist()
            self.tau_publisher.publish(tau_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HybridPathControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
