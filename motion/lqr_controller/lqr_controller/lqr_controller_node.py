#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from lqr_controller import LQRController
from conversions import odometrymsg_to_state
from time import sleep

from rclpy.qos import QoSProfile, qos_profile_sensor_data, QoSReliabilityPolicy

qos_profile = QoSProfile(depth=1, history=qos_profile_sensor_data.history, 
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)

class LQRControllerNode(Node):
    def __init__(self):
        super().__init__("lqr_controller_node")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('lqr_controller.Q', [10.0, 10.0, 5.0, 0.1, 1.0, 5.0]),
                ('lqr_controller.R', [1.0, 1.0, 1.0]),
                ('physical.inertia_matrix', [90.5, 0.0, 0.0, 0.0, 167.5, 12.25, 0.0, 12.25, 42.65]),
                ('physical.damping_matrix_diag', [77.55, 162.5, 42.65])
            ])
        
        self.state_subscriber_ = self.create_subscription(Odometry, "/sensor/seapath/odom/ned", self.state_cb, qos_profile=qos_profile)
        self.guidance_subscriber_ = self.create_subscription(Odometry, "guidance/dp/reference", self.guidance_cb, 1)
        self.wrench_publisher_ = self.create_publisher(Wrench, "thrust/wrench_input", 1)

        Q = self.get_parameter('lqr_controller.Q').get_parameter_value().double_array_value
        R = self.get_parameter('lqr_controller.R').get_parameter_value().double_array_value
        D_diag = self.get_parameter('physical.damping_matrix_diag').get_parameter_value().double_array_value
        M = self.get_parameter('physical.inertia_matrix').get_parameter_value().double_array_value

        D = np.diag(D_diag)
        M = np.reshape(M, (3, 3))

        heading_ref = 0.0

        self.LQR = LQRController(M, D, Q, R, heading_ref)

        self.x_ref = [0, 0, 0]
        self.state = [0, 0, 0, 0, 0, 0]

        self.enabled = False

        controller_period = 0.1
        self.controller_timer_ = self.create_timer(controller_period, self.controller_callback)

        self.get_logger().info("lqr_controller_node started")

    def state_cb(self, msg):
        self.state = odometrymsg_to_state(msg)

    def guidance_cb(self, msg):
        self.x_ref = odometrymsg_to_state(msg)[:3]

    def controller_callback(self):
        if hasattr(self, 'state') and hasattr(self, 'x_ref'):
            self.LQR.linearize_model(self.state[2])
            control_input = self.LQR.calculate_control_input(self.state, self.x_ref)
            wrench_msg = Wrench()
            wrench_msg.force.x  = control_input[0]
            wrench_msg.force.y  = control_input[1]
            wrench_msg.torque.z = control_input[2]
            self.wrench_publisher_.publish(wrench_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LQRControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
