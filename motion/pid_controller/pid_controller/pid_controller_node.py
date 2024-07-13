#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from pid_controller import PID
from conversions import odometrymsg_to_state
from time import sleep
from rcl_interfaces.msg import SetParametersResult

from rclpy.qos import QoSProfile, qos_profile_sensor_data, QoSReliabilityPolicy

qos_profile = QoSProfile(depth=1, history=qos_profile_sensor_data.history, 
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__("pid_controller_node")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('pid_controller.Kp', [1.0, 1.0, 1.0]),
                ('pid_controller.Ki', [1.0, 1.0, 1.0]),
                ('pid_controller.Kd', [1.0, 1.0, 1.0]),
                ])
        
        self.parameters_updated = False

        self.state_subscriber_ = self.create_subscription(Odometry, "/seapath/odom/ned", self.state_cb, qos_profile=qos_profile)
        self.guidance_subscriber_ = self.create_subscription(Odometry, "guidance/dp/reference", self.guidance_cb, 1)
        self.wrench_publisher_ = self.create_publisher(Wrench, "thrust/wrench_input", 1)

        ## PID TUNING VALUES ## (OVERWRITES YAML FILE VALUES)
        # M = self.get_parameter('physical.inertia_matrix').get_parameter_value().double_array_value
        
        # M = np.reshape(M, (3, 3))
        # M_diag = np.diag(M)
        # omega_n = 1.2
        # zeta = 0.75
        # Kp = M_diag * omega_n**2
        # self.get_logger().info(f"Kp: {Kp}")
        # Kd = M_diag * 2 * zeta * omega_n #- D_diag
        # self.get_logger().info(f"Kd: {Kd}")
        # Ki = omega_n/10 * Kp
        # self.get_logger().info(f"Ki: {Ki}")

        self.pid_controller = PID()

        self.update_controller_parameters()

        self.enabled = False

        self.controller_period = 0.1
        self.controller_timer_ = self.create_timer(self.controller_period, self.controller_callback)
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.get_logger().info("pid_controller_node started")

    def update_controller_parameters(self):
        Kp = self.get_parameter('pid_controller.Kp').get_parameter_value().double_array_value
        Ki = self.get_parameter('pid_controller.Ki').get_parameter_value().double_array_value
        Kd = self.get_parameter('pid_controller.Kd').get_parameter_value().double_array_value

        self.pid_controller.update_parameters(Kp, Ki, Kd)

        self.get_logger().info("Updated controller parameters")

    def parameter_callback(self, params):
        self.parameters_updated = True
        for param in params:
            if param.name == 'pid_controller.Kp':
                self.get_logger().info(f"Updated Kp to {param.value}")
            elif param.name == 'pid_controller.Ki':
                self.get_logger().info(f"Updated Ki to {param.value}")
            elif param.name == 'pid_controller.Kd':
                self.get_logger().info(f"Updated Kd to {param.value}")

        return SetParametersResult(successful=True)
    
    def state_cb(self, msg):
        self.state = odometrymsg_to_state(msg)

    def guidance_cb(self, msg):
        self.x_ref = odometrymsg_to_state(msg)[:3]

    def controller_callback(self):
        if hasattr(self, 'state') and hasattr(self, 'x_ref'):
            control_input = self.pid_controller.calculate_control_input(self.state[:3], self.x_ref, self.state[3:], self.controller_period)
            # self.get_logger().info(f"Control input: {control_input}")
            wrench_msg = Wrench()
            wrench_msg.force.x  = control_input[0]
            wrench_msg.force.y  = control_input[1]
            wrench_msg.torque.z = control_input[2]
            self.wrench_publisher_.publish(wrench_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
