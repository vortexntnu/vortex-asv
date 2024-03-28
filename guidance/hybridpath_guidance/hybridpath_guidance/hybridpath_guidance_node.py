#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from vortex_msgs.msg import HybridpathReference
from vortex_msgs.srv import Waypoint

from hybridpath_guidance.hybridpath import HybridPathGenerator, HybridPathSignals

class Guidance(Node):
    def __init__(self):
        super().__init__("hp_guidance")
        self.declare_parameters(
            namespace='',
            parameters=[
                ('hybridpath_guidance.lambda_val', 0.15),
                ('hybridpath_guidance.path_generator_order', 1),
                ('hybridpath_guidance.time_to_max_speed', 10.0),
                ('hybridpath_guidance.dt', 0.1),
                ('hybridpath_guidance.u_desired', 0.5)
            ])
        
        self.waypoint_server = self.create_service(Waypoint, 'waypoint_list', self.waypoint_callback)
        self.guidance_publisher = self.create_publisher(HybridpathReference, 'guidance/hybridpath/reference', 1)
        
        # Get parameters
        self.lambda_val = self.get_parameter('hybridpath_guidance.lambda_val').get_parameter_value().double_value
        self.path_generator_order = self.get_parameter('hybridpath_guidance.path_generator_order').get_parameter_value().integer_value
        self.dt = self.get_parameter('hybridpath_guidance.dt').get_parameter_value().double_value
        self.u_desired = self.get_parameter('hybridpath_guidance.u_desired').get_parameter_value().double_value

        # Flags for logging
        self.waypoints_received = False
        self.waiting_message_printed = False

        # Timer for guidance
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.guidance_callback)

    def waypoint_callback(self, request, response):
        self.get_logger().info('Received waypoints, generating path...')
        waypoints = request.waypoint
        self.waypoints_array = np.array([[waypoint.x, waypoint.y] for waypoint in waypoints])
        generator = HybridPathGenerator(self.waypoints_array, self.path_generator_order, self.lambda_val)
        self.path = generator.Path
        self.waypoints_received = True
        self.waiting_message_printed = False  # Reset this flag to handle multiple waypoint sets
        self.s = 0
        response.success = True
        return response

    def guidance_callback(self):
        if self.waypoints_received:
            self.s += HybridPathGenerator.update_s(self.path, self.dt, self.u_desired, self.s)
            signals = HybridPathSignals(self.path, self.s)
            pos = signals.pd
            pos_der = signals.pd_der[0]
            pos_dder = signals.pd_der[1]
            psi = signals.get_heading()
            psi_der = signals.get_heading_derivative()
            psi_dder = signals.get_heading_second_derivative()

            hp_msg = HybridpathReference()
            hp_eta = Pose2D(x=pos[0], y=pos[1], theta=psi)
            hp_eta_d = Pose2D(x=pos_der[0], y=pos_der[1], theta=psi_der)
            hp_eta_dd = Pose2D(x=pos_dder[0], y=pos_dder[1], theta=psi_dder)
            hp_msg.eta_d = hp_eta
            hp_msg.eta_d_s = hp_eta_d
            hp_msg.eta_d_ss = hp_eta_dd

            self.guidance_publisher.publish(hp_msg)

            if self.s >= self.path['NumSubpaths']:
                self.waypoints_received = False
                self.waiting_message_printed = False
                self.get_logger().info('Last waypoint reached')

        else:
            if not self.waiting_message_printed:
                self.get_logger().info('Waiting for waypoints to be received')
                self.waiting_message_printed = True

def main(args=None):
    rclpy.init(args=args)
    guidance = Guidance()
    rclpy.spin(guidance)
    guidance.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
