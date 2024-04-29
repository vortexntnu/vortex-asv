#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from vortex_msgs.msg import HybridpathReference
from vortex_msgs.srv import Waypoint
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler
from hybridpath_guidance.hybridpath import HybridPathGenerator, HybridPathSignals
from rclpy.qos import QoSProfile, qos_profile_sensor_data, QoSReliabilityPolicy


qos_profile = QoSProfile(depth=1, history=qos_profile_sensor_data.history, 
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)

class Guidance(Node):
    def __init__(self):
        super().__init__("hp_guidance")
        self.declare_parameters(
            namespace='',
            parameters=[
                ('hybridpath_guidance.lambda_val', 0.15),
                ('hybridpath_guidance.path_generator_order', 1),
                ('hybridpath_guidance.dt', 0.1),
                ('hybridpath_guidance.mu', 0.2)
            ])
        
        self.waypoint_server = self.create_service(Waypoint, 'waypoint_list', self.waypoint_callback)
        self.eta_subscriber_ = self.create_subscription(Odometry, '/seapath/odom/ned', self.eta_callback, qos_profile=qos_profile)
        self.guidance_publisher = self.create_publisher(HybridpathReference, 'guidance/hybridpath/reference', 1)
        
        # Get parameters
        self.lambda_val = self.get_parameter('hybridpath_guidance.lambda_val').get_parameter_value().double_value
        self.path_generator_order = self.get_parameter('hybridpath_guidance.path_generator_order').get_parameter_value().integer_value
        self.dt = self.get_parameter('hybridpath_guidance.dt').get_parameter_value().double_value
        self.mu = self.get_parameter('hybridpath_guidance.mu').get_parameter_value().double_value
        self.eta = np.zeros(3)

        self.u_desired = 0.2 # Desired velocity

        # Flags for logging
        self.waypoints_received = False
        self.waiting_message_printed = False

        # Timer for guidance
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.guidance_callback)

    def waypoint_callback(self, request, response):
        self.get_logger().info('Received waypoints, generating path...')
        self.waypoints = request.waypoint

        generator = HybridPathGenerator(self.waypoints, self.path_generator_order, self.lambda_val)
        self.path = generator.path

        self.waypoints_received = True
        self.waiting_message_printed = False  # Reset this flag to handle multiple waypoint sets

        self.s = 0
        signals = HybridPathSignals(self.path, self.s)
        self.w = signals.get_w(self.mu, self.eta)
        
        response.success = True
        return response
    
    def eta_callback(self, msg: Odometry):
        self.eta = self.odom_to_eta(msg)

    def guidance_callback(self):
        if self.waypoints_received:
            self.s = HybridPathGenerator.update_s(self.path, self.dt, self.u_desired, self.s, self.w)
            signals = HybridPathSignals(self.path, self.s)
            self.w = signals.get_w(self.mu, self.eta)

            pos = signals.get_position()
            pos_der = signals.get_derivatives()[0]
            pos_dder = signals.get_derivatives()[1]

            psi = signals.get_heading()
            psi_der = signals.get_heading_derivative()
            psi_dder = signals.get_heading_second_derivative()

            hp_msg = HybridpathReference()
            hp_msg.eta_d = Pose2D(x=pos[0], y=pos[1], theta=psi)
            hp_msg.eta_d_s = Pose2D(x=pos_der[0], y=pos_der[1], theta=psi_der)
            hp_msg.eta_d_ss = Pose2D(x=pos_dder[0], y=pos_dder[1], theta=psi_dder)

            hp_msg.w = signals.get_w(self.mu, self.eta)
            hp_msg.v_s = signals.get_vs(self.u_desired)
            hp_msg.v_ss = signals.get_vs_derivative(self.u_desired)

            self.guidance_publisher.publish(hp_msg)

            if self.s >= self.path.NumSubpaths:
                self.waypoints_received = False
                self.waiting_message_printed = False
                self.get_logger().info('Last waypoint reached')

        else:
            if not self.waiting_message_printed:
                self.get_logger().info('Waiting for waypoints to be received')
                self.waiting_message_printed = True

    @staticmethod
    def odom_to_eta(msg: Odometry) -> np.ndarray:
        """
        Converts an Odometry message to 3DOF eta vector.

        Args:
            msg (Odometry): The Odometry message to convert.

        Returns:
            np.ndarray: The eta vector.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z
        ]

        # Convert quaternion to Euler angles
        yaw = quat2euler(orientation_list)[2]

        state = np.array([x, y, yaw])
        return state

def main(args=None):
    rclpy.init(args=args)
    guidance = Guidance()
    rclpy.spin(guidance)
    guidance.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
