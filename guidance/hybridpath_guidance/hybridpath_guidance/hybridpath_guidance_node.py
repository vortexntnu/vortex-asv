#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Point
from std_msgs.msg import Float32, Float64MultiArray
from vortex_msgs.msg import HybridpathReference
from vortex_msgs.srv import Waypoint, DesiredVelocity
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler
from hybridpath_guidance.hybridpath import HybridPathGenerator, HybridPathSignals
from rclpy.qos import QoSProfile, qos_profile_sensor_data, QoSReliabilityPolicy
import threading


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
        self.u_desired_server = self.create_service(DesiredVelocity, 'u_desired', self.u_desired_callback)
        self.eta_subscriber_ = self.create_subscription(Odometry, '/seapath/odom/ned', self.eta_callback, qos_profile=qos_profile)
        self.yaw_publisher = self.create_publisher(Float32, 'yaw', 1)
        self.s_publisher = self.create_publisher(Float32, 's', 1)
        self.w_publisher = self.create_publisher(Float32, 'w', 1)
        self.wp_arr_publisher = self.create_publisher(Float64MultiArray, 'waypoints', 1)
        self.guidance_publisher = self.create_publisher(HybridpathReference, 'guidance/hybridpath/reference', 1)
        
        # Get parameters
        self.lambda_val = self.get_parameter('hybridpath_guidance.lambda_val').get_parameter_value().double_value
        self.path_generator_order = self.get_parameter('hybridpath_guidance.path_generator_order').get_parameter_value().integer_value
        self.dt = self.get_parameter('hybridpath_guidance.dt').get_parameter_value().double_value
        self.mu = self.get_parameter('hybridpath_guidance.mu').get_parameter_value().double_value
        self.eta = np.zeros(3)

        self.u_desired = 0.3 # Desired velocity

        self.waypoints = []
        self.path = None
        self.s = 0.
        self.w = 0.

        # Initialize path generator
        self.generator = HybridPathGenerator(self.waypoints, self.path_generator_order, self.lambda_val)

        # Initialize signals
        self.signals = HybridPathSignals()

        # Flags for logging
        self.waypoints_received = False
        self.waiting_message_printed = False
        self.first_pos_flag = False
        self.eta_received = False

        # Timer for guidance
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.guidance_callback)

        self.lock = threading.Lock()

    def u_desired_callback(self, request, response):
        self.u_desired = request.u_desired
        self.get_logger().info(f"Received desired velocity: {self.u_desired}")
        response.success = True
        return response


    def waypoint_callback(self, request, response):

        with self.lock:
            if self.eta_received:
                self.waypoints = [Point(x=self.eta[0], y=self.eta[1])]

                self.get_logger().info('Received waypoints, generating path...')

                new_waypoints = request.waypoint

                for point in new_waypoints:
                    self.waypoints.append(point)

                self.generator.create_path(self.waypoints)
                self.path = self.generator.get_path()

                self.waypoints_received = True
                self.waiting_message_printed = False  # Reset this flag to handle multiple waypoint sets
                self.first_pos_flag = False

                self.s = 0.
                self.signals.update_path(self.path)
                self.w = self.signals.get_w(self.mu, self.eta)

                wp_arr = Float64MultiArray()
                wp_list = self.generator.WP.tolist()
                wp_arr.data = [coordinate for wp in wp_list for coordinate in wp]
                self.wp_arr_publisher.publish(wp_arr)
            
        response.success = True

        return response
    
    def eta_callback(self, msg: Odometry):
        yaw_msg = Float32()
        self.eta = self.odom_to_eta(msg)
        self.yaw = float(self.eta[2])
        yaw_msg.data = self.yaw
        self.yaw_publisher.publish(yaw_msg)
        self.eta_received = True

    def guidance_callback(self):
        with self.lock:
            if self.waypoints_received:
                self.s = self.generator.update_s(self.path, self.dt, self.u_desired, self.s, self.w)
                self.signals.update_s(self.s)
                self.w = self.signals.get_w(self.mu, self.eta)

                pos = self.signals.get_position()

                if not self.first_pos_flag:
                    self.get_logger().info(f"First position: {pos}")
                    self.first_pos_flag = True

                # pos[0] = self.eta[0]
                pos_der = self.signals.get_derivatives()[0]
                pos_dder = self.signals.get_derivatives()[1]

                psi = 0. #signals.get_heading()
                psi_der = 0.#signals.get_heading_derivative()
                psi_dder = 0.#signals.get_heading_second_derivative()

                hp_msg = HybridpathReference()
                hp_msg.eta_d = Pose2D(x=pos[0], y=pos[1], theta=psi)
                hp_msg.eta_d_s = Pose2D(x=pos_der[0], y=pos_der[1], theta=psi_der)
                hp_msg.eta_d_ss = Pose2D(x=pos_dder[0], y=pos_dder[1], theta=psi_dder)

                hp_msg.w = self.signals.get_w(self.mu, self.eta)
                hp_msg.v_s = self.signals.get_vs(self.u_desired)
                hp_msg.v_ss = self.signals.get_vs_derivative(self.u_desired)

                self.guidance_publisher.publish(hp_msg)

                if self.s >= self.path.NumSubpaths:
                    self.waypoints_received = False
                    self.waiting_message_printed = False
                    self.get_logger().info('Last waypoint reached')

            else:
                if not self.waiting_message_printed:
                    self.get_logger().info('Waiting for waypoints to be received')
                    self.waiting_message_printed = True

        s_msg = Float32()
        s_msg.data = self.s
        self.s_publisher.publish(s_msg)

        w_msg = Float32()
        w_msg.data = self.w
        self.w_publisher.publish(w_msg)

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

        # yaw = np.deg2rad(yaw)

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
