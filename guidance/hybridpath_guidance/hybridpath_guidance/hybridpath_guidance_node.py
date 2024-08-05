#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Point
from std_msgs.msg import Float32, Float64MultiArray, String, Bool
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
        self.yaw_server = self.create_service(DesiredVelocity,'yaw_reference', self.yaw_ref_callback)
        self.operational_mode_subscriber = self.create_subscription(String, 'softWareOperationMode', self.operation_mode_callback, 10)
        self.killswitch_subscriber = self.create_subscription(Bool, 'softWareKillSwitch', self.killswitch_callback, 10)

        # Get parameters
        self.lambda_val = self.get_parameter('hybridpath_guidance.lambda_val').get_parameter_value().double_value
        self.path_generator_order = self.get_parameter('hybridpath_guidance.path_generator_order').get_parameter_value().integer_value
        self.dt = self.get_parameter('hybridpath_guidance.dt').get_parameter_value().double_value
        self.mu = self.get_parameter('hybridpath_guidance.mu').get_parameter_value().double_value
        self.eta = np.zeros(3)

        self.u_desired = 1.1 # Desired velocity

        self.yaw_ref = 0. # Desired heading, 100 if hybridpath heading, else can be set by service call

        self.waypoints = []
        self.path = None
        self.s = 0.
        self.w = 0.
        self.v_s = 0.
        self.v_ss = 0.
        self.operational_mode = 'autonomous mode'
        self.killswitch_active = False

        # Initialize path generator
        self.generator = HybridPathGenerator(self.waypoints, self.path_generator_order, self.lambda_val)

        # Initialize signals
        self.signals = HybridPathSignals()

        # Flags for logging
        self.waypoints_received = False
        self.waiting_message_printed = False
        self.first_pos_flag = False
        self.eta_received = False
        self.initial_pos = False

        # Timer for guidance
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.guidance_callback)

        self.lock = threading.Lock()

    def operation_mode_callback(self, msg: String):
        self.operational_mode = msg.data

    def killswitch_callback(self, msg: Bool):
        self.killswitch_active = msg.data

    def u_desired_callback(self, request, response):
        self.u_desired = request.u_desired
        self.get_logger().info(f"Received desired velocity: {self.u_desired}")
        response.success = True
        return response

    def yaw_ref_callback(self, request, response):
        self.yaw_ref = request.u_desired # xd

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

        if not self.initial_pos:
            self.eta_initial = self.eta
            self.yaw_ref = self.yaw
            self.initial_pos = True

        yaw_msg.data = self.yaw
        self.yaw_publisher.publish(yaw_msg)
        self.eta_received = True

    def guidance_callback(self):
        with self.lock:
            if self.killswitch_active or self.operational_mode != 'autonomous mode':
                return
            
            if self.initial_pos:
                if self.path is None and not self.waypoints_received:
                    pos = [self.eta_initial[0], self.eta_initial[1]]
                    pos_der = [0., 0.]
                    pos_dder = [0., 0.]

                else:
                    self.s = self.generator.update_s(self.path, self.dt, self.u_desired, self.s, self.w)
                    self.signals.update_s(self.s)
                    self.w = self.signals.get_w(self.mu, self.eta)
                    self.v_s = self.signals.get_vs(self.u_desired)
                    self.v_ss = self.signals.get_vs_derivative(self.u_desired)

                    pos = self.signals.get_position()

                    pos_der = self.signals.get_derivatives()[0]
                    pos_dder = self.signals.get_derivatives()[1]

                if self.yaw_ref == 100.:
                    psi = self.signals.get_heading()

                else:
                    psi = self.yaw_ref

                psi_der = 0.#signals.get_heading_derivative()
                psi_dder = 0.#signals.get_heading_second_derivative()

                hp_msg = HybridpathReference()
                hp_msg.eta_d = Pose2D(x=pos[0], y=pos[1], theta=psi)
                hp_msg.eta_d_s = Pose2D(x=pos_der[0], y=pos_der[1], theta=psi_der)
                hp_msg.eta_d_ss = Pose2D(x=pos_dder[0], y=pos_dder[1], theta=psi_dder)

                hp_msg.w = self.w
                hp_msg.v_s = self.v_s
                hp_msg.v_ss = self.v_ss

                self.guidance_publisher.publish(hp_msg)

                if self.path is not None and self.s >= self.path.NumSubpaths:
                    self.waypoints_received = False
                    self.waiting_message_printed = False
                    self.path = None
                    self.eta_initial = self.eta
                    self.get_logger().info('Last waypoint reached')

            else:
                if not self.waiting_message_printed:
                    self.get_logger().info('Waiting for eta')
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
