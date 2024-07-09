#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from vortex_msgs.srv import Waypoint
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler

from conversions import odometrymsg_to_state, state_to_odometrymsg
from reference_filter import ReferenceFilter
from rclpy.qos import QoSProfile, qos_profile_sensor_data, QoSReliabilityPolicy

qos_profile = QoSProfile(depth=1, history=qos_profile_sensor_data.history, 
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)

class Guidance(Node):
    def __init__(self):
        super().__init__("dp_guidance")
        self.declare_parameters(
            namespace='',
            parameters=[
                ('dp_guidance.dt', 0.1)
            ])
        
        self.waypoint_server = self.create_service(Waypoint, 'waypoint_list', self.waypoint_callback)
        self.eta_subscriber_ = self.state_subscriber_ = self.create_subscription(Odometry, '/seapath/odom/ned', self.eta_callback, qos_profile=qos_profile)
        self.guidance_publisher = self.create_publisher(Odometry, 'guidance/dp/reference', 1)
        
        # Get parameters
        self.dt = self.get_parameter('dp_guidance.dt').get_parameter_value().double_value

        # Flags for logging
        self.waypoints_received = False
        self.waiting_message_printed = False

        self.init_pos = False
        self.eta_received = False
        self.eta_logger = True

        self.eta = np.array([0, 0, 0])
        self.eta_ref = np.array([0, 0, 0])

        self.xd = np.zeros(9)

        self.reference_filter = ReferenceFilter()

        # Timer for guidance
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.guidance_callback)

    def waypoint_callback(self, request, response):
        self.waypoints = request.waypoint
        self.get_logger().info(f'Received waypoints: {self.waypoints}')
        self.waypoints_received = True
        self.waiting_message_printed = False  # Reset this flag to handle multiple waypoint sets
        response.success = True
        return response
    
    def eta_callback(self, msg: Odometry):
        self.eta = odometrymsg_to_state(msg)[:3]
        self.eta_received = True

    def guidance_callback(self):
        if self.waypoints_received and self.eta_received:
            if not self.init_pos:
                self.xd[0:3] = self.eta
                self.get_logger().info(f"Reference initialized at {self.xd[0:3]}")
                self.init_pos = True
            last_waypoint = self.waypoints[-1]
            self.eta_ref = np.array([last_waypoint.x, last_waypoint.y, self.eta[2]])
            x_next = self.reference_filter.step(self.eta_ref, self.xd)
            self.xd = x_next
            # self.get_logger().info(f'x_next[0]: {x_next[0]}')
            # self.get_logger().info(f'x_next[0]: {x_next[1]}')
            # self.get_logger().info(f'x_next[0]: {x_next[2]}')

            odom_msg = Odometry()
            # odom_msg = state_to_odometrymsg(x_next[:3])
            odom_msg = state_to_odometrymsg(self.eta_ref)
            self.guidance_publisher.publish(odom_msg)

        else:
            if not self.eta_received and self.eta_logger:
                self.get_logger().info("Waiting for eta")
                self.eta_logger = False

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
