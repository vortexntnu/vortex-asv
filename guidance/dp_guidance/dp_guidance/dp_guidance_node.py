#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from vortex_msgs.msg import HybridpathReference
from vortex_msgs.srv import Waypoint
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler

from dp_guidance.conversions import odometrymsg_to_state
from dp_guidance.hybridpath import HybridPathGenerator, HybridPathSignals

class Guidance(Node):
    def __init__(self):
        super().__init__("dp_guidance")
        self.declare_parameters(
            namespace='',
            parameters=[
                ('dp_guidance.dt', 0.1)
            ])
        
        self.waypoint_server = self.create_service(Waypoint, 'waypoint_list', self.waypoint_callback)
        self.eta_subscriber_ = self.state_subscriber_ = self.create_subscription(Odometry, '/sensor/seapath/odom/ned', self.eta_callback, 1)
        self.guidance_publisher = self.create_publisher(Odometry, 'guidance/dp/reference', 1)
        
        # Get parameters
        self.dt = self.get_parameter('dp_guidance.dt').get_parameter_value().double_value

        # Flags for logging
        self.waypoints_received = False
        self.waiting_message_printed = False

        self.eta_d = np.array([0, 0, 0])

        # Timer for guidance
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.guidance_callback)

    def waypoint_callback(self, request, response):
        self.get_logger().info('Received waypoints, generating path...')
        self.waypoints = request.waypoint
        self.eta_d = self.waypoints[0] # Choosing first waypoint as desired eta
        self.waypoints_received = True
        self.waiting_message_printed = False  # Reset this flag to handle multiple waypoint sets
        response.success = True
        return response
    
    def eta_callback(self, msg: Odometry):
        self.eta = odometrymsg_to_state(msg)[:3]

    def guidance_callback(self):
        if self.waypoints_received:
            
            odom_msg = Odometry()

            self.guidance_publisher.publish(odom_msg)

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
