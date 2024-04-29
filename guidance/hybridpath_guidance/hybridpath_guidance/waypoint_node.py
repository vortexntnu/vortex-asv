#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vortex_msgs.srv import Waypoint
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler
import numpy as np

class WaypointClient(Node):
    def __init__(self):
        super().__init__('waypoint_client')
        self.eta_odom = self.create_subscription(Odometry, '/seapath/odom/ned', self.eta_callback, 1)

        self.eta_received = False

        while not self.eta_received:
            self.get_logger().info('Waiting for eta...')
            rclpy.spin_once(self)

        self.client = self.create_client(Waypoint, 'waypoint_list')

        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting again...')

        self.send_request()

    def eta_callback(self, msg: Odometry):
        self.eta = self.odom_to_eta(msg)
        #self.get_logger().info(f'Received eta {self.eta}')
        self.eta_received = True

    def send_request(self):
        if self.eta_received:
            req = Waypoint.Request()
            wp_list = [[self.eta[0], self.eta[1]], [self.eta[0] + 3., self.eta[1] + 3]]
            req.waypoint = [Point(x=float(wp[0]), y=float(wp[1]), z=0.0) for wp in wp_list]
            self.get_logger().info(f'Sending request: {req}')
            self.future = self.client.call_async(req)
            self.future.add_done_callback(self.get_response)

    def get_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Received response: {response}')
            if response.success:
                self.destroy_node()
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))

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
    waypoint_client_node = WaypointClient()
    try:
        rclpy.spin(waypoint_client_node)
    except Exception as e:
        waypoint_client_node.get_logger().error(f'Unhandled exception: {e}')
    finally:
        if rclpy.ok():
            waypoint_client_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()