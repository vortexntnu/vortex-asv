#!/usr/bin/env python3

import rclpy
from rclpy.node import Node as rclpy_node
import numpy as np
from d_star_lite import DStarLite, Node
from vortex_msgs.srv import MissionPlanner, Waypoint

class DStarLiteNode(rclpy_node):
    """
    A ROS2 node implementing the D* Lite algorithm.

    The node offers a mission planner service to calculate the optimal waypoints, which are then sent to the waypoint service.
    """
    def __init__(self):
        """
        Initialize the DStarLiteNode, creating necessary services and clients
        for mission planning and waypoint submission.
        """
        super().__init__('d_star_lite_node')
        self.obs_srv = self.create_service(MissionPlanner, 'mission_planner', self.d_star_lite_callback)
        self.wp_client = self.create_client(Waypoint, 'waypoint')
        self.get_logger().info('D Star Lite Node has been started')
        self.WP_float = []
        

    def d_star_lite_callback(self, request, response):
        """
        Callback for the mission planner service.

        Args:
            request: start and goal coordinates, and the obstacle coordinates
            response: success flag

        Returns:
            The modified response object with success status
        """
        self.get_logger().info('D Star Lite Service has been called')
        ox = request.ox
        oy = request.oy
        sx = request.sx
        sy = request.sy
        gx = request.gx
        gy = request.gy
        dsl = DStarLite(ox, oy)
        dsl.dsl_main(Node(sx, sy), Node(gx, gy))
        path = dsl.compute_current_path()
        WP = np.array(dsl.get_WP()).tolist()
        # Convert to float32[] for Waypoint service
        self.WP_float = [float(coordinate) for pair in WP for coordinate in pair]
        
        self.send_waypoints_request()

        response.success = True

        return response
    
    def send_waypoints_request(self):
        """
        Sends the computed waypoints to the waypoint service.
        """
        if not self.wp_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waypoint service not available, waiting again...')
            return None  # Return None to indicate the service is not available.
        request = Waypoint.Request()
        request.waypoint = self.WP_float
        future = self.wp_client.call_async(request)
        return future

def main(args=None):
    rclpy.init(args=args)
    node = DStarLiteNode()
    rclpy.spin(node)

    # Cleanup and shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()