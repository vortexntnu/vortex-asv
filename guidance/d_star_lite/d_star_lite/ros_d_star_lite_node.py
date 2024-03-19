#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
import numpy as np
from d_star_lite.d_star_lite import DStarLite
from d_star_lite.d_star_lite_node import DSLNode
from vortex_msgs.srv import MissionParameters, Waypoint
from geometry_msgs.msg import Point

class DStarLiteNode(Node):
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
        self.obstacle_srv = self.create_service(MissionParameters, 'mission_parameters', self.d_star_lite_callback)
        self.waypoint_client = self.create_client(Waypoint, 'waypoint')
        self.get_logger().info('D Star Lite Node has been started')
        self.waypoint_list = []
        

    def d_star_lite_callback(self, request, response):
        """
        Callback for the mission planner service.

        Args:
            request: start and goal coordinates, the obstacle coordinates and the world boundaries
            response: success flag

        Returns:
            The modified response object with success status
        """
        self.get_logger().info('D Star Lite Service has been called')
        obstacles = request.obstacles
        obstacles_x = [obs.x for obs in obstacles]
        obstacles_y = [obs.y for obs in obstacles]
        start = request.start
        goal = request.goal
        origin = request.origin
        height = request.height
        width = request.width
        start_node = DSLNode(int(start.x), int(start.y))
        goal_node = DSLNode(int(goal.x), int(goal.y))
        origin = (origin.x, origin.y)
        
        dsl = DStarLite(obstacles_x, obstacles_y, start_node, goal_node, origin=origin, height=height, width=width)
        dsl.dsl_main() # Run the main function to generate path
        
        waypoints_list = np.array(dsl.get_WP()).tolist()
        # Convert to Point2D[] for Waypoint service
        self.waypoints = [Point(x=float(wp[0]), y=float(wp[1])) for wp in waypoints_list]
        # Send waypoints to waypoint service
        self.send_waypoints_request()

        response.success = True

        return response
    
    def send_waypoints_request(self):
        """
        Sends the computed waypoints to the waypoint service.
        """
        if not self.waypoint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waypoint service not available, waiting again...')
            return None  # Return None to indicate the service is not available.
        request = Waypoint.Request()
        request.waypoint = self.waypoint_list
        future = self.waypoint_client.call_async(request)
        return future

def main(args=None):
    rclpy.init(args=args)
    node = DStarLiteNode()
    rclpy.spin(node)

    # Cleanup and shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()