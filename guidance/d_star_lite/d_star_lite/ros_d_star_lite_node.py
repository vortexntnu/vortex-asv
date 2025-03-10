#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vortex_msgs.srv import MissionParameters, Waypoint

from d_star_lite.d_star_lite import DStarLite


class DStarLiteNode(Node):
    """A ROS2 node implementing the D* Lite algorithm.

    The node offers a mission planner service to calculate the optimal waypoints, which are then sent to the waypoint service.
    """

    def __init__(self):
        """Initialize the DStarLiteNode.

        Creates necessary services and clients for mission planning and waypoint submission.
        """
        super().__init__('d_star_lite_node')
        self.obstacle_srv = self.create_service(
            MissionParameters, 'mission_parameters', self.d_star_lite_callback
        )
        self.waypoint_client = self.create_client(Waypoint, 'waypoint')
        self.get_logger().info('D Star Lite Node has been started')

    def d_star_lite_callback(self, request, response):
        """Callback for the mission planner service.

        Args:
            request: start and goal coordinates, the obstacle coordinates and the world boundaries
            response: success flag

        Returns:
            The modified response object with success status
        """
        self.get_logger().info('D Star Lite Service has been called')
        obstacles = request.obstacles
        start = request.start
        goal = request.goal
        origin = request.origin
        height = request.height
        width = request.width

        dsl = DStarLite(
            obstacles=obstacles,
            start=start,
            goal=goal,
            origin=origin,
            height=height,
            width=width,
        )
        dsl.dsl_main()  # Run the main function to generate path

        # Get waypoints
        self.waypoints = dsl.get_waypoints()

        # Send waypoints to waypoint service
        self.send_waypoints_request()

        response.success = True

        return response

    def send_waypoints_request(self):
        """Sends the computed waypoints to the waypoint service."""
        if not self.waypoint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waypoint service not available, waiting again...')
            return None  # Return None to indicate the service is not available.
        request = Waypoint.Request()
        request.waypoint = self.waypoints
        future = self.waypoint_client.call_async(request)
        future.add_done_callback(self.waypoint_response_callback)  # Handle response

    def waypoint_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Waypoints successfully submitted.')
            else:
                self.get_logger().error('Waypoint submission failed.')
        except Exception as e:
            self.get_logger().error(f'Service call failed {e!r}')


def main(args=None):
    rclpy.init(args=args)
    node = DStarLiteNode()
    rclpy.spin(node)

    # Cleanup and shutdown
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
