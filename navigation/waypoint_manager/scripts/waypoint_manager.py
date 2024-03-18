#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vortex_msgs.srv import Waypoint
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from scripts.Waypoint2D import Waypoint2D


class WaypointManager(Node):
    """
    Nodes created: WaypointManager
    Subscribes to: d_star_lite
    Publishes to: hybridpath_guidance
    """
    
    def __init__(self):
        """
        Initializes the WaypointManager node, sets up services, clients, and publishers.
        """

        super().__init__("WaypointManager")

        self.waypoint_list = []

        self.waypoint_service = self.create_service(Waypoint, 'waypoint', self.waypoint_callback)
        self.add_waypoint_service = self.create_service(Waypoint, 'add_waypoint', self.add_waypoint_callback)
        self.waypoint_list_client = self.create_client(Waypoint, 'waypoint_list')

        self.path = Path()
        self.path.header.frame_id = 'world'

    def waypoint_callback(self, request, response):
        """
        Clears the current waypoint list and adds new waypoints from the request.

        Args:
            request: The request message containing waypoints.
            response: The response message indicating the success of the operation.

        Returns:
            The response message with the success status.
        """

        self.waypoint_list.clear()
        return self.add_waypoint_to_list(request)
    
   
    def add_waypoint_to_list(self, req):
        """
        Adds waypoints to the waypoint list from the request.

        This method processes a request containing a list of waypoints, converting them into `Waypoint2D` objects,
        and appending them to the waypoint list. It also updates the `path` with new poses.

        Args:
            req (Waypoint.Request): Request containing the list of waypoints to be added.

        Returns:
            Waypoint.Response: A response object indicating success.
        """

        new_waypoints = list(zip(req.waypoint[::2], req.waypoint[1::2]))
        self.waypoint_list.extend([Waypoint2D(north=n, east=e) for n, e in new_waypoints])

        for waypoint in self.waypoint_list:
            new_pose = PoseStamped()
            new_pose.pose.position.x = waypoint.north  
            new_pose.pose.position.y = waypoint.east   
            new_pose.pose.position.z = 0.0  
            self.path.poses.append(new_pose)

        response = Waypoint.Response()
        response.success = True
        return response

    def add_waypoint_callback(self, request, response):
        """
        Callback function for the add_waypoint service.

        Forwards the request to `add_waypoint_to_list` method.

        Args:
            request: The request object containing new waypoints.
            response: The response object to be filled by `add_waypoint_to_list`.

        Returns:
            The response from `add_waypoint_to_list`.
        """

        return self.add_waypoint_to_list(request)
    
    def sending_waypoints(self):
        """
        Sends the current list of waypoints to the waypoint list service.

        Waits for the waypoint list service to become available. Once available,
        it sends the current list of waypoints and waits for a response. Logs the
        outcome of the service call.
        """

        while not self.waypoint_list_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waypoint service not available, waiting again...')
        request = Waypoint.Request()
        
        request.waypoints = self.waypoint_list
        future = self.waypoint_list_client.call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec = 1.0)
        try:
            response = future.result()
            self.get_logger().info(f'Waypoints successfully submitted: {response.success}')
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))

def main(args=None):
    rclpy.init(args=args)
    waypoint_manager = WaypointManager()
    rclpy.spin(waypoint_manager)
    rclpy.shutdown()

if __name__ == '__main__':
    main()      
