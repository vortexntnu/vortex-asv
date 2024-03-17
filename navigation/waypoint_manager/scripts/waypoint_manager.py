#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from vortex_msgs.action import LosPathFollowing

from vortex_msgs.srv import Waypoint
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point


class WaypointManager(Node):
    """
    Nodes created:
        WaypointManager
    Subscribes to:
    Publishes to:
    """
    
    def __init__(self):
        """
        Initializes WaypointManager class.

        Initializes node, sets up action client, services, and publishers.
        """
        super().__init__("WaypointManager")

        self.waypoint_list = []

        # Service for getting the list of waypoints and client for sending the list of waypoint
        self.waypoint_service = self.create_service(Waypoint, 'waypoint', self.waypoint_callback)
        self.waypoint_list_client = self.create_client(Waypoint, 'waypoint_list')

        # Service to send waypoints
        # self.send_waypoint_service = self.create_service(Waypoint, 'send_waypoint_service', self.send_waypoint_service_callback)

        # Services
        self.get_logger().info('kommet til services')
        self.add_waypoint_service = self.create_service(Waypoint, 'add_waypoint', self.add_waypoint_callback)
        self.get_logger().info('kommet til etter at man har laget add_waypoint_services')
        self.remove_waypoint_service = self.create_service(Waypoint, 'remove_waypoint', self.remove_waypoint_callback)

        # nav_msgs Path to visualize LOS in Rviz
        self.path_pub = self.create_publisher(Path, 'los_path', 10)
        self.path = Path()
        self.path.header.frame_id = 'world'

    def waypoint_callback(self, request, response):
        """
        Callback function to handle received waypoints.
        """
        # Clear the existing waypoint list
        self.waypoint_list.clear()

        return self.add_waypoint_to_list(request)
    
   
    def add_waypoint_to_list(self, req):
        """
        Adds waypoints to the waypoint list.

        Args:
            req (Waypoint.Request): Request containing the list of waypoints to be added.

        Returns:
            Waypoint.Response: True if waypoints are added successfully.
        """
        for i in range(0, len(req.waypoint), 2):
            x = req.waypoint[i]
            y = req.waypoint[i+1]

            self.waypoint_list.append([x, y])
            self.get_logger().info("Added waypoint to waypoint_list")

            new_pose = PoseStamped()
            new_pose.pose.position = Point(x=x, y=y, z=0.0)
            self.path.poses.append(new_pose)
            self.path_pub.publish(self.path)
            self.get_logger().info("points :" + str(self.waypoint_list))

        response = Waypoint.Response()
        response.success = True
        return response

    
    def remove_waypoint_from_list(self, req):
        """
        Removes a waypoint from the waypoint list.

        Args:
            req (Waypoint.Request): Request containing the waypoint to be removed.
        """

        self.get_logger().info("request: " + str(req.waypoint))
        self.get_logger().info("before remove: " + str(self.waypoint_list))

        # Checks if the waypoint_list is empty
        if not self.waypoint_list:
            self.get_logger().info("Waypoint list is empty")

            response = Waypoint.Response()
            response.success = False
            return response

        # Checks if the waypoint exists in the list, and removes it if it exists
        waypoint_list_len = len(self.waypoint_list)

        if waypoint_list_len == 1 and ([req.waypoint[0], req.waypoint[1]] == self.waypoint_list[0]):
                self.waypoint_list.remove(self.waypoint_list[0])
                self.get_logger().info("remove waypoint from waypoint_list")
        else:
            for i in range(0, waypoint_list_len-1):
                if [req.waypoint[0], req.waypoint[1]] == self.waypoint_list[i]:
                    self.waypoint_list.remove(self.waypoint_list[i])
                    self.get_logger().info("remove waypoint from waypoint_list")
            if waypoint_list_len == len(self.waypoint_list):
                self.get_logger().info("Waypoint not found in waypoint_list")

                response = Waypoint.Response()
                response.success = False
                return response
        
        self.get_logger().info("after remove: " +str(self.waypoint_list))

        # Check if path.poses is empty before popping
        if self.path.poses:
            self.path.poses.reverse()
            self.path.poses.pop()
            self.path.poses.reverse()

            self.path_pub.publish(self.path)

        response = Waypoint.Response()
        response.success = True
        
        return response
    
    def remove_waypoint_callback(self, request, response):
        return self.remove_waypoint_from_list(request)

    def add_waypoint_callback(self, request, response):
        self.get_logger().info(str(request))
        return self.add_waypoint_to_list(request)
    
    def sending_waypoints(self):
        while not self.waypoint_list_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waypoint service not available, waiting again...')
        request = Waypoint.Request()
        self.temp_waypoint_list = []
        for waypoint in self.waypoint_list:
            self.temp_waypoint_list.append(waypoint[0])
            self.temp_waypoint_list.append(waypoint[1])
        request.waypoints = self.temp_waypoint_list
        future = self.waypoint_list_client.call_async(request)

        rclpy.spin_until_future_complete(self, future)
        try:
            response = future.result()
            self.get_logger().info(f'Waypoints successfully submitted: {response.success}')
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))
    
    """
    def remove_reached_waypoints(self, current_position):
       
        Remove waypoints from the list that have been reached by the robot.
        
        if not self.waypoint_list:
            return
        
        # Calculate the distance between the current position and each waypoint
        distance_threshold = 0.1  # Define a threshold for considering a waypoint reached
        remaining_waypoints = []
        for waypoint in self.waypoint_list:
            distance_to_waypoint = ((waypoint[0] - current_position.x) ** 2 + (waypoint[1] - current_position.y) ** 2) ** 0.5
            if distance_to_waypoint > distance_threshold:
                remaining_waypoints.append(waypoint)
        
        # Update the waypoint list
        self.waypoint_list = remaining_waypoints

        # Publish the updated waypoints for visualization
        self.path.poses = []
        for waypoint in self.waypoint_list:
            new_pose = PoseStamped()
            new_pose.pose.position = Point(x=waypoint[0], y=waypoint[1], z=0.0)
            self.path.poses.append(new_pose)
        self.path_pub.publish(self.path)

    def send_waypoint_service_callback(self, request, response, current_position):
        
        Service callback function to send the waypoint list.
        
        self.remove_reached_waypoints(current_position)  # Pass the current position of the robot
        # Populate the response with the waypoint list
        response.waypoints = []
        for waypoint in self.waypoint_list:
            response.waypoints.extend(waypoint)
        
        return response
"""

def main(args=None):
    rclpy.init(args=args)
    waypoint_manager = WaypointManager()
    rclpy.spin(waypoint_manager)
    rclpy.shutdown()

if __name__ == '__main__':
    main()      
