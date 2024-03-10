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

        ## Action client
        self._action_client = ActionClient(self, LosPathFollowing, 'LosPathFollowing')
        #while not self._action_client.wait_for_server(timeout_sec=1.0):
        #   self.get_logger().info('Action server not available, waiting again...')
           
        #Action server 

        # Services
        self.get_logger().info('kommet til services')
        self.add_waypoint_service = self.create_service(Waypoint, 'add_waypoint', self.add_waypoint_callback)
        self.get_logger().info('kommet til etter at man har laget add_waypoint_services')
        self.remove_waypoint_service = self.create_service(Waypoint, 'remove_waypoint', self.remove_waypoint_callback)

        # nav_msgs Path to visualize LOS in Rviz
        self.path_pub = self.create_publisher(Path, 'los_path', 10)
        self.path = Path()
        self.path.header.frame_id = 'world'


   
    def add_waypoint_to_list(self, req):
        """
        Adds waypoints to the waypoint list.

        Args:
            req (Waypoint.Request): Request containing the list of waypoints to be added.

        Returns:
            Waypoint.Response: True if waypoints are added successfully.
        """
        waypoints = req.waypoints

        for waypoint in waypoints:
            x = waypoint[0]
            y = waypoint[1]

            self.waypoint_list.append(waypoint)
            self.get_logger().info(f"Added waypoint {waypoint} to waypoint_list")

            new_pose = PoseStamped()
            new_pose.pose.position = Point(x=x, y=y, z=0.0)
            self.path.poses.append(new_pose)

        self.path_pub.publish(self.path)

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

        # Checks if the waypoint exists in the list
        if req.waypoint in self.waypoint_list:
            self.waypoint_list.remove(req.waypoint)
            self.get_logger().info("remove waypoint from waypoint_list")
        else:
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
    
    def add_waypoint_callback(self, request, response):
        return self.add_waypoint_to_list(request)

    def remove_waypoint_callback(self, request, response):
        return self.remove_waypoint_from_list(request)
    
    def spin(self):
        """
        Spins the node to process added waypoints and send them to the action server.
        """
        index_waypoint_k = 0
        while rclpy.ok():
            if len(self.waypoint_list) >= 2 and index_waypoint_k < len(self.waypoint_list) - 1:
                goal = LosPathFollowing.Goal()
                self.get_logger().info("define goal to send to los_guidance_node")

                goal.waypoints[0].x = self.waypoint_list[self.index_waypoint_k][0]
                goal.waypoints[0].y = self.waypoint_list[self.index_waypoint_k][1]
                goal.waypoints[1].x = self.waypoint_list[self.index_waypoint_k + 1][0]
                goal.waypoints[1].y = self.waypoint_list[self.index_waypoint_k + 1][1]
                self.get_logger().info("add waypoints to goal")
                self.get_logger().info(
                        "current points are: \n("
                        + str(goal.waypoints[0].x)
                        + ","
                        + str(goal.waypoints[0].y)
                        + ")\n"
                        + "("
                        + str(goal.waypoints[1].x)
                        + ","
                        + str(goal.waypoints[1].y)
                        + ")"
                )

                self.action_client.send_goal_async(goal)
                self.get_logger().info("send goal to los_guidance_node")
                self.action_client.wait_for_result_async()
                self.get_logger().info("receive result from los_guidance_node")
                self.index_waypoint_k += 1
            elif len(self.waypoint_list) >= 2 and index_waypoint_k >= len(self.waypoint_list) - 1:
                self.waypoint_list.clear()
                self.path.poses.clear()
                self.path_pub.publish(self.path)
                self.get_logger().info("clear waypoint_list")


def main(args=None):
    rclpy.init(args=args)
    waypoint_manager = WaypointManager()
    rclpy.spin(waypoint_manager)
    waypoint_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()      
