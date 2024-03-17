#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
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

        # Services
        self.get_logger().info('kommet til services')
        self.add_waypoint_service = self.create_service(Waypoint, 'add_waypoint', self.add_waypoint_callback)
        self.get_logger().info('kommet til etter at man har laget add_waypoint_services')

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

def main(args=None):
    rclpy.init(args=args)
    waypoint_manager = WaypointManager()
    rclpy.spin(waypoint_manager)
    rclpy.shutdown()

if __name__ == '__main__':
    main()      
