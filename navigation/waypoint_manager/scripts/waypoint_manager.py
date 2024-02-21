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
        super().init("WaypointManager")

        self.waypoint_list = []

        ## Action client
        self.action_client = ActionClient(LosPathFollowing, 'guidance/los_action_server')
        while not self.action_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Action server not available, waiting again...')

        # Services
        self.add_waypoint_service = self.create_service(Waypoint, 'add_waypoint', self.add_waypoint_to_list)
        self.remove_waypoint_service = self.create_service(Waypoint, 'remove_waypoint', self.remove_waypoint_from_list)

        # nav_msgs Path to visualize LOS in Rviz
        self.path_pub = self.create_publisher(Path, 'los_path', 10)
        self.path = Path()
        self.path.header.frame_id = 'world'
        
    def add_waypoint_to_list(self, req):
        self.waypoint_list.append(req.waypoint)
        self.get_logger().info("add waypoint to waypoint_list")
        newpose = PoseStamped()
        newpose.pose.position = Point(x=req.waypoint.x, y=req.waypoint.y, z=0)
        self.path.poses.append(newpose)
        self.path_pub.publish(self.path)

        return Waypoint.Response(True)
    
    def remove_waypoint_from_list(self, req):
        self.waypoint_list.remove(req)
        self.get_logger().info("remove waypoint from waypoint_list")
        self.path.poses.reverse()
        self.path.poses.pop()
        self.path.poses.reverse()
        self.path_pub.publish(self.path)
    
    def spin(self):
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
