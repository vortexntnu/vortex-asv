#!/usr/bin/env python3

<<<<<<< HEAD
import sys
=======
>>>>>>> development
import rclpy
from rclpy.node import Node
from vortex_msgs.srv import MissionParameters
from geometry_msgs.msg import Point

class MissionPlannerClient(Node):
    """
    A ROS2 client node for interacting with the MissionPlanner service.
    """
    def __init__(self):
        """
        Initializes the client node, creates a client for the MissionPlanner service,
        and waits for the service to become available.
        """
        super().__init__('mission_planner_client')
        self.client = self.create_client(MissionParameters, 'mission_parameters')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = MissionParameters.Request()

    def send_request(self, obstacles: list[Point], start: Point, goal: Point, origin: Point, height: int, width: int):
        """
        Sends an asynchronous request to the MissionPlanner service with obstacle locations,
        start, and goal positions.

        Args:
            obstacles (list[Point]): The list of obstacle points.
            start (Point): The start point.
            goal (Point): The goal point.
            origin (Point): The origin point of the world.
            height (int): The height of the world.
            width (int): The width of the world.
        """
        self.req.obstacles = obstacles
        self.req.start = start
        self.req.goal = goal
        self.req.origin = origin
        self.req.height = height
        self.req.width = width

        self.future = self.client.call_async(self.req)
        self.get_logger().info('MissionPlanner service has been called')

def main(args=None):
    """
    Main function with example usage of the MissionPlannerClient.

    """
    rclpy.init(args=args)
    mission_planner_client = MissionPlannerClient()
    # Test data
<<<<<<< HEAD
    obstacles = []
    start = Point(x=1.0, y=1.0)
    goal = Point(x=10.0, y=10.0)
    mission_planner_client.send_request(obstacles, start, goal, Point(x=0.0, y=0.0), 15, 15)
=======
    obstacles = [Point(x=5.0, y=5.0)]
    start = Point(x=0.0, y=0.0)
    goal = Point(x=10.0, y=10.0)
    mission_planner_client.send_request(obstacles, start, goal, Point(x=0.0, y=0.0), 30, 30)
>>>>>>> development

    while rclpy.ok():
        rclpy.spin_once(mission_planner_client)
        if mission_planner_client.future.done():
            try:
                response = mission_planner_client.future.result()
                mission_planner_client.get_logger().info(f'Success: {response.success}')
            except Exception as e:
                mission_planner_client.get_logger().info(f'Service call failed {e}')
                break  # Break out of the loop if the service call failed
            else:
                # This else block should be executed if no exceptions were raised
                mission_planner_client.get_logger().info('Successfully created path')
                break  # Ensure to break out of the loop once processing is done
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        