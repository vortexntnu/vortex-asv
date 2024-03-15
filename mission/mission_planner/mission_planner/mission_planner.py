#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from vortex_msgs.srv import MissionPlanner

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
        self.client = self.create_client(MissionPlanner, 'mission_planner')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = MissionPlanner.Request()

    def send_request(self, ox: list, oy: list, sx: int, sy: int, gx: int, gy: int):
        """
        Sends an asynchronous request to the MissionPlanner service with obstacle locations,
        start, and goal positions.

        Args:
            ox (list): The x-coordinates of obstacles.
            oy (list): The y-coordinates of obstacles.
            sx (int): The x-coordinate of the start position.
            sy (int): The y-coordinate of the start position.
            gx (int): The x-coordinate of the goal position.
            gy (int): The y-coordinate of the goal position.
        """
        self.req.ox = ox
        self.req.oy = oy
        self.req.sx = sx
        self.req.sy = sy
        self.req.gx = gx
        self.req.gy = gy
        self.future = self.client.call_async(self.req)
        self.get_logger().info('MissionPlanner service has been called')

def main(args=None):
    """
    Main function with example usage of the MissionPlannerClient.

    """
    rclpy.init(args=args)
    mission_planner_client = MissionPlannerClient()
    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)
        oy.append(60)
    for i in range(20, 60):
        ox.append(60)
        oy.append(i)
    for i in range(-10, 60):
        ox.append(i)
        oy.append(60)
    for i in range(20, 61):
        ox.append(-10)
        oy.append(i)
    for i in range(-10, 60):
        ox.append(i)
        oy.append(55)
    for i in range(-10, 46):
        ox.append(i)
        oy.append(45)
    for i in range(35, 45):
        ox.append(45)
        oy.append(i)
    for i in range(-10, 46):
        ox.append(i)
        oy.append(35)
    for i in range(-10, 60):
        ox.append(i)
        oy.append(25)
    for i in range(-10, 60):
        ox.append(i)
        oy.append(20)
    mission_planner_client.send_request(ox, oy, -5, 50, -5, 30)

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

        