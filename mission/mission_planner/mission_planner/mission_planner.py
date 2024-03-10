#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from vortex_msgs.srv import MissionPlanner

class MissionPlannerClient(Node):
    def __init__(self):
        super().__init__('mission_planner_client')
        self.client = self.create_client(MissionPlanner, 'mission_planner')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = MissionPlanner.Request()

    def send_request(self, ox, oy, sx, sy, gx, gy):
        self.req.ox = ox
        self.req.oy = oy
        self.req.sx = sx
        self.req.sy = sy
        self.req.gx = gx
        self.req.gy = gy
        self.future = self.client.call_async(self.req)

def main(args=None):
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
            else:
                mission_planner_client.get_logger().info(f'Successfully created path')
            break
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        