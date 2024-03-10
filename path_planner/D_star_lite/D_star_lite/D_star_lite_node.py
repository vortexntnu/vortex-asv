#!/usr/bin/env python3

import rclpy
from rclpy.node import Node as rclpy_node
import numpy as np
import matplotlib.pyplot as plt
from D_star_lite.dsl import DStarLite, Node
from vortex_msgs.srv import MissionPlanner, Waypoint

class DStarLiteNode(rclpy_node):
    def __init__(self):
        super().__init__('d_star_lite_node')
        self.obs_srv = self.create_service(MissionPlanner, 'mission_planner', self.d_star_lite_callback)
        self.wp_client = self.create_client(Waypoint, 'waypoint')
        self.get_logger().info('D Star Lite Node has been started')
        self.WP_float = []
        

    def d_star_lite_callback(self, request, response):
        self.get_logger().info('D Star Lite Service has been called')
        ox = request.ox
        oy = request.oy
        sx = request.sx
        sy = request.sy
        gx = request.gx
        gy = request.gy
        dsl = DStarLite(ox, oy)
        dsl.dsl_main(Node(sx, sy), Node(gx, gy))
        path = dsl.compute_current_path()
        WP = np.array(dsl.get_WP()).tolist()
        self.WP_float = [float(coordinate) for pair in WP for coordinate in pair]

        self.send_waypoints_request()

        response.success = True
        return response
    
    def send_waypoints_request(self):
        while not self.wp_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waypoint service not available, waiting again...')
        request = Waypoint.Request()
        request.waypoints = self.WP_float
        future = self.wp_client.call_async(request)

        rclpy.spin_until_future_complete(self, future)
        try:
            response = future.result()
            self.get_logger().info(f'Waypoints successfully submitted: {response.success}')
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = DStarLiteNode()
    rclpy.spin(node)
    # Cleanup and shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()