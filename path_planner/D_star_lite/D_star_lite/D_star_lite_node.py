import rclpy
import numpy as np
from rclpy.node import Node
import matplotlib.pyplot as plt
from D_star_lite.dsl import DStarLite

class DStarLiteNode(Node):
    def __init__(self):
        super().__init__('d_star_lite_node')
        
        # Needs to be subscribed to the server to get obstacles and coordinates for the map
        
        # Will publish waypoints to the waypoint manager

        ox, oy = [], [] # Placeholder for obstacle coordinates

        # Create a DStarLite object
        self.dsl = DStarLite(ox, oy)

        self.get_logger().info('D* Lite Node has been initialized')

    
def main(args=None):
    rclpy.init(args=args)
    d_star_lite_node = DStarLiteNode()
    rclpy.spin(d_star_lite_node)
    d_star_lite_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()