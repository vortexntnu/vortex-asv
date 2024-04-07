#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from asv_simulator.plotting_matplotlib import plot_matplotlib
from asv_simulator.realtime_plotting_foxglove import plot_foxglove

class PlottingNode(Node):
    def __init__(self):
        super().__init__('asv_simulator_node')
        
        self.declare_parameter('plotting_method', 'matplotlib')
        plotting_library = self.get_parameter('plotting_method').value
        
        logger = self.get_logger()

        if plotting_library == 'matplotlib':
            plot_matplotlib(logger)
        elif plotting_library == 'foxglove':
            plot_foxglove(logger)
        else:
            self.get_logger().error("Invalid plotting method choice!")

def main(args=None):
    rclpy.init(args=args)
    node = PlottingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
