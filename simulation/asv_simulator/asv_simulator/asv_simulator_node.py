#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from asv_simulator.plotting_matplotlib import VesselHybridpathSimulatorNode
from asv_simulator.realtime_plotting_foxglove import plot_foxglove

class ASVSimulatorNode(Node):
    def __init__(self):
        super().__init__('asv_simulator_node')
        
        self.declare_parameter('plotting_method', 'foxglove')
        plotting_method = self.get_parameter('plotting_method').value
        
        logger = self.get_logger()

        if plotting_method == 'matplotlib':
            simulation_node = VesselHybridpathSimulatorNode()
            rclpy.spin(simulation_node)
        elif plotting_method == 'foxglove':
            plot_foxglove(logger)
        else:
            self.get_logger().error("Invalid plotting method choice!")

def main(args=None):
    rclpy.init(args=args)
    node = ASVSimulatorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
