import rclpy
from vessel_simulator_realtime.VesselVisualizerNode import VesselVisualizerNode


def main(args=None):
    rclpy.init(args=args)
    node = VesselVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()  

if __name__ == "__main__":
    main()
