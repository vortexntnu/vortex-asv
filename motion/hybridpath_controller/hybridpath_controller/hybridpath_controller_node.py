import rclpy
from rclpy.node import Node
from hybridpath_controller.adaptive_backstep import AdaptiveBackstep
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from vortex_msgs.msg import HybridpathReference
from rclpy.qos import QoSProfile, qos_profile_sensor_data, QoSHistoryPolicy, QoSReliabilityPolicy


qos_profile = QoSProfile(depth=1, history=qos_profile_sensor_data.history, 
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)


class HybridPathControllerNode(Node):
    def __init__(self):
        super().__init__("hybridpath_controller_node")
        
        self.state_subscriber_ = self.state_subscriber_ = self.create_subscription(Odometry, "/sensor/seapath/odom/ned", self.state_callback, qos_profile=qos_profile)
        self.hpref_subscriber_ = self.create_subscription(HybridpathReference, "guidance/hybridpath/reference", self.reference_callback, 1)
        self.wrench_publisher_ = self.create_publisher(Wrench, "thrust/wrench_input", 1)

        self.AB_controller_ = AdaptiveBackstep()

        controller_period = 0.1
        self.controller_timer_ = self.create_timer(controller_period, self.controller_callback)

        self.get_logger().info("hybridpath_controller_node started")

    def state_callback(self, msg: Odometry):
        """
        Callback function for the Odometry message. This function saves the state message.
        """
        self.state_odom = msg

    def reference_callback(self, msg: HybridpathReference):
        self.reference = msg

    def controller_callback(self):
        """
        Callback function for the controller timer. This function calculates the control input and publishes the control input.
        """
        if hasattr(self, 'state_odom') and hasattr(self, 'reference'):
            control_input = self.AB_controller_.control_law(self.state_odom, self.reference)
            wrench_msg = Wrench()
            wrench_msg.force.x = control_input[0]
            wrench_msg.force.y = control_input[1]
            wrench_msg.torque.z = control_input[2]
            self.wrench_publisher_.publish(wrench_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HybridPathControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
