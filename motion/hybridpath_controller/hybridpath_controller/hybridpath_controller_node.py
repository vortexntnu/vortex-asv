import rclpy
import numpy as np
from rclpy.node import Node
from hybridpath_controller.adaptive_backstep import AdaptiveBackstep
from geometry_msgs.msg import Wrench
from vortex_msgs.msg import HybridpathReference
# TESTING
# from geometry_msgs.msg import Pose2D

class HybridPathControllerNode(Node):
    def __init__(self):
        super().__init__("hybridpath_controller_node")
        
        self.hpref_subscriber_ = self.create_subscription(HybridpathReference, "guidance/hybridpath/reference", self.hpref_cb, 1)
        self.wrench_publisher_ = self.create_publisher(Wrench, "thrust/wrench_input", 1)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('hybridpath_controller.kappa', 0.5)
            ])

        self.kappa_ = self.get_parameter('hybridpath_controller.kappa').get_parameter_value().double_value

        self.AB_controller_ = AdaptiveBackstep(self.kappa_)

        # TESTING

        # msg = HybridpathReference()
        # # Set the properties of the msg object as needed
        # msg.w_ref = 0.03819000726434428
        # msg.v_ref = 0.1001407579811512
        # msg.v_ref_t = 0.0
        # msg.v_ref_s = 0.0108627157420586

        # msg.eta = Pose2D(x=1.0, y=1.0, theta=0.0)
        # msg.nu = Pose2D(x=0.0, y=0.0, theta=0.0)
        # msg.eta_d = Pose2D(x=10.00035914, y=0.24996664, theta=1.56654648)
        # msg.eta_d_s = Pose2D(x=0.04243858, y=9.98585381, theta=-0.33009297)
        # msg.eta_d_ss = Pose2D(x=3.29165665, y=-1.09721888, theta=-11.90686869)

        # self.hpref_cb(msg)

        self.get_logger().info("hybridpath_controller_node started")

    def hpref_cb(self, msg: HybridpathReference):
        """
        Callback function for the HybridpathReference message. This function calculates the control law and publishes thruster wrench messages.

        Args:
            msg (HybridpathReference): The HybridpathReference message containing the reference values.

        Returns:
            None
        """
        w_ref = msg.w_ref
        v_ref = msg.v_ref
        v_ref_t = msg.v_ref_t
        v_ref_s = msg.v_ref_s

        eta = np.array([msg.eta.x, msg.eta.y, msg.eta.theta])
        nu = np.array([msg.nu.x, msg.nu.y, msg.nu.theta])
        eta_d = np.array([msg.eta_d.x, msg.eta_d.y, msg.eta_d.theta])
        eta_d_s = np.array([msg.eta_d_s.x, msg.eta_d_s.y, msg.eta_d_s.theta])
        eta_d_ss = np.array([msg.eta_d_ss.x, msg.eta_d_ss.y, msg.eta_d_ss.theta])

        tau = self.AB_controller_.control_law(eta, nu, w_ref, v_ref, v_ref_t, v_ref_s, eta_d, eta_d_s, eta_d_ss)

        # TESTING
        # with open('tau.txt', 'w') as file:
        #     file.write(f'tau: {tau}')

        wrench_msg = Wrench()
        wrench_msg.force.x  = float(tau[0])
        wrench_msg.force.y  = float(tau[1])
        wrench_msg.force.z  = 0.0
        wrench_msg.torque.x = 0.0
        wrench_msg.torque.y = 0.0
        wrench_msg.torque.z = float(tau[2])

        self.wrench_publisher_.publish(wrench_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HybridPathControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
