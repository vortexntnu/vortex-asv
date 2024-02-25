import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler, euler2quat
from geometry_msgs.msg import Pose2D
from vortex_msgs.msg import HybridpathReference
from hybridpath_guidance.hp_guidance_utils import odometrymsg_to_state, state_to_odometrymsg, Rot
from hybridpath_guidance.hybridpath import HybridPathGenerator, HybridPathSignals

class HybridPathGuidanceNode(Node):
    def __init__(self, waypoints):
        super().__init__("hybridpath_guidance_node")

        self.declare_parameters(
        namespace='',
        parameters=[
            ('hybridpath_guidance.lambda_val', 0.6),
            ('hybridpath_guidance.path_generator_order', 2),
            ('hybridpath_guidance.time_to_max_speed', 10.0)
        ])
        # Subscribers
        self.state_subscriber_ = self.create_subscription(Odometry, "/sensor/seapath/odometry/ned", self.state_cb, 1)

        # Publishers
        self.guidance_publisher_ = self.create_publisher(HybridpathReference, "guidance/Hybridpath/reference", 1)

        # Init parameters
        self.waypoints = waypoints
        self.lambda_val = self.get_parameter('hybridpath_guidance.lambda_val').get_parameter_value().double_value
        self.path_generator_order = self.get_parameter('hybridpath_guidance.path_generator_order').get_parameter_value().integer_value
        self.time_to_max_speed = self.get_parameter('hybridpath_guidance.time_to_max_speed').get_parameter_value().double_value
        
        self.generator = HybridPathGenerator(waypoints, self.path_generator_order, self.lambda_val, 1)
        self.path = self.generator.Path
        self.dt = 0.05

        self.eta_d = np.zeros(3)
        self.eta_d_prev = np.zeros(3)

        self.eta = np.zeros(3)
        self.eta_prev = np.zeros(3)
        self.nu = np.zeros(3)
        self.tau = np.zeros(3)
        self.s = 0
        # self.sig = HybridPathSignals(self.path, self.s)
        self.u_desired = 1
        
        self.get_logger().info("hybridpath_guidance_node started")

    def calculate_desired_states(self, dt):
        sig = HybridPathSignals(self.path, self.s)
        v_ref, v_ref_s = sig.calc_vs(self.u_desired)
        self.s += v_ref * dt
        pd = sig.pd # Reference position
        psi_d = sig.psi # Reference heading
        self.eta_d = np.array([pd[0], pd[1], psi_d])
        psi0 = self.eta_d_prev[2]
        psi1 = self.eta_d[2]
        psi_vec = np.array([psi0, psi1])
        psi_vec = np.unwrap(psi_vec, period=2*np.pi)
        self.eta_d[2] = psi_vec[1]

        # Variables needed for tau
        #w_ref = 0
        v_ref_t = 0 # Constant speed
        eta_d_s = np.array([sig.pd_der[0][0], sig.pd_der[0][1], sig.psi_der])
        eta_d_ss = np.array([sig.pd_der[1][0], sig.pd_der[1][1], sig.psi_dder])
        R, R_trsp = Rot(self.eta_prev[2])
        eta_error = self.eta_prev - self.eta_d
        my = 0.5
        w_ref = my * (eta_d_s @ R @ (R_trsp @ eta_error)) / (np.linalg.norm(eta_d_s)**2)

        return w_ref, v_ref, v_ref_t, v_ref_s, self.eta_d, eta_d_s, eta_d_ss

    def update_path(self, waypoints):
        self.generator = HybridPathGenerator(waypoints, self.path_generator_order, self.lambda_val, 1)
        self.path = self.generator.Path

    def state_cb(self, msg):
        state = odometrymsg_to_state(msg)
        self.eta, self.nu = state[:3], state[3:]
        w_ref, v_ref, v_ref_t, v_ref_s, eta_d, eta_d_s, eta_d_ss = self.calculate_desired_states(self.dt)

        # Send (eta[:,i-1], nu[:,i-1], w, v_ref, v_ref_t, v_ref_s, eta_d[:,i], eta_d_s, eta_d_ss)
        hp_ref_msg = HybridpathReference()
        hp_ref_msg.w_ref = w_ref
        hp_ref_msg.v_ref = v_ref
        hp_ref_msg.v_ref_t = float(v_ref_t)
        hp_ref_msg.v_ref_s = float(v_ref_s)

        hp_ref_msg.eta = Pose2D(x=self.eta[0], y=self.eta[1], theta=self.eta[2])
        hp_ref_msg.nu = Pose2D(x=self.nu[0], y=self.nu[1], theta=self.nu[2])
        hp_ref_msg.eta_d = Pose2D(x=eta_d[0], y=eta_d[1], theta=eta_d[2])
        hp_ref_msg.eta_d_s = Pose2D(x=eta_d_s[0], y=eta_d_s[1], theta=eta_d_s[2])
        hp_ref_msg.eta_d_ss = Pose2D(x=eta_d_ss[0], y=eta_d_ss[1], theta=eta_d_ss[2])

        self.guidance_publisher_.publish(hp_ref_msg)
        
        self.eta_d_prev = self.eta_d
        self.eta_prev = self.eta


def main(args=None):
    waypoints = np.array([[10, 0],
            [10, 10],
            [0, 20],
            [30, 30],
            [40,0],
            [0,-10],
            [0, 0],
            [10, 0]])

    rclpy.init(args=args)
    node = HybridPathGuidanceNode(waypoints)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()