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
            ('hybridpath_guidance.time_to_max_speed', 10.0),
            ('hybridpath_guidance.dt', 0.05),
        ])

        self.state_subscriber_ = self.create_subscription(Odometry, "/sensor/seapath/odometry/ned", self.state_cb, 1)
        self.guidance_publisher_ = self.create_publisher(HybridpathReference, "guidance/hybridpath/reference", 1)

        # TODO: Add support for WaypointManager, add removal of "completed" waypoints

        self.waypoints_ = waypoints
        self.lambda_val_ = self.get_parameter('hybridpath_guidance.lambda_val').get_parameter_value().double_value
        self.path_generator_order_ = self.get_parameter('hybridpath_guidance.path_generator_order').get_parameter_value().integer_value
        self.time_to_max_speed_ = self.get_parameter('hybridpath_guidance.time_to_max_speed').get_parameter_value().double_value # Not used
        self.dt_ = self.get_parameter('hybridpath_guidance.dt').get_parameter_value().double_value
        
        self.generator_ = HybridPathGenerator(waypoints, self.path_generator_order_, self.lambda_val_, 1)
        self.path_ = self.generator_.Path

        self.eta_d_ = np.zeros(3)
        self.eta_d_prev_ = np.zeros(3)
        self.eta_ = np.zeros(3)
        self.eta_prev_ = np.zeros(3)
        self.nu_ = np.zeros(3)
        self.tau_ = np.zeros(3)
        self.s_ = 0
        self.u_desired_ = 1
        
        self.get_logger().info("hybridpath_guidance_node started")

    def calculate_desired_states(self, dt: float) -> tuple[float, float, float, float, np.ndarray, np.ndarray, np.ndarray]:
        """
        Calculate the desired states for the guidance system

        Args:
            dt (float): Time step

        Returns:
            w_ref, v_ref, v_ref_t, v_ref_s, eta_d, eta_d_s, eta_d_ss
        """
        sig = HybridPathSignals(self.path_, self.s_)
        v_ref, v_ref_s = sig.calc_vs(self.u_desired_)
        self.s_ += v_ref * dt
        pd = sig.pd  # Reference position
        psi_d = sig.psi  # Reference heading
        self.eta_d_ = np.array([pd[0], pd[1], psi_d])
        psi0 = self.eta_d_prev_[2]
        psi1 = self.eta_d_[2]
        psi_vec = np.array([psi0, psi1])
        psi_vec = np.unwrap(psi_vec, period=2 * np.pi)
        self.eta_d_[2] = psi_vec[1]

        # Variables needed for tau
        v_ref_t = 0  # Constant speed
        eta_d_s = np.array([sig.pd_der[0][0], sig.pd_der[0][1], sig.psi_der])
        eta_d_ss = np.array([sig.pd_der[1][0], sig.pd_der[1][1], sig.psi_dder])
        R, R_trsp = Rot(self.eta_prev_[2])
        eta_error = self.eta_prev_ - self.eta_d_
        my = 0.5
        w_ref = my * (eta_d_s @ R @ (R_trsp @ eta_error)) / (np.linalg.norm(eta_d_s) ** 2)

        return w_ref, v_ref, v_ref_t, v_ref_s, self.eta_d_, eta_d_s, eta_d_ss

    def update_path(self, waypoints: np.ndarray) -> None:
        """
        Update the path generator with new waypoints

        Args:
            waypoints (np.ndarray): New waypoints
        """
        self.s_ = 0
        self.generator_ = HybridPathGenerator(waypoints, self.path_generator_order_, self.lambda_val_, 1)
        self.path_ = self.generator_.Path

    def state_cb(self, msg: Odometry) -> None:
        """
        Callback function for the state message. Calculates the desired states and publishes the reference message.

        Args:
            msg (Odometry): The state message.
        """

        state = odometrymsg_to_state(msg)
        self.eta_, self.nu_ = state[:3], state[3:]
        w_ref, v_ref, v_ref_t, v_ref_s, eta_d, eta_d_s, eta_d_ss = self.calculate_desired_states(self.dt_)

        # Write w_ref, v_ref, v_ref_t, v_ref_s, eta_d, eta_d_s, eta_d_ss to file
        with open('hp_ref_msg.txt', 'w') as f:
            f.write(f'w_ref: {w_ref}\n')
            f.write(f'v_ref: {v_ref}\n')
            f.write(f'v_ref_t: {v_ref_t}\n')
            f.write(f'v_ref_s: {v_ref_s}\n')
            f.write(f'eta: {self.eta_}\n')
            f.write(f'nu: {self.nu_}\n')
            f.write(f'eta_d: {eta_d}\n')
            f.write(f'eta_d_s: {eta_d_s}\n')
            f.write(f'eta_d_ss: {eta_d_ss}\n')

        hp_ref_msg = HybridpathReference()
        hp_ref_msg.w_ref = w_ref
        hp_ref_msg.v_ref = v_ref
        hp_ref_msg.v_ref_t = float(v_ref_t)
        hp_ref_msg.v_ref_s = float(v_ref_s)

        hp_ref_msg.eta = Pose2D(x=self.eta_[0], y=self.eta_[1], theta=self.eta_[2])
        hp_ref_msg.nu = Pose2D(x=self.nu_[0], y=self.nu_[1], theta=self.nu_[2])
        hp_ref_msg.eta_d = Pose2D(x=eta_d[0], y=eta_d[1], theta=eta_d[2])
        hp_ref_msg.eta_d_s = Pose2D(x=eta_d_s[0], y=eta_d_s[1], theta=eta_d_s[2])
        hp_ref_msg.eta_d_ss = Pose2D(x=eta_d_ss[0], y=eta_d_ss[1], theta=eta_d_ss[2])

        self.guidance_publisher_.publish(hp_ref_msg)

        self.eta_d_prev_ = self.eta_d_
        self.eta_prev_ = self.eta_


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