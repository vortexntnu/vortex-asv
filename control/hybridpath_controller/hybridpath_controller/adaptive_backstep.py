import numpy as np
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler
from vortex_msgs.msg import HybridpathReference


class AdaptiveBackstep:
    def __init__(self):
        self.k1 = np.eye(3)
        self.k2 = np.eye(3)
        self.m = np.eye(3)
        self.d = np.eye(3)

    def update_parameters(
        self, k1: np.ndarray, k2: np.ndarray, m: np.ndarray, d: np.ndarray
    ) -> None:
        self.k1 = k1
        self.k2 = k2
        self.m = m
        self.d = d

    def control_law(
        self, state: Odometry, reference: HybridpathReference
    ) -> np.ndarray:
        """Calculates the control input based on the state and reference.

        Args:
            state (Odometry): The current state of the system.
            reference (HybridpathReference): The reference to follow.

        Returns:
            np.ndarray: The control input.
        """
        # Transform the Odometry message to a state vector
        state = self.odom_to_state(state)

        # Extract values from the state and reference
        eta = state[:3]
        # eta[0] = 0.
        nu = state[3:]
        w = reference.w
        v_s = reference.v_s
        v_ss = reference.v_ss
        eta_d = np.array([reference.eta_d.x, reference.eta_d.y, reference.eta_d.theta])
        eta_d_s = np.array(
            [reference.eta_d_s.x, reference.eta_d_s.y, reference.eta_d_s.theta]
        )
        eta_d_ss = np.array(
            [reference.eta_d_ss.x, reference.eta_d_ss.y, reference.eta_d_ss.theta]
        )

        # Get R_transposed and S
        rot_trps = self.rotationmatrix_in_yaw_transpose(eta[2])
        skew = self.skew_symmetric_matrix(nu[2])

        # Define error signals
        eta_error = eta - eta_d
        eta_error[2] = self.ssa(eta_error[2])

        z1 = rot_trps @ eta_error
        alpha1 = -self.k1 @ z1 + rot_trps @ eta_d_s * v_s

        z2 = nu - alpha1

        sigma1 = (
            self.k1 @ (skew @ z1) - self.K_1 @ nu - skew @ (rot_trps @ eta_d_s) * v_s
        )

        ds_alpha1 = (
            self.K_1 @ (rot_trps @ eta_d_s)
            + rot_trps @ eta_d_ss * v_s
            + rot_trps @ eta_d_s * v_ss
        )

        tau = (
            -self.k2 @ z2
            + self.calculate_coriolis_matrix(nu)
            + self.d @ nu
            + self.m @ sigma1
            + self.m @ ds_alpha1 * (v_s + w)
        )

        self.eta_error = eta_error
        self.z1 = z1
        self.alpha1 = alpha1
        self.z2 = z2
        self.ds_alpha1 = ds_alpha1
        self.sigma1 = sigma1

        return tau

    def get_eta_error(self):
        return self.eta_error

    def get_z1(self):
        return self.z1

    def get_alpha1(self):
        return self.alpha1

    def get_z2(self):
        return self.z2

    def get_sigma1(self):
        return self.sigma1

    def get_ds_alpha1(self):
        return self.ds_alpha1

    @staticmethod
    def calculate_coriolis_matrix(nu: np.ndarray) -> np.ndarray:
        """Returns the Coriolis matrix times the velocity vector nu."""
        coriolis = np.array([[0, 0, -82.5], [0, 0, 5.5], [82.5, -5.5, 0]])
        return coriolis @ nu

    @staticmethod
    def rotationmatrix_in_yaw_transpose(psi: float) -> np.ndarray:
        """Returns the transposed rotation matrix in the yaw angle psi."""
        rot = np.array(
            [[np.cos(psi), -np.sin(psi), 0], [np.sin(psi), np.cos(psi), 0], [0, 0, 1]]
        )
        rot_trps = np.transpose(rot)
        return rot_trps

    @staticmethod
    def skew_symmetric_matrix(r: float) -> np.ndarray:
        """Returns the skew symmetric matrix times the angular velocity r."""
        skew = np.array([[0, -r, 0], [r, 0, 0], [0, 0, 0]])
        return skew

    @staticmethod
    def ssa(angle: float) -> float:
        """Maps an angle to the range [-pi, pi]."""
        angle = np.arctan2(np.sin(angle), np.cos(angle))
        return angle

    @staticmethod
    def odom_to_state(msg: Odometry) -> np.ndarray:
        """Converts an Odometry message to a state 3DOF vector.

        Args:
            msg (Odometry): The Odometry message to convert.

        Returns:
            np.ndarray: The state vector.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.w,
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
        ]

        # Convert quaternion to Euler angles
        yaw = quat2euler(orientation_list)[2]

        # yaw = np.deg2rad(yaw)

        u = msg.twist.twist.linear.x
        v = msg.twist.twist.linear.y
        r = msg.twist.twist.angular.z

        state = np.array([x, y, yaw, u, v, r])

        return state
