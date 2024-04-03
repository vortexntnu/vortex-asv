import numpy as np
from scipy.linalg import solve_continuous_are
from lqr_controller.asv_model import ASV

def ssa(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

class LQRController:

    def __init__(self, m: float, D: list[float], Q: list[float], R: list[float]):
        self.heading_ref = 50*np.pi/180

        self.M = np.diag([m, m, m])
        self.M_inv = np.linalg.inv(self.M)
        self.D = np.diag(D)

        self.asv = ASV(self.M, self.D)

        self.linearize_model(self.heading_ref)

        C = np.zeros((3,6))
        C[:3, :3] = np.eye(3)

        self.Q = np.diag(Q)
        self.R = np.diag(R)
        self.P = solve_continuous_are(self.A, self.B, self.Q, self.R)
        self.K_LQR = np.dot(np.dot(np.linalg.inv(self.R), self.B.T), self.P)
        self.K_r = np.linalg.inv(C@np.linalg.inv(self.B @ self.K_LQR - self.A) @ self.B)

    def linearize_model(self, heading: float):
        self.A, self.B = self.asv.linearize_model(heading)

    def calculate_control_input(self, x, x_ref, K_LQR, K_r):
        # print("x:", x)
        # print("x_ref:", x)
        # print("K_LQR:", K_LQR)
        # print("K_r:", K_r)
        u = -K_LQR @ x + K_r @ x_ref
        return u


   