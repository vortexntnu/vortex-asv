import numpy as np
from scipy.linalg import solve_continuous_are

class LQRController:
    def __init__(self, M: float, D: list[float], Q: list[float], R: list[float], heading_ref: float = 0.0):
        self.M = M
        self.M_inv = np.linalg.inv(self.M)
        self.D = D  

        self.A = np.zeros((6,6))
        self.B  = np.zeros((6,3))
        self.C = np.zeros((3,6))
        self.C[:3, :3] = np.eye(3)
        
        self.Q = np.diag(Q)
        print(f"Q: {self.Q}")
        self.R = np.diag(R)

        self.linearize_model(heading_ref)

    def calculate_control_input(self, x, x_ref):
        tau = -self.K_LQR @ x + self.K_r @ x_ref
        return tau
    
    def linearize_model(self, heading: float) -> tuple[np.ndarray, np.ndarray]:
        R = np.array(
            [[np.cos(heading), -np.sin(heading), 0],
            [np.sin(heading), np.cos(heading), 0],
            [0, 0, 1]]
        )

        self.A[:3,3:] = R
        self.A[3:, 3:] = - self.M_inv @ self.D

        self.B[3:,:] = self.M_inv

        self.P = solve_continuous_are(self.A, self.B, self.Q, self.R)
        self.K_LQR = np.dot(np.dot(np.linalg.inv(self.R), self.B.T), self.P)
        self.K_r = np.linalg.inv(self.C@np.linalg.inv(self.B @ self.K_LQR - self.A) @ self.B)