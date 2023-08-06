import numpy as np
import scipy.linalg as la


def ssa(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


class LQRController:

    def __init__(self, M, D, Q, R, actuator_limits=np.inf):
        """
        An LQR controller based on a simplified mass-damper vessel model

        State vector: [x, y, psi, u, v, r, ix, iy]
        """
        self.Q = np.diag(Q)
        self.R = np.diag(R)
        self.setpoint = None
        self.M = M
        self.D = D

        self.yaw_idx = 2
        self.integral_states = np.array([0.0, 0.0])

        self.actuator_limits = actuator_limits

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint

    def control(self, state, dt):

        if self.setpoint is None:
            print("Setpoint not set! Returning zero tau...")
            return np.zeros(3)

        state_with_integral = np.concatenate((state, self.integral_states))

        A, B = self.linearize(state_with_integral, dt)
        K = self.lqr(A, B, self.Q, self.R)

        error = state_with_integral - np.concatenate((self.setpoint, [0, 0]))
        error[self.yaw_idx] = ssa(error[self.yaw_idx])

        tau_body = -K @ error

        if np.any(np.abs(tau_body) > self.actuator_limits):  # Anti-windup
            return np.ravel(tau_body)

        self.integral_states += (state[:2] - self.setpoint[:2]) * dt

        if not np.array_equal(
                np.sign(state[:2] - self.setpoint[:2]),
                np.sign(
                    self.integral_states)):  # Integral reset on sign change
            self.integral_states = np.array([0.0, 0.0])

        return np.ravel(tau_body)

    def linearize(self, state, dt):
        A = np.eye(8)
        A[3:6, 3:6] -= dt * np.linalg.inv(self.M) @ self.D
        A[0:3, 3:6] = dt * np.array([[
            np.cos(state[self.yaw_idx]), -np.sin(state[self.yaw_idx]), 0
        ], [np.sin(state[self.yaw_idx]),
            np.cos(state[self.yaw_idx]), 0], [0, 0, 1]])
        A[6, 0] = dt  # add integral effect on x
        A[7, 1] = dt  # add integral effect on y

        B = np.block([[np.zeros((3, 3))], [dt * np.linalg.inv(self.M)],
                      [np.zeros((2, 3))]])

        return A, B

    def lqr(self, A, B, Q, R):
        X = np.matrix(la.solve_discrete_are(A, B, Q, R))
        K = np.matrix(la.inv(R + B.T @ X @ B) @ (B.T @ X @ A))

        return K
