import numpy as np


def ssa(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def RT(yaw):
    return np.array([[np.cos(yaw), np.sin(yaw), 0],
                     [-np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])


def saturation(control, min_value, max_value):
    return np.clip(control, min_value, max_value)


class MIMO3DOFNonlinearPID:

    def __init__(self,
                 Kp,
                 Ki,
                 Kd,
                 setpoint,
                 min_output=-np.inf,
                 max_output=np.inf):
        self.Kp = np.diag(Kp)
        self.Ki = np.diag(Ki)
        self.Kd = np.diag(Kd)
        self.setpoint = np.array(setpoint)
        self.min_output = min_output
        self.max_output = max_output

        self.integral_error = np.zeros(3)
        self.previous_error = np.zeros(3)
        self.output = np.zeros(3)

    def reset(self):
        self.integral_error = np.zeros(3)
        self.previous_error = np.zeros(3)

    def control(self, state, dt):
        eta = state[3:]
        eta_error = eta - self.setpoint
        eta_error[2] = ssa(eta_error[2])

        # calculate eta_dot
        eta_dot = (eta_error - self.previous_error) / dt

        self.output = -self.Kp @ eta_error - self.Kd @ eta_dot - self.Ki @ self.integral_error
        self.output = saturation(self.output, self.min_output, self.max_output)
        self.previous_error = eta_error

        return RT(eta[2]) @ self.output
