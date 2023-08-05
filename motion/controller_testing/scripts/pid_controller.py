import numpy as np


def ssa(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


class PIDController3DOF:

    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = np.array(Kp)
        self.Ki = np.array(Ki)
        self.Kd = np.array(Kd)
        self.setpoint = np.array(setpoint)

        self.integral_error = np.zeros(3)
        self.previous_error = np.zeros(3)

    def reset(self):
        self.integral_error = np.zeros(3)
        self.previous_error = np.zeros(3)

    def control(self, state, dt):
        x_error = self.setpoint[0] - state[0]
        y_error = self.setpoint[1] - state[1]
        yaw_error = ssa(self.setpoint[2] - state[2])

        error = np.array((x_error, y_error, yaw_error))

        P = np.dot(self.Kp, error)
        self.integral_error += error * dt
        I = self.Ki * self.integral_error
        D = self.Kd * (error - self.previous_error) / dt

        self.previous_error = error

        return P + I + D
