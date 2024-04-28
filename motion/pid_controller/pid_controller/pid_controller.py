#!/usr/bin/env python3

import numpy as np

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.error_sum = np.zeros(3)

        self.Kp = np.diag(Kp)
        self.Ki = np.diag(Ki)
        self.Kd = np.diag(Kd)

        self.tau_max = np.array([100, 100, 100])

    def calculate_control_input(self, eta, eta_d, eta_dot, dt):
        error = eta - eta_d
        self.error_sum += error * dt
        self.error_sum = np.clip(self.error_sum, -20, 20)

        p = self.Kp @ error
        i = 0 #self.Ki @ self.error_sum
        d = self.Kd @ eta_dot

        self.last_error = error

        tau = -(p + i + d)

        if tau[0] > self.tau_max[0]:
            tau[0] = self.tau_max[0]
        elif tau[0] < -self.tau_max[0]:
            tau[0] = -self.tau_max[0]
        
        if tau[1] > self.tau_max[1]:
            tau[1] = self.tau_max[1]
        elif tau[1] < -self.tau_max[1]:
            tau[1] = -self.tau_max[1]

        if tau[2] > self.tau_max[2]:
            tau[2] = self.tau_max[2]
        elif tau[2] < -self.tau_max[2]:
            tau[2] = -self.tau_max[2]

        return tau