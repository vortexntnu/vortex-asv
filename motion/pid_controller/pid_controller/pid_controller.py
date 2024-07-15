#!/usr/bin/env python3

import numpy as np

class PID:
    def __init__(self):
        self.error_sum = np.zeros(3)

        self.Kp = np.eye(3)
        self.Ki = np.eye(3)
        self.Kd = np.eye(3)

        self.upper = np.array([40, 40, 40])
        self.lower = np.array([-40, -40, -40])

        # self.tau_max = np.array([100, 100, 100])

    def update_parameters(self, Kp, Ki, Kd):
        self.Kp = np.diag(Kp)
        self.Ki = np.diag(Ki)
        self.Kd = np.diag(Kd)

    def calculate_control_input(self, eta, eta_d, eta_dot, dt):
        error = eta - eta_d
        self.error_sum += error * dt
        self.error_sum = np.clip(self.error_sum, -20, 20)

        p = self.Kp @ error
        i = self.Ki @ self.error_sum
        d = self.Kd @ eta_dot

        self.last_error = error

        tau = -(p + i + d)

        # if tau[0] > self.tau_max[0]:
        #     tau[0] = self.tau_max[0]
        # elif tau[0] < -self.tau_max[0]:
        #     tau[0] = -self.tau_max[0]
        
        # if tau[1] > self.tau_max[1]:
        #     tau[1] = self.tau_max[1]
        # elif tau[1] < -self.tau_max[1]:
        #     tau[1] = -self.tau_max[1]

        # if tau[2] > self.tau_max[2]:
        #     tau[2] = self.tau_max[2]
        # elif tau[2] < -self.tau_max[2]:
        #     tau[2] = -self.tau_max[2]

        return tau
    
    @staticmethod
    def cap_array(arr: np.ndarray, lower: np.ndarray, upper: np.ndarray):
        new_arr = np.zeros(3)
        for i in range(len(arr)):
            if arr[i] > upper[i]:
                new_arr[i] = upper[i]
            elif arr[i] < lower[i]:
                new_arr[i] = lower
            else:
                new_arr[i] = arr[i]
