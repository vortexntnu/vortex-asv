#!/usr/bin/python3

import numpy as np

class Vessel3DOF:
    def __init__(self, mass, damping, inertia):
        # Mass and inertia matrix
        self.M = np.diag([mass, mass, inertia])

        # Damping matrix
        self.D = np.diag([damping, damping, damping])

        # State vector [u, v, r, x, y, psi], 
        # where u, v, r are velocities and x, y, psi are positions
        self.state = np.zeros(6)

    def step(self, dt, u):
        # Unpack state vector
        u_vel, v_vel, r_vel, x_pos, y_pos, psi_pos = self.state

        # Body velocities
        nu = np.array([u_vel, v_vel, r_vel])

        # Newton's second law
        nu_dot = np.linalg.inv(self.M) @ (u - self.D @ nu)

        # Position update - simple Euler integration
        psi_dot = r_vel
        x_dot = u_vel * np.cos(psi_pos) - v_vel * np.sin(psi_pos)
        y_dot = u_vel * np.sin(psi_pos) + v_vel * np.cos(psi_pos)
        eta_dot = np.array([x_dot, y_dot, psi_dot])

        # Integrate forward in time
        self.state[:3] += nu_dot * dt
        self.state[3:] += eta_dot * dt

        return self.state.copy()