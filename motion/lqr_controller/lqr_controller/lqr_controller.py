import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_continuous_are
from asv_model import ASV

def ssa(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def calculate_control_input(x, x_ref, K_LQR, K_r):
    u = -K_LQR @ x + K_r @ x_ref
    return u

class LQRController:

    def __init__(self, m, D, Q, R):
        self.heading_ref = 50*np.pi/180 # magic init number!!!

        self.M = np.diag([m, m, m])
        self.M_inv = np.linalg.inv(self.M)
        self.D = np.diag(D)

        self.asv = ASV(self.M, self.D)
        self.A, self.B = self.asv.linearize_model(self.heading_ref)
        C = np.zeros((3,6))
        C[:3, :3] = np.eye(3)

        self.Q = np.diag(Q)
        self.R = np.diag(R)
        self.P = solve_continuous_are(self.A(), self.B(), self.Q, self.R)
        self.K_LQR = np.dot(np.dot(np.linalg.inv(self.R), self.B.T), self.P)
        self.K_r = np.linalg.inv(C@np.linalg.inv(self.B @ self.K_LQR - self.A) @ self.B)

    def run_ivan_sim(self):
        x_init = np.array([-10, -10, 40*np.pi/180, 0, 0, 0])
        x_ref = np.array([0, 0, self.heading_ref])

        T = 69.0        # Total simulation time
        dt = 0.01       # Time step
        num_steps = int(T / dt)  # Number of time steps

        # Initialize arrays to store data
        time = np.linspace(0, T, num_steps+1)
        x_history = np.zeros((num_steps+1, self.A.shape[0]))
        u_history = np.zeros((num_steps+1, self.B.shape[1]))
        x_ref_history = np.zeros((num_steps+1, np.shape(x_ref)[0]))

        # Simulation loop
        x = x_init # Initial state
        for i in range(num_steps+1):
            #x_ref[i] = reference_function_const(i * dt)  # Update the reference at each time step
            x_ref_history[i, :] = x_ref  # Update the reference at each time step
            
            u = calculate_control_input(x, x_ref, self.K_LQR, self.K_r)  # Calculate control input 'u' at each time step
            x = self.asv.RK4_integration_step(x, u, dt)

            x_history[i] = x  # Store state history
            u_history[i] = u  # Store control input history

        # Plot results
        plt.figure(figsize=(12, 8))
        plt.subplot(3, 1, 1)
        for j in range(3):
            plt.plot(time, x_history[:, j], label=f'State (x_{j+1})')
            plt.plot(time, x_ref_history[:, j], linestyle='--', label=f'Reference (x_ref_{j+1})')
        plt.xlabel('Time')
        plt.ylabel('State Value')
        plt.legend()

        plt.subplot(3, 1, 2)
        for j in range(B.shape[1]):
            plt.plot(time, u_history[:, j], label=f'Control Input (u_{j+1})')
        plt.xlabel('Time')
        plt.ylabel('Control Input Value')
        plt.legend()

        plt.subplot(3, 1, 3)
        plt.scatter(x_history[:,0], x_history[:,1], label=f'Position')
        plt.xlabel('Time')
        plt.ylabel('Control Input Value')
        plt.legend()

        plt.tight_layout()
        plt.show()



   