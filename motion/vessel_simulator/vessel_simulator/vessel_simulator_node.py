import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray, Wrench
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler, euler2quat

import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_continuous_are

def ssa(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

class VesselVisualizerNode(Node):
    def __init__(self):
        super().__init__("vessel_visualizer_node")
        # add wrench_input subscriber
        self.wrench_subscriber_ = self.create_subscription(Wrench, "thrust/wrench_input", self.wrench_input_cb, 1)
        # publish state (seapath)
        self.state_publisher_ = self.create_publisher(Odometry, "/sensor/seapath/odometry/ned", 1)

        self.x_init = np.array([10, -20, 40*np.pi/180, 0, 0, 0])
        self.x_ref = np.array([0, 0, self.heading_ref])
        self.dt = 0.01

    def wrench_input_cb(self, msg):
        u = np.array([msg.force.x, msg.force.y, msg.torque.z])
        x_next = self.RK4_integration_step(self, self.x_init, u, self.dt)

    

 

    def state_dot(self, state: np.ndarray, tau_actuation: np.ndarray, V_current: np.ndarray = np.zeros(2)) -> np.ndarray:
        """
        Calculate the derivative of the state using the non-linear kinematics
        """
        heading = state[2]

        J = np.array(
            [[np.cos(heading), -np.sin(heading), 0],
            [np.sin(heading), np.cos(heading), 0],
            [0, 0, 1]]
        )

        A = np.zeros((6,6))

        A[:3,3:] = J
        A[3:, 3:] = - self.M_inv @ self.D

        B = np.zeros((6,3))
        B[3:,:] = self.M_inv

        x_dot = A @ state + B @ tau_actuation
        x_dot[0:2] += V_current # add current drift term at velocity level

        return x_dot
    
    def RK4_integration_step(self, x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
        # integration scheme for simulation, implements the Runge-Kutta 4 integrator
        k1 = self.state_dot(x,         u)
        k2 = self.state_dot(x+dt/2*k1, u)
        k3 = self.state_dot(x+dt/2*k2, u)
        k4 = self.state_dot(x+dt*k3,   u)
        
        x_next = x + dt/6*(k1+2*k2+2*k3+k4)

        return x_next

    def run_ivan_sim(self):


        look_ahead = 5 # Look ahead distance

        T = 69.0        # Total simulation time
        dt = 0.01       # Time step
        num_steps = int(T / dt)  # Number of time steps

        # Initialize arrays to store data
        time = np.linspace(0, T, num_steps+1)
        x_history = np.zeros((num_steps+1, self.A.shape[0]))
        u_history = np.zeros((num_steps+1, self.B.shape[1]))
        x_ref_history = np.zeros((num_steps+1, np.shape(x_ref)[0]))

        cross_track_error_history = np.zeros(num_steps+1)

        # Simulation loop
        x = x_init # Initial state
        for i in range(num_steps+1):
            # x_ref[i] = reference_function_const(i * dt)  # Update the reference at each time step
            # x_ref_history[i, :] = x_ref  # Update the reference at each time step
            
            # generate reference at the look-ahead distance
            p_asv = np.array([x[0], x[1]])
            errors = self.R_Pi_p.T @ (p_asv - self.p0)
            along_track_error = errors[0]
            p_los_world = self.R_Pi_p @ np.array([along_track_error + look_ahead, 0]) + self.p0

            x_ref[:2] = p_los_world # Update the position reference at each time step
            x_ref_history[i, :] = x_ref
            
            u = self.calculate_control_input(x, x_ref, self.K_LQR, self.K_r)  # Calculate control input 'u' at each time step
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
        for j in range(self.B.shape[1]):
            plt.plot(time, u_history[:, j], label=f'Control Input (u_{j+1})')
        plt.xlabel('Time')
        plt.ylabel('Control Input Value')
        plt.legend()

        plt.subplot(3, 1, 3)
        plt.scatter(x_history[:,0], x_history[:,1], label=f'Position')
        plt.plot([self.p0[0], self.p1[0]], [self.p0[1], self.p1[1]], 'r-', label='Path')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.legend()

        plt.tight_layout()
        plt.show()
        plt.figure(1)
        plt.plot(time, cross_track_error_history, label="cross track error")
        plt.axis("equal")
        plt.plot(time, np.zeros_like(time))
        plt.legend()
        plt.show()
        


if __name__ == '__main__':
    rclpy.init()


    rclpy.spin()
    rclpy.shutdown()
