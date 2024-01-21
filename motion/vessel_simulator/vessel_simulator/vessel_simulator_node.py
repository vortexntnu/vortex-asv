import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from transforms3d.euler import euler2quat, quat2euler

import numpy as np
import matplotlib.pyplot as plt
import time

def ssa(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

class VesselVisualizerNode(Node):
    def __init__(self):
        super().__init__("vessel_visualizer_node")
        # add wrench_input subscriber
        self.wrench_subscriber_ = self.create_subscription(Wrench, "thrust/wrench_input", self.wrench_input_cb, 1)
        self.guidance_subscriber_ = self.create_subscription(Odometry, "controller/lqr/reference", self.guidance_cb, 1)
        
        # publish state (seapath)
        self.state_publisher_ = self.create_publisher(Odometry, "/sensor/seapath/odometry/ned", 1)
        
        self.state = np.array([10, -20, 40*np.pi/180, 0, 0, 0])
        self.x_ref = np.array([0, 0, 50*np.pi/180]) # Note to self: make x_ref have 6 states

        self.num_steps_simulated = 0

        m = 50.0
        self.M = np.diag([m, m, m])
        self.M_inv = np.linalg.inv(self.M)
        self.D = np.diag([10.0, 10.0, 5.0])
        
        self.T = 20.0
        self.dt = 0.01
        self.num_steps = int(self.T / self.dt)

        self.time = np.linspace(0, self.T, self.num_steps+1)
        self.x_history = np.zeros((self.num_steps+1, 6)) #6 rows in A
        self.x_ref_history = np.zeros((self.num_steps+1, 3)) # defined as length 3 for now
        self.u_history = np.zeros((self.num_steps+1, 3)) #3 columns in B

        self.get_logger().info("vessel_visualizer_node started")
        self.get_logger().info("len(self.x_history): " + str(len(self.x_history)) + "self.num_steps" + str(self.num_steps))

        # Send x_init
        self.get_logger().info("Simulation starting in 1 second...")
        time.sleep(1)

        self.get_logger().info("""\

                                       ._ o o
                                       \_`-)|_
                                    ,""       \ 
                                  ,"  ## |   ಠ ಠ. 
                                ," ##   ,-\__    `.
                              ,"       /     `--._;)
                            ,"     ## /           U
                          ,"   ##    /

                Waiting for simulation to finish...""" + str(self.T) + """ secs approximated waiting time :)
                    """)
        self.state_publisher_.publish(self.state_to_odometrymsg(self.state))

    def odometrymsg_to_state(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z
        ]

        # Convert quaternion to Euler angles, extract yaw
        yaw = quat2euler(orientation_list)[2]

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vyaw = msg.twist.twist.angular.z

        state = np.array([x, y, yaw, vx, vy, vyaw])
        return state
    
    def guidance_cb(self, msg):
        self.x_ref = self.odometrymsg_to_state(msg)[:3]
    
    def state_to_odometrymsg(self, state):
        orientation_list_next = euler2quat(0, 0, state[2])
        
        odometry_msg = Odometry()
        odometry_msg.pose.pose.position.x = state[0]
        odometry_msg.pose.pose.position.y = state[1]
        odometry_msg.pose.pose.position.z = 0.0
        odometry_msg.pose.pose.orientation.w = orientation_list_next[0]
        odometry_msg.pose.pose.orientation.x = orientation_list_next[1]
        odometry_msg.pose.pose.orientation.y = orientation_list_next[2]
        odometry_msg.pose.pose.orientation.z = orientation_list_next[3]
        
        return odometry_msg

    def wrench_input_cb(self, msg):
        u = np.array([msg.force.x, msg.force.y, msg.torque.z])
        x_next = self.RK4_integration_step(self.state, u, self.dt)
        self.x_history[self.num_steps_simulated] = x_next
        self.x_ref_history[self.num_steps_simulated, : ] = self.x_ref
        self.u_history[self.num_steps_simulated] = u
        print(f"self.x_ref_history[{self.num_steps_simulated}]: {self.x_ref_history[self.num_steps_simulated]}")

        if (self.num_steps_simulated >= self.num_steps):
            self.plot_history()
            return

        odometry_msg = self.state_to_odometrymsg(x_next)

        # Update state
        self.state = x_next

        self.state_publisher_.publish(odometry_msg)
        self.num_steps_simulated += 1

        # self.get_logger().info(str(self.cb_count) + " lol xD") # heh DO NOT REMOVE

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

    def plot_history(self):
        # Plot results
        # self.get_logger().info(str(self.x_history[:,0]))
        # print(self.x_history[:,0])
        # print(self.x_history[:,1])

        # Save the array to a text file
        # file_path = '/home/martin/x_ref_hist.txt' #  martin stays
        # np.savetxt(file_path, self.x_ref_history, fmt='%4f', delimiter=',')

        plt.figure(figsize=(12, 8))
        plt.subplot(3, 1, 1)
        for j in range(3):
            plt.plot(self.time, self.x_history[:, j], label=f'State (x_{j+1})')
            plt.plot(self.time, self.x_ref_history[:, j], linestyle='--', label=f'Reference (x_ref_{j+1})')
        plt.xlabel('Time')
        plt.ylabel('State Value')
        plt.legend()

        plt.subplot(3, 1, 2)
        for j in range(3): #3 columns in B
            plt.plot(self.time, self.u_history[:, j], label=f'Control Input (u_{j+1})')
        plt.xlabel('Time')
        plt.ylabel('Control Input Value')
        plt.legend()

        plt.subplot(3, 1, 3)
        plt.scatter(self.x_history[:,0], self.x_history[:,1], label=f'Position')

        p0 = [0.0, 0.0]
        p1 = [20.0, 20.0]
        plt.plot([p0[0], p1[0]], [p0[1], p1[1]], 'r-', label='Path')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.legend()

        plt.tight_layout()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = VesselVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()  

if __name__ == "__main__":
    main()
