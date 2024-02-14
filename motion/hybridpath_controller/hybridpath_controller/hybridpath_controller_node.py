import rclpy
import numpy as np
from rclpy.node import Node
import matplotlib.pyplot as plt
from hybridpath_controller.hybridpath import HybridPathGenerator, HybridPathSignals

class HybridPathControllerNode(Node):
    def __init__(self):
        super().__init__("hybridpath_controller_node")
        
        self.get_logger().info("hybridpath_controller_node started")

def calc_ps(s, generator):
    signals = HybridPathSignals(generator.Path, s)
    p_der = signals.pd_der
    ps = np.sqrt(p_der[0][0]**2 + p_der[0][1]**2)
    return ps

def s2pos(s, generator):
    signals = HybridPathSignals(generator.Path, s)
    pos = signals.pd
    return pos


def calculate_headings(init_heading, pos):
    headings = [init_heading]
    x_pos = pos[:,0]
    y_pos = pos[:,1]
    for i in range(len(x_pos) - 1):
        psi_0 = headings[i]
        dx = x_pos[i+1] - x_pos[i]
        dy = y_pos[i+1] - y_pos[i]
        heading = np.arctan2(dy, dx)
        psi_1 = heading
        psi_vec = np.array([psi_0, psi_1])
        psi_vec = np.unwrap(psi_vec, period = 2*np.pi)
        headings.append(psi_vec[1])
        
    return headings

def main(args=None):
    u_des = 1.5
    dt = 0.1
    t_tot = 120
    time = np.arange(0, t_tot, dt)
    s_val = np.zeros(len(time))
    s_dot_val = np.zeros(len(time))
    s = 0 # Initial value

    pos = np.array([[10, 0],
        [10, 10],
        [0, 20],
        [30, 30],
        [40,0],
        [0,-10],
        [0, 0],
        [10, 0]])

    # Simple Pos
    # pos = np.array([[-20, 0],
    #                 [3, 0],
    #                 [10, 5],
    #                 [15, 4],
    #                 [20, 0],
    #                 [35, 0]])

    #s = 0.5  # Path parameter
    lambda_val = 0.6  # Curvature constant
    r = 1  # Differentiability order
    PlotHandle = 1
    generator = HybridPathGenerator(pos, r, lambda_val, PlotHandle)
    # path = generator.Path
    # num_subpaths = path['NumSubpaths']
    

    for i,t in enumerate(time[:-1]):
        ps = calc_ps(s, generator)
        s_dot = u_des/ps
        s += s_dot*dt
        s_val[i+1] = s
        s_dot_val[i] = s_dot

    pos_arr = np.array([s2pos(s, generator) for s in s_val])
    headings = calculate_headings(0, pos_arr)

    eta_ref = np.zeros((3, len(time)))
    x_d = pos_arr[:,0]
    y_d = pos_arr[:,1]
    psi_d = headings
    eta_ref = np.array([x_d, y_d, psi_d])

    # Alternative reference path
    # r = 50
    # omega = 0.01
    # x_d = r*np.cos(omega*time)
    # y_d = r*np.sin(omega*time)
    # psi_d = np.arctan2(np.diff(y_d, prepend=y_d[0]), np.diff(x_d, prepend=x_d[0]))
    # eta_ref = np.array([x_d, y_d, psi_d])

    # Vessel parameters
    M = np.diag([50, 50, 50])
    D = np.diag([10, 10, 5])
    K1 = np.diag([1, 1, 2.5])
    K2 = np.diag([1, 1, 2.5])

    # Initial states
    eta = np.zeros((3, len(eta_ref[0])))
    eta[:,0] = eta_ref[:,0]
    nu = np.zeros((3, len(eta[0])))

    n = len(eta[0])

    def Rot(psi):
        R = np.array([[np.cos(psi), -np.sin(psi), 0],
                    [np.sin(psi), np.cos(psi), 0],
                    [0, 0, 1]])
        R_T = np.transpose(R)
        return R, R_T

    e = np.zeros((3, n))
    # Simulation loop
    for i in range(1, n):
        e[0, i] = eta[0, i-1] - x_d[i]
        e[1, i] = eta[1, i-1] - y_d[i]
        e[2, i] = eta[2, i-1] - psi_d[i]

        R, R_T = Rot(eta[2,i-1])
        e_body = R_T @ e[:,i]

        nu_d = -K1 @ e_body

        tau = M.dot(-K2.dot(e_body) - D.dot(nu[:,i-1] - nu_d))

        # Calculate next simulation steps

        nu_dot = np.linalg.inv(M).dot(tau - D.dot(nu[:,i-1]))

        nu[:,i] = nu[:,i-1] + nu_dot*dt

        eta_dot = R @ nu[:,i]

        eta[:,i] = eta[:,i-1] + eta_dot*dt
        
    # for i in range(1, n):
    #     e = eta_ref[:,i] - eta[:,i-1]
    #     psi = eta[2,i-1]
    #     R, R_T = Rot(psi)
    #     nu_d = -K1 @ R_T @ e

    #     # Control law
    #     tau = M @ (-K2 @ e) - D @ (nu[:,i-1] - nu_d)

    #     # Vessel dynamics
    #     nu_dot = np.linalg.inv(M) @ (tau - D @ nu[:,i-1])
    #     nu[:,i] = nu[:,i-1] + nu_dot*dt
    
    #     # Update position and orientation
    #     eta_dot = R @ nu[:,i]
    #     eta[:,i] = eta[:,i-1] + eta_dot*dt

    plt.figure()
    plt.plot(eta[1,:], eta[0,:], label = 'Vessel trajectory')
    plt.plot(eta_ref[1,:], eta_ref[0,:],'--', label = 'Reference trajectory')
    # Plot heading arrows using quiver
    for i in range(0, len(eta_ref[0])-1, 20):
        plt.quiver(eta[1,i], eta[0,i], np.sin(eta[2,i+1]), np.cos(eta[2,i+1]), scale=0.3, scale_units='xy', angles='xy')
    plt.legend()
    plt.grid()
    plt.xlabel('y [m]')
    plt.ylabel('x [m]')
    plt.axis('equal')

    plt.figure()
    plt.plot(time, eta[2,:], label = 'Heading')
    plt.plot(time, eta_ref[2,:], '--', label = 'Reference heading')
    plt.legend()
    plt.grid()
    plt.xlabel('Time [s]')
    plt.ylabel('Heading [rad]')

    plt.figure()
    plt.plot(time, nu[0,:], label = 'Surge velocity')
    plt.plot(time, nu[1,:], label = 'Sway velocity')

    plt.legend()
    plt.grid()
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [m/s]')

    plt.show()

    rclpy.init(args=args)
    node = HybridPathControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
