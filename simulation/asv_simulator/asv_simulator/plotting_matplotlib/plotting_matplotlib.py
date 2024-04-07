import numpy as np
import time

from rclpy.node import Node
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench
from vortex_msgs.msg import HybridpathReference
from hybridpath_controller.adaptive_backstep import AdaptiveBackstep
# from hybridpath_guidance.hp_guidance_utils import Rot
from transforms3d.euler import euler2quat


class VesselHybridpathSimulatorNode(Node):
    def __init__(self):
        super().__init__("vessel_hybridpath_simulator_node")
        
        self.wrench_subscriber_ = self.create_subscription(Wrench, "thrust/wrench_input", self.wrench_input_cb, 15)
        self.hp_ref_subscriber_ = self.create_subscription(HybridpathReference, "guidance/hybridpath/reference", self.hp_ref_cb, 20)
        self.state_publisher_ = self.create_publisher(Odometry, "/sensor/seapath/odometry/ned", 1)
        
        self.T = 200.0
        self.dt = 0.05

        self.time = np.arange(0, self.T, self.dt)

        self.i = 1 # Step counter
        self.i_ref = 1 # Step counter for guidance ref

        waypoints = np.array([[10, 0],
            [10, 10],
            [0, 20],
            [30, 30],
            [40,0],
            [0,-10],
            [0, 0],
            [10, 0]])


        self.eta_d = np.zeros((3, len(self.time)))
        self.eta = np.zeros((3, len(self.time)))
        self.eta_d[:,0] = np.array([waypoints[0,0], waypoints[0,1], np.pi/2])
        self.eta[:,0] = np.array([waypoints[0,0], waypoints[0,1], np.pi/2])
        self.nu = np.zeros((3, len(self.time)))
        self.tau = np.zeros((3, len(self.time)))

        self.AB = AdaptiveBackstep()

        time.sleep(1)
        odometry_msg = self.state_to_odometrymsg()
        self.state_publisher_.publish(odometry_msg)


        self.get_logger().info("vessel_hybridpath_simulator_node started")
        self.get_logger().info("""\

                                       ._ o o
                                       \_`-)|_
                                    ,""       \ 
                                  ,"  ## |   ಠ ಠ. 
                                ," ##   ,-\__    `.
                              ,"       /     `--._;)
                            ,"     ## /           U
                          ,"   ##    /

                Simulation time: """ + str(self.T) + """ secs. Please wait for sim to finish:)
                    """)

    def wrench_input_cb(self, msg: Wrench):
        if self.i == len(self.time):
            self.plot_simulation()
            return
        if self.i_ref > len(self.time):
            return

        self.tau[:, self.i] = np.array([msg.force.x, msg.force.y, msg.torque.z])

        # Step in nu and eta
        nu_dot = np.linalg.inv(self.AB.M) @ self.tau[:, self.i] - np.linalg.inv(self.AB.M) @ self.AB.D @ self.nu[:, self.i-1]
        self.nu[:, self.i] = self.nu[:, self.i-1] + nu_dot * self.dt
        R, R_trsp = Rot(self.eta[2,self.i-1])
        eta_dot = R @ self.nu[:, self.i]
        self.eta[:, self.i] = self.eta[:, self.i-1] + eta_dot * self.dt

        # SSA
        psi0 = self.eta[2, self.i-1]
        psi1 = self.eta[2, self.i]
        psi_vec = np.array([psi0, psi1])
        psi_vec = np.unwrap(psi_vec, period=2*np.pi)
        self.eta[2, self.i] = psi_vec[1]

        # Publish state
        odometry_msg = self.state_to_odometrymsg()
        self.state_publisher_.publish(odometry_msg)

        self.i += 1


    def hp_ref_cb(self, msg: HybridpathReference):
        if self.i_ref < self.eta_d.shape[1]:
            self.eta_d[:, self.i_ref] = np.array([msg.eta_d.x, msg.eta_d.y, msg.eta_d.theta])
        else:
            self.get_logger().error('Index out of bounds: self.i_ref is too large')
        self.i_ref += 1
    
    def state_to_odometrymsg(self):
        orientation_list_next = euler2quat(0, 0, self.eta[2, self.i])

        odometry_msg = Odometry()
        odometry_msg.pose.pose.position.x = self.eta[0, self.i]
        odometry_msg.pose.pose.position.y = self.eta[1, self.i]
        odometry_msg.pose.pose.position.z = 0.0
        odometry_msg.pose.pose.orientation.w = orientation_list_next[0]
        odometry_msg.pose.pose.orientation.x = orientation_list_next[1]
        odometry_msg.pose.pose.orientation.y = orientation_list_next[2]
        odometry_msg.pose.pose.orientation.z = orientation_list_next[3]
        odometry_msg.twist.twist.linear.x = self.nu[0, self.i]
        odometry_msg.twist.twist.linear.y = self.nu[1, self.i]
        odometry_msg.twist.twist.linear.z = 0.0
        odometry_msg.twist.twist.angular.x = 0.0
        odometry_msg.twist.twist.angular.y = 0.0
        odometry_msg.twist.twist.angular.z = self.nu[2, self.i]
        
        return odometry_msg
    

    def plot_simulation(self):
        # np.savetxt('/home/eta_d_ros.txt', self.eta_d.T)
        # Plotting
        plt.figure()
        plt.plot(self.eta_d[1,:], self.eta_d[0,:], label='Reference path', zorder = 0)
        plt.plot(self.eta[1,:], self.eta[0,:], label='Actual path', zorder = 1)
        for i in range(0, len(self.eta_d[2]), 100):
            plt.quiver(self.eta[1,i], self.eta[0,i], np.sin(self.eta[2,i]), np.cos(self.eta[2,i]), zorder = 2)
        plt.title('Actual path vs reference path')
        plt.xlabel('y [m]')
        plt.ylabel('x [m]')
        plt.gca().set_axisbelow(True)
        plt.grid()
        plt.legend()

        plt.figure()
        plt.plot(self.time, self.eta[0,:], label='Actual x')
        plt.plot(self.time, self.eta_d[0,:], label='Reference x')
        plt.plot(self.time, self.eta[1,:], label='Actual y')
        plt.plot(self.time, self.eta_d[1,:], label='Reference y')
        plt.title('Actual position vs reference position')
        plt.xlabel('Time [s]')
        plt.ylabel('Position [m]')
        plt.gca().set_axisbelow(True)
        plt.grid()
        plt.legend()

        plt.figure()
        plt.plot(self.time, self.eta[2,:], label='Actual heading')
        plt.plot(self.time, self.eta_d[2,:], label='Reference heading')
        plt.title('Actual heading vs reference heading')
        plt.xlabel('Time [s]')
        plt.ylabel('Heading [rad]')
        plt.gca().set_axisbelow(True)
        plt.grid()
        plt.legend()

        plt.figure()
        plt.plot(self.time, self.nu[0,:], label='Surge velocity')
        plt.plot(self.time, self.nu[1,:], label='Sway velocity')
        plt.title('velocity')
        plt.xlabel('Time [s]')
        plt.ylabel('Velocity [m/s]')
        plt.gca().set_axisbelow(True)
        plt.grid()
        plt.legend()

        plt.figure()
        plt.plot(self.time, self.tau[0,:], label='Surge force')
        plt.plot(self.time, self.tau[1,:], label='Sway force')
        plt.title('Force')
        plt.xlabel('Time [s]')
        plt.ylabel('Force [N]')
        plt.gca().set_axisbelow(True)
        plt.grid()
        plt.legend()

        plt.show()