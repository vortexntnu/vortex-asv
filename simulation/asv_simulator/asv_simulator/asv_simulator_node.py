#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from geometry_msgs.msg import Wrench, PoseStamped
from nav_msgs.msg import Odometry, Path
from vortex_msgs.msg import HybridpathReference

import matplotlib.pyplot as plt
import numpy as np

from asv_simulator.conversions import *
from asv_simulator.simulation import *
from asv_simulator.TF2Broadcaster import TF2Broadcaster

class ASVSimulatorNode(Node):
    def __init__(self):
        super().__init__('asv_simulator_node')
        
        self.declare_parameters(
                namespace='',
                parameters=[
                    ('asv_sim.plot_matplotlib_enabled', True),
                    ('asv_sim.progress_bar_enabled', True),
                    ('asv_sim.T', 200),
                    ('physical.inertia_matrix', [50.0, 50.0, 50.0]),
                    ('physical.damping_matrix_diag', [10.0, 10.0, 5.0]),
                ])
            
        self.plot_matplotlib_enabled = self.get_parameter('asv_sim.plot_matplotlib_enabled').get_parameter_value().bool_value
        self.progress_bar_enabled = self.get_parameter('asv_sim.progress_bar_enabled').get_parameter_value().bool_value
        self.T = self.get_parameter('asv_sim.T').get_parameter_value().integer_value
        D_diag = self.get_parameter('physical.damping_matrix_diag').get_parameter_value().double_array_value
        M = self.get_parameter('physical.inertia_matrix').get_parameter_value().double_array_value

        # Init simulation parameters
        self.D = np.diag(D_diag)
        self.M = np.reshape(M, (3, 3))
        self.M_inv = np.linalg.inv(self.M)

        # Init TF2 broadcaster for frames and tfs
        self.own_vessel_frame_id = "asv_pose"
        self.tf_broadcaster = TF2Broadcaster(self.own_vessel_frame_id)

        # subscribe to thrust input and guidance
        self.wrench_subscriber_ = self.create_subscription(Wrench, "thrust/wrench_input", self.wrench_input_cb, 1)
        self.guidance_subscriber_ = self.create_subscription(HybridpathReference, 'guidance/hybridpath/reference', self.guidance_cb, 1)

        # publish state (seapath)
        self.state_publisher_ = self.create_publisher(Odometry, "/sensor/seapath/odom/ned", 1)
        self.posestamped_publisher_ = self.create_publisher(PoseStamped, "/sensor/seapath/posestamped/ned", 1)
        self.state_path_publisher_ = self.create_publisher(Path, "/sensor/seapath/state_path/ned", 1)
        self.xref_path_publisher_ = self.create_publisher(Path, "/sensor/seapath/xref_path/ned", 1)

        self.yaw_ref_publisher_ = self.create_publisher(Float32, "/sensor/seapath/yaw_ref/ned", 1)
        self.yaw_publisher_ = self.create_publisher(Float32, "/sensor/seapath/yaw/ned", 1)

        # create timer
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Init state and control variables
        self.state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.x_ref = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.tau = np.array([0.0, 0.0, 0.0])

        # Init historical pose (path)
        self.state_path = Path()
        self.xref_path = Path()
        self.tau_history = np.zeros((0, 3))

        realtime_factor = 10.0
        self.dt = timer_period*realtime_factor
        self.num_steps_simulated = 0

        self.N = 5000  #  max Path length
        self.max_steps = int(self.T/self.dt)
        self.sim_finished = False
        
        # Publish initial state
        self.state_publisher_.publish(state_to_odometrymsg(self.state))

        self.get_logger().info("""\

                                       ._ o o
                                       \_`-)|_
                                    ,""       \ 
                                  ,"  ## |   ಠ ಠ. 
                                ," ##   ,-\__    `.
                              ,"       /     `--._;)
                            ,"     ## /           U
                          ,"   ##    /
            """
            + "Starting ASV simulator..." + "\n" 
            + "            Simulation time T = " + str(self.T) + "\n" 
            + "            Matplotlib plotting enabled: " + str(self.plot_matplotlib_enabled))

    def timer_callback(self):
        self.simulate_step()

        if self.progress_bar_enabled:
            self.show_progressbar()

        if self.num_steps_simulated > self.max_steps and self.plot_matplotlib_enabled and not self.sim_finished:
            self.plot_matplotlib()
            self.sim_finished = True

    def show_progressbar(self):
        progress = (self.num_steps_simulated / self.max_steps) * 100
        filled_length = int(progress // 10)
        bar = '[' + '=' * filled_length + ' ' * (10 - filled_length) + ']'
        self.get_logger().info(f"Progress: {bar} {progress:.1f}%")

    def guidance_cb(self, msg):
        if isinstance(msg, HybridpathReference):
            self.x_ref[:3] = np.array([msg.eta_d.x, msg.eta_d.y, msg.eta_d.theta])
            self.yaw_ref_publisher_.publish(Float32(data=self.x_ref[2]))
        else:
            self.get_logger().error(f"Received message of type {type(msg).__name__}, expected HybridpathReference")
        
    def wrench_input_cb(self, msg):
        self.tau = np.array([msg.force.x, msg.force.y, msg.torque.z])
        tau_values = np.array([[msg.force.x, msg.force.y, msg.torque.z]])
        self.tau_history = np.append(self.tau_history, tau_values, axis=0)

    def update_path_element(self, path, new_pose, N):
        if len(path.poses) < N:
            path.poses.append(new_pose)
        else:
            # Shift elements and append new_pose
            path.poses.pop(0)
            path.poses.append(new_pose)

    def simulate_step(self):
        x_next = RK4_integration_step(self.M_inv, self.D, self.state, self.tau, self.dt)
        self.state = x_next
        
        # Pub odom
        odometry_msg = state_to_odometrymsg(x_next)
        self.state_publisher_.publish(odometry_msg)

        # Pub Posestamped
        posestamped_msg = state_to_posestamped(x_next, "world", self.get_clock().now().to_msg())
        self.posestamped_publisher_.publish(posestamped_msg)
        
        self.yaw_publisher_.publish(Float32(data=x_next[2]))
        
        # Pub Paths
        self.state_path.header.frame_id = "world"
        self.state_path.header.stamp = self.get_clock().now().to_msg()
        
        self.update_path_element(self.state_path, posestamped_msg, self.N)
        self.state_path_publisher_.publish(self.state_path)

        xref_msg = state_to_posestamped(self.x_ref, "world", self.get_clock().now().to_msg())
        self.xref_path.header.frame_id = "world"
        self.xref_path.header.stamp = self.get_clock().now().to_msg()
        
        self.update_path_element(self.xref_path, xref_msg, self.N)
        self.xref_path_publisher_.publish(self.xref_path)

        # Pub frame and odom tf
        self.tf_broadcaster.fixed_frame_broadcaster()
        self.tf_broadcaster.handle_pose(x_next)

        # Update current sim step
        self.num_steps_simulated += 1

    def plot_matplotlib(self):
        state_x = [pose.pose.position.x for pose in self.state_path.poses]
        state_y = [pose.pose.position.y for pose in self.state_path.poses]
        ref_x = [pose.pose.position.x for pose in self.xref_path.poses]
        ref_y = [pose.pose.position.y for pose in self.xref_path.poses]

        time = np.linspace(0, self.T, len(state_x))
        tau_time = np.linspace(0, self.T, len(self.tau_history))

        plt.figure()
        plt.plot(state_x, state_y, label='State Path')
        plt.plot(ref_x, ref_y, label='Reference Path')
        plt.title('ASV Path')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.grid()
        plt.legend()

        plt.figure()
        plt.plot(time, state_x, label='Actual x')
        plt.plot(time, ref_x, label='Reference x')
        plt.plot(time, state_y, label='Actual y')
        plt.plot(time, ref_y, label='Reference y')
        plt.title('Actual position vs reference position')
        plt.xlabel('Time [s]')
        plt.ylabel('Position [m]')
        plt.gca().set_axisbelow(True)
        plt.grid()
        plt.legend()

        plt.figure()
        plt.title('Control input values tau')
        plt.subplot(2, 1, 1)
        plt.plot(tau_time, self.tau_history[:, 0], label='Surge force')
        plt.xlabel('Time [s]')
        plt.ylabel('Force [N]')
        plt.grid()
        plt.legend()
        plt.subplot(2, 1, 2)
        plt.plot(tau_time, self.tau_history[:, 1], label='Sway force', color='red')
        plt.xlabel('Time [s]')
        plt.ylabel('Force [N]')
        plt.grid()
        plt.legend()

        plt.show()
        
def main(args=None):
    rclpy.init(args=args)
    node = ASVSimulatorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
