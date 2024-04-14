#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from asv_simulator.plotting_matplotlib.plotting_matplotlib import VesselHybridpathSimulatorNode

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64, Float32
from geometry_msgs.msg import Wrench, PoseStamped
from nav_msgs.msg import Odometry, Path
from vortex_msgs.msg import HybridpathReference

import numpy as np
import time

from asv_simulator.conversions import *
from asv_simulator.simulation import *
from asv_simulator.TF2Broadcaster import TF2Broadcaster

class ASVSimulatorNode(Node):
    def __init__(self):
        super().__init__('asv_simulator_node')
        
        self.declare_parameter('plotting_method', 'foxglove')
        plotting_method = self.get_parameter('plotting_method').value
        
        logger = self.get_logger()

        if plotting_method == 'matplotlib':
            logger.info("Starting matplotlib Plotting")
            # simulation_node = VesselHybridpathSimulatorNode()
            # rclpy.spin(simulation_node)
        elif plotting_method == 'foxglove':
            logger.info("Starting Foxglove Realtime Plotting")

            # Init TF2 brpadcaster for frames and tfs
            self.own_vessel_frame_id = "asv_pose"
            self.tf_broadcaster = TF2Broadcaster(self.own_vessel_frame_id)

            # add wrench_input subscriber
            self.wrench_subscriber_ = self.create_subscription(Wrench, "thrust/wrench_input", self.wrench_input_cb, 1)
            self.guidance_subscriber_ = self.create_subscription(HybridpathReference, 'guidance/hybridpath/reference', self.guidance_cb, 1)
            
            # publish state (seapath)
            self.state_publisher_ = self.create_publisher(Odometry, "/sensor/seapath/odometry/ned", 1)
            self.posestamped_publisher_ = self.create_publisher(PoseStamped, "/sensor/seapath/posestamped/ned", 1)
            self.state_path_publisher_ = self.create_publisher(Path, "/sensor/seapath/state_path/ned", 1)
            self.xref_path_publisher_ = self.create_publisher(Path, "/sensor/seapath/xref_path/ned", 1)

            self.yaw_ref_publisher_ = self.create_publisher(Float32, "/sensor/seapath/yaw_ref/ned", 1)
            self.yaw_publisher_ = self.create_publisher(Float32, "/sensor/seapath/yaw/ned", 1)

            # self.test_publisher = self.create_publisher(Int64, "/test1", 1)

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

            self.N = 5000  # Example length
            # for _ in range(N):
            #     pose = PoseStamped()
            #     pose.header.frame_id = "world"
            #     pose.header.stamp = self.get_clock().now().to_msg()
            #     # Initialize pose here if needed
            #     self.state_path.poses.append(pose)
            #     self.xref_path.poses.append(pose)

            self.declare_parameters(
                namespace='',
                parameters=[
                    ('physical.inertia_matrix', [50.0, 50.0, 50.0]),
                    ('physical.damping_matrix_diag', [10.0, 10.0, 5.0]),
                ])
            
            D_diag = self.get_parameter('physical.damping_matrix_diag').get_parameter_value().double_array_value
            M = self.get_parameter('physical.inertia_matrix').get_parameter_value().double_array_value

            # Init simulation parameters
            self.D = np.diag(D_diag)
            self.M = np.reshape(M, (3, 3))
            self.M_inv = np.linalg.inv(self.M)
            
            realtime_factor = 10.0
            self.dt = timer_period*realtime_factor
            self.num_steps_simulated = 0

            # Publish initial state    
            self.state_publisher_.publish(state_to_odometrymsg(self.state))

        else:
            self.get_logger().error("Invalid plotting method choice!")

    def timer_callback(self):
        self.simulate_step()
        msg = Int64()
        msg.data = self.num_steps_simulated
        # self.test_publisher.publish(msg)

    def guidance_cb(self, msg: HybridpathReference):
        # self.get_logger().info(str([msg.eta_d.x, msg.eta_d.y, msg.eta_d.theta]))
        self.x_ref[:3] = np.array([msg.eta_d.x, msg.eta_d.y, msg.eta_d.theta])
        self.yaw_ref_publisher_.publish(Float32(data=self.x_ref[2]))

    def wrench_input_cb(self, msg):
        self.tau = np.array([msg.force.x, msg.force.y, msg.torque.z])

    def update_path_element(self, path, new_pose, N):
        if len(path.poses) < N:
            path.poses.append(new_pose)
        else:
            # Shift elements and append new_pose
            path.poses.pop(0)
            path.poses.append(new_pose)

    def simulate_step(self):
        # Integrate a step forward using RK4 
        x_next = RK4_integration_step(self.M_inv, self.D, self.state, self.tau, self.dt)

        # Update state
        self.state = x_next
        
        # Pub odom
        odometry_msg = state_to_odometrymsg(x_next)
        self.state_publisher_.publish(odometry_msg)

        # Pub Posestamped
        posestamped_msg = state_to_posestamped(x_next, "world", self.get_clock().now().to_msg())
        self.posestamped_publisher_.publish(posestamped_msg)
        
        self.get_logger().info("x_next[2]: " + str(x_next[2]))
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

def main(args=None):
    rclpy.init(args=args)
    node = ASVSimulatorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
