import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64, Float32
from geometry_msgs.msg import Wrench, PoseStamped
from nav_msgs.msg import Odometry, Path

import numpy as np
import time

from vessel_simulator_realtime.conversions import *
from vessel_simulator_realtime.simulation import *
from vessel_simulator_realtime.TF2Broadcaster import TF2Broadcaster

class VesselVisualizerNode(Node):
    def __init__(self):
        super().__init__("vessel_visualizer_node")

        # Init TF2 brpadcaster for frames and tfs
        self.own_vessel_frame_id = "asv_pose"
        self.tf_broadcaster = TF2Broadcaster(self.own_vessel_frame_id)

        # add wrench_input subscriber
        self.wrench_subscriber_ = self.create_subscription(Wrench, "thrust/wrench_input", self.wrench_input_cb, 1)
        self.guidance_subscriber_ = self.create_subscription(Odometry, "controller/lqr/reference", self.guidance_cb, 1)
        
        # publish state (seapath)
        self.state_publisher_ = self.create_publisher(Odometry, "/sensor/seapath/odometry/ned", 1)
        self.posestamped_publisher_ = self.create_publisher(PoseStamped, "/sensor/seapath/posestamped/ned", 1)
        self.state_path_publisher_ = self.create_publisher(Path, "/sensor/seapath/state_path/ned", 1)
        self.xref_path_publisher_ = self.create_publisher(Path, "/sensor/seapath/xref_path/ned", 1)

        self.yaw_ref_publisher_ = self.create_publisher(Float32, "/sensor/seapath/yaw_ref/ned", 1)
        self.yaw_publisher_ = self.create_publisher(Float32, "/sensor/seapath/yaw/ned", 1)

        self.test_publisher = self.create_publisher(Int64, "/test1", 1)

        # create timer
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Init state and control variables
        self.state = np.array([10, -20, 40*np.pi/180, 0, 0, 0])
        self.x_ref = np.array([0, 0, 50*np.pi/180, 0, 0, 0]) # Note to self: make x_ref have 6 states
        self.u = np.array([0, 0, 0])

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

        # Init simulation parameters
        m = 50.0
        self.M = np.diag([m, m, m])
        self.M_inv = np.linalg.inv(self.M)
        self.D = np.diag([10.0, 10.0, 5.0])
        
        realtime_factor = 10.0
        self.dt = timer_period*realtime_factor
        self.num_steps_simulated = 0

        # Publish initial state    
        self.state_publisher_.publish(state_to_odometrymsg(self.state))

    def timer_callback(self):
        self.simulate_step()
        msg = Int64()
        msg.data = self.num_steps_simulated
        self.test_publisher.publish(msg)

    def guidance_cb(self, msg):
        self.x_ref = odometrymsg_to_state(msg)
        self.yaw_ref_publisher_.publish(Float32(data=self.x_ref[2]))

    def wrench_input_cb(self, msg):
        self.u = np.array([msg.force.x, msg.force.y, msg.torque.z])

    # def update_path_element(self, n, new_pose):
    #     if n < len(self.state_path.poses):
    #         self.state_path.poses[n] = new_pose
    #     else:
    #         # Shift elements and append new_pose
    #         self.state_path.poses.pop(0)
    #         self.state_path.poses.append(new_pose)

    def update_path_element(self, path, new_pose, N):
        if len(path.poses) < N:
            path.poses.append(new_pose)
        else:
            # Shift elements and append new_pose
            path.poses.pop(0)
            path.poses.append(new_pose)

    def simulate_step(self):
        # Integrate a step forward using RK4 
        x_next = RK4_integration_step(self.M_inv, self.D, self.state, self.u, self.dt)

        # Update state
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
