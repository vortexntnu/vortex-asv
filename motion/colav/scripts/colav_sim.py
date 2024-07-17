#!/usr/bin/env python3

import colav_controller
import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry
from vortex_msgs.msg import GuidanceData
from transforms3d.euler import quat2euler, euler2quat


class Simulator(Node):
    def __init__(self):
        super().__init__("simulator")

        self.vessel_pub = self.create_publisher(Odometry, "/pose_gt", 10)
        self.obstacle_pub = self.create_publisher(Odometry, "/obstacle", 10)
        self.colav_sub = self.create_subscription(GuidanceData, "/guidance/collision_avoidance", self.colav_callback, 10)

        self.pos_x = 0
        self.pos_y = 0
        self.speed = 0.5
        self.obstacle_pos_x = 5
        self.obstacle_pos_y = 5
        self.obstacle_speed = 0.1
        self.obstacle_heading = 5*np.pi/4
        self.goal_pos_x = 8
        self.goal_pos_y = 8

        self.dt = 0.1

        self.first_iteration = False

        obs_timer = 0.1
        self.create_timer(obs_timer, self.obstacle_callback)

        self.get_logger().info("Simulator node started")

    def step(self, speed, heading):
        speed_x = speed*np.cos(heading)
        speed_y = speed*np.sin(heading)
        self.pos_x += speed_x*self.dt
        self.pos_y += speed_y*self.dt
        odom_msg = Odometry()
        odom_msg.pose.pose.position.x = self.pos_x
        odom_msg.pose.pose.position.y = self.pos_y
        odom_msg.twist.twist.linear.x = speed_x
        odom_msg.twist.twist.linear.y = speed_y
        quat = self.heading_to_quaternion(heading)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        self.vessel_pub.publish(odom_msg)

    def colav_callback(self, msg: GuidanceData):
        if msg.is_colav:
            speed = msg.u
            heading = msg.psi
            self.step(speed, heading)
        else:
            speed = self.speed
            heading = np.arctan2(self.goal_pos_y - self.pos_y, self.goal_pos_x - self.pos_x)
            self.step(speed, heading)

    def obstacle_callback(self):
        obs_speed_x = self.obstacle_speed*np.cos(self.obstacle_heading)
        obs_speed_y = self.obstacle_speed*np.sin(self.obstacle_heading)
        self.obstacle_pos_x += obs_speed_x*self.dt
        self.obstacle_pos_y += obs_speed_y*self.dt
        obs_msg = Odometry()
        obs_msg.pose.pose.position.x = self.obstacle_pos_x
        obs_msg.pose.pose.position.y = self.obstacle_pos_y
        obs_msg.twist.twist.linear.x = obs_speed_x
        obs_msg.twist.twist.linear.y = obs_speed_y
        quaternion = euler2quat(0, 0, self.obstacle_heading)
        # self.get_logger().info(f"Quat: {quaternion}")
        obs_msg.pose.pose.orientation.x = quaternion[0]
        obs_msg.pose.pose.orientation.y = quaternion[1]
        obs_msg.pose.pose.orientation.z = quaternion[2]
        obs_msg.pose.pose.orientation.w = quaternion[3]
        self.obstacle_pub.publish(obs_msg)

    @staticmethod
    def get_heading(msg: Odometry):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z
        ]
        heading = quat2euler(orientation_list)[2]
        return heading
    
    @staticmethod
    def heading_to_quaternion(heading):
        return euler2quat(0, 0, heading)
    

    
def main(args=None):
    rclpy.init(args=args)
    simulator = Simulator()
    rclpy.spin(simulator)
    rclpy.shutdown()

if __name__ == "__main__":
    main()


        