#!/usr/bin/env python3

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from vortex_msgs.msg import GuidanceData, OdometryArray
from matplotlib.patches import Circle, Arrow, Wedge

# Assuming VO_math.py and your defined classes are imported correctly
from enum import Enum
import math

class Zones(Enum):
    NOCOL = 1
    COLIMM = 2
    STOPNOW = 3

class Approaches(Enum):
    FRONT = 1
    RIGHT = 2
    LEFT = 3
    BEHIND = 4

class Obstacle:
    def __init__(self) -> None:
        self.vx = 0
        self.vy = 0
        self.r = 2  # Example radius
        self.x = 0
        self.y = 0
        self.heading = 0
        self.speed = 0

class VelocityObstacle:
    def __init__(self, vessel: Obstacle, obstacle: Obstacle) -> None:
        self.vessel = vessel
        self.obstacle = obstacle
        self.left_angle = 0.0
        self.right_angle = 0.0
        self.truncated_time = 5  # placeholder

    def set_cone_angles(self) -> None:
        theta_ro = math.atan2(self.obstacle.y - self.vessel.y, self.obstacle.x - self.vessel.x)
        distance = math.sqrt((self.obstacle.x - self.vessel.x) ** 2 + (self.obstacle.y - self.vessel.y) ** 2)
        if distance == 0:
            return
        theta_ray = math.asin((self.vessel.r + self.obstacle.r) / distance)
        self.right_angle = theta_ro - theta_ray
        self.left_angle = theta_ro + theta_ray

    def check_if_collision(self) -> bool:
        dvx = self.obstacle.vx - self.vessel.vx
        dvy = self.obstacle.vy - self.vessel.vy
        angle = math.atan2(-dvy, -dvx)
        return angle > self.right_angle and angle < self.left_angle

class PlotterNode(Node):
    def __init__(self):
        super().__init__('plotter_node')
        self.fig, self.ax = plt.subplots()
        self.vessel_pos, = self.ax.plot([], [], 'ro', label='Vessel')  # Red dot for vessel
        self.obstacle_pos, = self.ax.plot([], [], 'go', label='Obstacles')  # Green dots for obstacles
        self.collision_zones = []

        self.ax.axis('equal')
        self.ax.set_xlim(-50, 50)
        self.ax.set_ylim(-50, 50)
        self.ax.legend()

        self.vessel_subscription = self.create_subscription(Odometry, '/pose_gt', self.vessel_callback, 10)
        self.obstacle_subscription = self.create_subscription(OdometryArray, '/tracking/mul_tracked_cv_objects', self.obstacle_callback, 10)

        self.vessel = Obstacle()
        self.obstacles = []

    def vessel_callback(self, msg):
        self.vessel.x = msg.pose.pose.position.x
        self.vessel.y = msg.pose.pose.position.y
        self.vessel.vx = msg.twist.twist.linear.x
        self.vessel.vy = msg.twist.twist.linear.y

    def obstacle_callback(self, msg):
        self.obstacles = [Obstacle() for _ in msg.odoms]
        for obs, odom in zip(self.obstacles, msg.odoms):
            obs.x = odom.pose.pose.position.x
            obs.y = odom.pose.pose.position.y
            obs.vx = odom.twist.twist.linear.x
            obs.vy = odom.twist.twist.linear.y

    def update_plot(self, frame):
        self.vessel_pos.set_data(self.vessel.x, self.vessel.y)
        self.obstacle_pos.set_data([obs.x for obs in self.obstacles], [obs.y for obs in self.obstacles])

        # Clear previous collision zones
        for patch in self.collision_zones:
            patch.remove()
        self.collision_zones = []

        # Collision detection and path planning
        for obstacle in self.obstacles:
            vo = VelocityObstacle(self.vessel, obstacle)
            vo.set_cone_angles()
            if vo.check_if_collision():
                wedge = Wedge((self.vessel.x, self.vessel.y), 10, np.degrees(vo.right_angle), np.degrees(vo.left_angle), color='orange', alpha=0.5)
                self.ax.add_patch(wedge)
                self.collision_zones.append(wedge)

        return [self.vessel_pos, self.obstacle_pos] + self.collision_zones

    def start_animation(self):
        ani = FuncAnimation(self.fig, self.update_plot, frames=100, interval=100, blit=False)
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    plotter_node = PlotterNode()
    rclpy.spin_once(plotter_node, timeout_sec=0.1)  # Spin briefly to update positions
    plotter_node.start_animation()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
