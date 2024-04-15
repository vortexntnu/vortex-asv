#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from vortex_msgs.msg import GuidanceData, OdometryArray
import math
from VO_math import VelocityObstacle, Obstacle, Zones, Approaches
from tf2_ros import Buffer, TransformListener
from transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion
import numpy as np

import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt

plt.ion()  # Turn interactive mode on
fig, ax = plt.subplots()  # Pre-create the figure and axes

class ColavController(Node):

    def __init__(self):
        super().__init__("colav_controller")

        self.vessel_path = []  # To store vessel positions over time

        self.declare_parameter('guidance_interface/colav_data_topic', 'guidance/collision_avoidance')    
        self.declare_parameter('stop_zone_radius', 0.0)
        self.declare_parameter('colimm_max_radius', math.inf)

        stop_zone_radius = self.get_parameter('stop_zone_radius').value
        colimm_max_radius = self.get_parameter('colimm_max_radius').value
        colav_data_topic = self.get_parameter('guidance_interface/colav_data_topic').get_parameter_value().string_value

        self.obstacle_sub = self.create_subscription(OdometryArray, "/tracking/mul_tracked_cv_objects", self.obst_callback, 10)
        self.vessel_sub = self.create_subscription(Odometry, "/pose_gt", self.vessel_callback, 10)
        self.colav_pub = self.create_publisher(GuidanceData, colav_data_topic, 10)
        
        self.obstacles = []
        self.vessel_odom = Odometry()
        self.vessel = Obstacle() 
        self.vessel.radius = stop_zone_radius
        self.stop_zone_radius = stop_zone_radius
        self.colimm_max_radius = colimm_max_radius
        self.current_time = self.get_clock().now()
    
    def vessel_callback(self, msg):
        self.vessel_odom = msg
        self.vessel = self.odometry_to_obstacle(msg)
        self.t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

        self.vessel_path.append((self.vessel.x, self.vessel.y))
        self.plot_positions()

    def obst_callback(self, msg):
        self.obstacles = [self.odometry_to_obstacle(odom) for odom in msg.odoms]
        if not self.obstacles:
            self.get_logger().info('No obstacles detected!')
            return
        colav_data = self.gen_colav_data()
        if colav_data:
            self.colav_pub.publish(colav_data)
    
    @staticmethod
    def quaternion_to_euler(quaternion: Quaternion):
        """Convert a ROS Quaternion message to Euler angles (roll, pitch, yaw)."""
        q = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        return euler_from_quaternion(q)

    def odometry_to_obstacle(self, odom: Odometry) -> Obstacle:
        obstacle = Obstacle()
        obstacle.x = odom.pose.pose.position.x
        obstacle.y = odom.pose.pose.position.y
        obstacle.vx = odom.twist.twist.linear.x
        obstacle.vy = odom.twist.twist.linear.y
        obstacle.heading = np.arctan2(obstacle.vy, obstacle.vx)  # Assuming heading is direction of velocity
        obstacle.speed = np.sqrt(obstacle.vx**2 + obstacle.vy**2)
        
        # Assuming 'r' (radius) needs to be calculated or set here. You might have a different way to determine it.
        # obstacle.r = <some_value_or_calculation>

        return obstacle


    def get_distance(self, obstacle1: Obstacle, obstacle2: Obstacle):
        return math.hypot(obstacle1.x - obstacle2.x, obstacle1.y - obstacle2.y)

    def get_closest_obst(self, obstacles: list[Obstacle], vessel: Obstacle):
        return min(obstacles, key=lambda obs: self.get_distance(obs, vessel), default=None)

    def gen_colav_data(self):
        closest_obst = self.get_closest_obst(self.obstacles, self.vessel)
        if closest_obst is None:
            self.get_logger().info('No obstacles detected.')
            return None

        VO = VelocityObstacle(self.vessel, closest_obst)
        VO.set_cone_angles()

        self.get_logger().info(f'Closest obstacle at {closest_obst.x}, {closest_obst.y}')
        self.get_logger().info(f'Vessel at {self.vessel.x}, {self.vessel.y}')
        self.get_logger().info(f'VO Angles: Left {VO.left_angle}, Right {VO.right_angle}')

        zone = self.get_zone(closest_obst, self.vessel)
        self.get_logger().info(f'Zone: {zone}')

        if zone == Zones.NOCOL:
            return None
        elif zone == Zones.STOPNOW:
            return self.create_guidance_data(0, 0, self.vessel.heading, self.vessel_odom)
        elif zone == Zones.COLIMM and not VO.check_if_collision():
            return None

        approach = self.gen_approach(closest_obst, self.vessel)

        if approach in [Approaches.FRONT, Approaches.RIGHT]:
            buffer = math.pi / 6  # 30 degrees
            new_heading = VO.right_angle - buffer
            return self.create_guidance_data(self.vessel.speed, new_heading, self.vessel.heading, self.vessel_odom)
        elif approach in [Approaches.BEHIND, Approaches.LEFT]:
            return None
        return None

    def create_guidance_data(self, speed, psi_d, vessel_heading, vessel_odom):
        data = GuidanceData()
        data.u_d = float(speed)
        data.psi_d = float(psi_d)
        data.u = float(speed)  # Assuming this is the desired speed
        data.t = float(self.get_clock().now().seconds_nanoseconds()[0]) + float(self.get_clock().now().seconds_nanoseconds()[1]) * 1e-9
        orientation_q = Quaternion(
            x=vessel_odom.pose.pose.orientation.x,
            y=vessel_odom.pose.pose.orientation.y,
            z=vessel_odom.pose.pose.orientation.z,
            w=vessel_odom.pose.pose.orientation.w)
        _, _, yaw = self.quaternion_to_euler(orientation_q)
        data.psi = float(yaw)
        return data

    def gen_approach(self, obstacle: Obstacle, vessel: Obstacle):
        dx = obstacle.x - vessel.x
        dy = obstacle.y - vessel.y
        buffer = 10 * math.pi / 180  # 10 degrees in radians
        phi = math.atan2(dy, dx)

        if vessel.heading + buffer > phi > vessel.heading - buffer:
            return Approaches.FRONT
        elif phi < vessel.heading - buffer:
            return Approaches.RIGHT
        elif phi > vessel.heading + buffer:
            return Approaches.LEFT
        return Approaches.BEHIND

    def get_zone(self, obstacle: Obstacle, vessel: Obstacle):
        distance = self.get_distance(obstacle, vessel)
        if distance > self.colimm_max_radius:
            return Zones.NOCOL
        elif self.stop_zone_radius < distance <= self.colimm_max_radius:
            return Zones.COLIMM
        return Zones.STOPNOW

    def plot_positions(self):
        ax.clear()  # Clear previous drawings
        if self.vessel_path:
            path_x, path_y = zip(*self.vessel_path)
            ax.plot(path_x, path_y, 'b-', label='Vessel Path')
        ax.plot(self.vessel.x, self.vessel.y, 'bo', label='Vessel Current')
        for obstacle in self.obstacles:
            ax.plot(obstacle.x, obstacle.y, 'ro', label='Obstacle')
        ax.set_xlabel('X position')
        ax.set_ylabel('Y position')
        ax.legend()
        ax.grid(True)
        plt.title('Vessel and Obstacles Position and Path')
        fig.canvas.draw_idle()  # Redraw the current figure
        fig.canvas.flush_events()  # Process events like key presses or resizes



def main(args=None):
    rclpy.init()
    colav_controller = ColavController()
    rclpy.spin(colav_controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()    