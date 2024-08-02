#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from vortex_msgs.msg import GuidanceData, OdometryArray
import math
from VO_math import VelocityObstacle, Obstacle, Zones, Approaches
from tf2_ros import Buffer, TransformListener
from transformations import euler_from_quaternion
from transforms3d.euler import quat2euler
from geometry_msgs.msg import Quaternion
import numpy as np
import matplotlib.pyplot as plt

class ColavController(Node):
    def __init__(self):
        super().__init__("colav_controller")

        self.declare_parameter('guidance_interface/colav_data_topic', 'guidance/collision_avoidance')    
        self.declare_parameter('stop_zone_radius', 0.9)
        self.declare_parameter('colimm_max_radius', 2.0)

        stop_zone_radius = self.get_parameter('stop_zone_radius').value
        colimm_max_radius = self.get_parameter('colimm_max_radius').value
        colav_data_topic = self.get_parameter('guidance_interface/colav_data_topic').get_parameter_value().string_value

        self.obstacle_sub = self.create_subscription(Odometry, "/obstacle", self.obst_callback, 10)
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
        #print("vessel odometry: ", self.vessel_odom)
        self.t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

    def obst_callback(self, msg):
        self.obstacles = [self.odometry_to_obstacle(msg)]
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

        # self.get_logger().info(f'Closest obstacle at {closest_obst.x}, {closest_obst.y}')
        # self.get_logger().info(f'Vessel at {self.vessel.x}, {self.vessel.y}')
        # self.get_logger().info(f'VO Angles: Left {VO.left_angle}, Right {VO.right_angle}')

        #self.get_logger().info(f'Heading Angle: Left {self.vessel.heading}')

        zone = self.get_zone(closest_obst, self.vessel)
        self.get_logger().info(f'Zone: {zone}')

        if zone == Zones.NOCOL:
            return self.create_guidance_data(self.vessel.speed, self.vessel.heading, self.vessel.heading, self.vessel_odom, is_colav=False)
        elif zone == Zones.STOPNOW:
            return self.create_guidance_data(0, 0, self.vessel.heading, self.vessel_odom)
        elif zone == Zones.COLIMM and not VO.check_if_collision():
            self.get_logger().info(f'No collision detected!')
            return self.create_guidance_data(self.vessel.speed, self.vessel.heading, self.vessel.heading, self.vessel_odom, is_colav=True)

        approach = self.gen_approach(closest_obst, self.vessel)
        self.get_logger().info(f'Collition Detected!!!!')

        if approach in [Approaches.FRONT, Approaches.RIGHT]:
            buffer = math.pi / 6  # 30 degrees
            new_heading = VO.right_angle - buffer
            #self.get_logger().info(f'New heading: {new_heading}, current heading: {self.vessel.heading}, VO right angle: {VO.right_angle}, buffer: {buffer}')
            return self.create_guidance_data(self.vessel.speed, new_heading, self.vessel.heading, self.vessel_odom)
        elif approach in [Approaches.BEHIND, Approaches.LEFT]:
            return None
        return None
    
        # TODO: Calculate proper psi_d

    def create_guidance_data(self, speed, psi_d, vessel_heading, vessel_odom, is_colav=True):
        data = GuidanceData()
        data.u_d = float(speed)
        data.psi_d = float(psi_d)
        data.u = float(speed)  # Assuming this is the desired speed
        data.t = float(self.get_clock().now().seconds_nanoseconds()[0]) + float(self.get_clock().now().seconds_nanoseconds()[1]) * 1e-9
        # orientation_q = Quaternion(
        #     x=vessel_odom.pose.pose.orientation.x,
        #     y=vessel_odom.pose.pose.orientation.y,
        #     z=vessel_odom.pose.pose.orientation.z,
        #     w=vessel_odom.pose.pose.orientation.w)
        orientation_list = [
            vessel_odom.pose.pose.orientation.x,
            vessel_odom.pose.pose.orientation.y,
            vessel_odom.pose.pose.orientation.z,
            vessel_odom.pose.pose.orientation.w
        ]
        # _, _, yaw = self.quaternion_to_euler(orientation_q)
        yaw = quat2euler(orientation_list)[2]
        self.get_logger().info(f'Current yaw: {yaw}')
        data.psi = float(yaw)
        data.is_colav = is_colav
        return data
    
    def normalize_angle(self, angle):
        """Normalize angle to be within the range [0, 2*pi)"""
        return angle % (2 * math.pi)

    def gen_approach(self, obstacle: Obstacle, vessel: Obstacle):
        dx = obstacle.x - vessel.x
        dy = obstacle.y - vessel.y
        buffer = 30 * math.pi / 180  # 10 degrees in radians
        phi = math.atan2(dy, dx)

        if vessel.heading + buffer > phi > vessel.heading - buffer:
            return Approaches.FRONT
        elif self.normalize_angle(math.pi - (vessel.heading - buffer)) < phi < self.normalize_angle(vessel.heading - buffer):
            return Approaches.RIGHT
        elif self.normalize_angle(math.pi - (vessel.heading + buffer)) > phi > self.normalize_angle(vessel.heading + buffer):
            return Approaches.LEFT
        return Approaches.BEHIND

    def get_zone(self, obstacle: Obstacle, vessel: Obstacle):
        distance = self.get_distance(obstacle, vessel)
        if distance > self.colimm_max_radius:
            return Zones.NOCOL
        elif self.stop_zone_radius < distance <= self.colimm_max_radius:
            return Zones.COLIMM
        return Zones.STOPNOW


# TODO: add actual controller and not just generate references

def main(args=None):
    rclpy.init()
    colav_controller = ColavController()
    rclpy.spin(colav_controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()    