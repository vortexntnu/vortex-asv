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

class ColavController(Node):
    def __init__(self):
        super().__init__("colav_controller")

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

    def obst_callback(self, msg):
        self.obstacles = [self.odometry_to_obstacle(odom) for odom in msg.odometry_array]
        if not self.obstacles:
            self.get_logger().info('No obstacles detected!')
            return
        colav_data = self.gen_colav_data()
        if colav_data:
            self.colav_pub.publish(colav_data)
    
    def quaternion_to_euler(quaternion: Quaternion):
        """Convert a ROS Quaternion message to Euler angles (roll, pitch, yaw)."""
        q = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        return euler_from_quaternion(q)

    def odometry_to_obstacle(self, odometry: Odometry):
        # Convert Odometry message to an Obstacle object
        # Assuming Obstacle class exists and is properly defined elsewhere
        quaternion = (
            odometry.pose.pose.orientation.x,
            odometry.pose.pose.orientation.y,
            odometry.pose.pose.orientation.z,
            odometry.pose.pose.orientation.w)
        _, _, yaw = euler_from_quaternion(quaternion)
        
        return Obstacle(
            x=odometry.pose.pose.position.x,
            y=odometry.pose.pose.position.y,
            vx=odometry.twist.twist.linear.x,
            vy=odometry.twist.twist.linear.y,
            heading=yaw,
            speed=math.hypot(odometry.twist.twist.linear.x, odometry.twist.twist.linear.y),
            radius=2)  # Assuming a fixed radius for simplicity

    def get_distance(self, obstacle1: Obstacle, obstacle2: Obstacle):
        return math.hypot(obstacle1.x - obstacle2.x, obstacle1.y - obstacle2.y)

    def get_closest_obst(self, obstacles: list[Obstacle], vessel: Obstacle):
        return min(obstacles, key=lambda obs: self.get_distance(obs, vessel), default=None)

    def gen_colav_data(self):
        closest_obst = self.get_closest_obst(self.obstacles)
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
        data.u_d = speed
        data.psi_d = psi_d
        data.u = speed  # Assuming this is the desired speed
        data.t = self.get_clock().now().to_msg()
        orientation_q = Quaternion(
            x=vessel_odom.pose.pose.orientation.x,
            y=vessel_odom.pose.pose.orientation.y,
            z=vessel_odom.pose.pose.orientation.z,
            w=vessel_odom.pose.pose.orientation.w)
        _, _, yaw = self.quaternion_to_euler(orientation_q)
        data.psi = yaw
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

def main(args=None):
    rclpy.init()
    colav_controller = ColavController()
    rclpy.spin(colav_controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()    