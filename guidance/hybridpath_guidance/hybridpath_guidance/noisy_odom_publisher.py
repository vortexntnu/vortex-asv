#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vortex_msgs.srv import Waypoint
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler
import numpy as np
from rclpy.qos import QoSProfile, qos_profile_sensor_data, QoSReliabilityPolicy


qos_profile = QoSProfile(depth=1, history=qos_profile_sensor_data.history, 
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')

        self.eta_odom_publisher = self.create_publisher(Odometry, '/seapath/odom/ned', qos_profile)

        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.odom_callback)

        self.get_logger().info("Odom publisher started")

    def odom_callback(self):

        noise = np.random.normal(0, 0.005, 3)

        msg = Odometry()
        msg.pose.pose.position.x = 0.0 + noise[0]
        msg.pose.pose.position.y = 0.0 + noise[1]
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x = 0.0 + noise[0]
        msg.twist.twist.linear.y = 0.0 + noise[1]
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = 0.0 + noise[2]

        self.eta_odom_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()