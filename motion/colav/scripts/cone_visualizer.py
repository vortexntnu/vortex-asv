#!/usr/bin/env python3

import colav_controller
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class ConeVisualizer(Node):
    def __init__(self):
        super().__init__('cone_visualizer')
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(0.1, self.publish_marker)

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "cone"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = 1.0
        marker.pose.position.y = 1.0
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 0.8
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.publisher.publish(marker)

if __name__ == '__main__':
    rclpy.init()
    node = ConeVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()