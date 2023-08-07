#!/usr/bin/python3
from dataclasses import dataclass

from std_msgs.msg import Float64MultiArray, Bool, Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import rospy
import numpy as np

CONVERGENCE_RADIUS = 0.25  # the radius of convergence in x-y
CONVERGENCE_ANGLE = np.pi / 8  # the convergence range for heading in radians

# TODO: Add option to set path-dependent heading

@dataclass
class Waypoint:
    north: float
    east: float
    heading: float


class LQRGuidanceROS:

    def __init__(self):
        rospy.init_node("lqr_guidance")

        self.waypoints = []
        self.current_waypoint_index = None

        # Subscriptions
        rospy.Subscriber("/odometry/filtered", Odometry,
                         self.odometry_callback)
        rospy.Subscriber("/guidance/lqr/clear_waypoints", Empty,
                         self.clear_waypoints_callback)
        rospy.Subscriber("/guidance/lqr/add_waypoint", Point,
                         self.add_waypoint_callback)

        # Publishers
        self.setpoint_pub = rospy.Publisher("/controller/lqr/setpoints",
                                            Float64MultiArray,
                                            queue_size=10)
        self.enable_pub = rospy.Publisher("/controller/lqr/enable",
                                          Bool,
                                          queue_size=10)

    def odometry_callback(self, data):
        """ Check if vessel is close to the waypoint and send the next one if so """
        if self.current_waypoint_index is not None:
            waypoint = self.waypoints[self.current_waypoint_index]
            dx = data.pose.pose.position.x - waypoint.north
            dy = data.pose.pose.position.y - waypoint.east
            if np.hypot(dx, dy) < CONVERGENCE_RADIUS:
                rospy.loginfo(
                    f"Reached waypoint {self.current_waypoint_index}")
                self.send_waypoint(self.current_waypoint_index + 1)

    def publish_setpoint(self, waypoint):

        msg = Float64MultiArray()
        msg.data = [waypoint.north, waypoint.east, waypoint.heading, 0, 0, 0]

        self.setpoint_pub.publish(msg)

    def send_waypoint(self, index):
        """ Send the waypoint at the given index to the controller """
        if 0 <= index < len(self.waypoints):
            self.current_waypoint_index = index
            waypoint = self.waypoints[index]
            rospy.loginfo(f"Sending waypoint {index}: {waypoint}")
            self.publish_setpoint(waypoint)

            #self.enable_pub.publish(True)
        else:
            rospy.loginfo("Final waypoint reached!")
            #self.enable_pub.publish(False)

    def clear_waypoints_callback(self, data):
        """ Callback for the clear waypoints topic """
        self.waypoints = []
        rospy.loginfo("Waypoints cleared!")

    def add_waypoint_callback(self, data):
        """ Callback for the add waypoint topic """
        waypoint = Waypoint(data.x, data.y, data.z)
        self.waypoints.append(waypoint)
        rospy.loginfo(f"Waypoint added: {waypoint}")

        # if it's the first waypoint, send it immediately
        if len(self.waypoints) == 1:
            self.send_waypoint(0)


if __name__ == "__main__":

    guidance_scheme = LQRGuidanceROS()

    rospy.spin()
