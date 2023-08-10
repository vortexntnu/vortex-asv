#!/usr/bin/python3

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Point


class LQRInterface:

    def __init__(self):

        self.add_waypoint_publisher = rospy.Publisher(
            "/guidance/lqr/add_waypoint", Point, queue_size=10)
        self.enable_controller_publisher = rospy.Publisher(
            "/controller/lqr/enable", Bool, queue_size=10)
        self.toggle_path_dependent_heading = rospy.Publisher(
            "/guidance/lqr/toggle_path_dependent_heading", Bool, queue_size=10)

    def add_point(self, point, path_dependent_heading=False):
        self.add_waypoint_publisher.publish(point)

    def move_to_point(self, point):
        self.add_waypoint_publisher.publish(point)
        self.enable_controller_publisher.publish(Bool(True))

    def turn_on_controller(self):
        self.enable_controller_publisher.publish(Bool(True))

    def turn_off_controller(self):
        self.enable_controller_publisher.publish(Bool(False))

    def set_path_dependent_heading(self, data):
        self.toggle_path_dependent_heading.publish(Bool(data))


if __name__ == "__main__":

    interface = LQRInterface()

    rospy.spin()
