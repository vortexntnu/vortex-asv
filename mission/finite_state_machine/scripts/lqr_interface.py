#!/usr/bin/python3

import rospy
from std_msgs.msg import Bool, Empty
from geometry_msgs.msg import Point

import numpy as np


class LQRInterface:

    def __init__(self):

        self.add_waypoint_publisher = rospy.Publisher(
            "/guidance/lqr/add_waypoint", Point, queue_size=10)
        self.enable_controller_publisher = rospy.Publisher(
            "/controller/lqr/enable", Bool, queue_size=10)
        self.toggle_path_dependent_heading = rospy.Publisher(
            "/guidance/lqr/toggle_path_dependent_heading", Bool, queue_size=10)
        self.clear_waypoints_publisher = rospy.Publisher(
            "/guidance/lqr/clear_waypoints", Empty, queue_size=10)

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

    def clear_all_waypoints(self):
        self.clear_waypoints_publisher.publish(Empty())

    def north_east_displacement_in_meters(target_north, target_east,
                                          origin_north, origin_east):
        earth_radius_wgs84 = 6371 * 1000.0

        meter_per_degree_lat = earth_radius_wgs84 * np.pi / 180.0
        meter_per_degree_lon = meter_per_degree_lat * np.cos(
            origin_north * np.pi / 180.0)

        displacement_north = (target_north -
                              origin_north) * meter_per_degree_lat
        displacement_east = (target_east - origin_east) * meter_per_degree_lon

        return displacement_north, displacement_east


if __name__ == "__main__":

    interface = LQRInterface()

    rospy.spin()
