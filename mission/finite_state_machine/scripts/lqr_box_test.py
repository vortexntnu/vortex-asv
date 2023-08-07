#!/usr/bin/python3

from dataclasses import dataclass

import numpy as np

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Point


@dataclass
# TODO: Add heading
class Waypoint:
    north: float
    east: float


INTERPOLATION_STEP_SIZE = 1.0


class LQRControllerBoxTest:

    def __init__(self):
        rospy.init_node("lqr_box_path_generator")

        self.add_waypoint_publisher = rospy.Publisher(
            "/guidance/lqr/add_waypoint", Point, queue_size=10)
        self.enable_controller_publisher = rospy.Publisher(
            "/controller/lqr/enable", Bool, queue_size=10)

        north = 3
        east = 3

        rospy.sleep(2)  # TODO: wait for signal or smth

        self.generate_and_publish_box(north, east)

    def generate_and_publish_box(self, north, east):
        waypoints = [
            Point(0.0, 0.0, 0.0),  # Starting position
            Point(north, east, -np.pi / 2),  # top_right
            Point(north, -east, -np.pi),  # top_left
            Point(-north, -east, np.pi / 2),  # bottom_left
            Point(-north, east, 0.0),  # bottom_right
            Point(north, east, -np.pi / 2),  # back to top_right
            Point(north, -east, -np.pi),  # top_left
            Point(-north, -east, np.pi / 2),  # bottom_left
            Point(-north, east, 0.0),  # bottom_right
            Point(north, east, -np.pi / 2),  # back to top_right
            Point(north, -east, -np.pi),  # top_left
            Point(-north, -east, np.pi / 2),  # bottom_left
            Point(-north, east, 0.0),  # bottom_right
            Point(north, east, -np.pi / 2),  # back to top_right
            Point(north, -east, -np.pi),  # top_left
            Point(-north, -east, np.pi / 2),  # bottom_left
            Point(-north, east, 0.0),  # bottom_right
            Point(north, east, -np.pi / 2),  # back to top_right
        ]

        # Interpolate and publish waypoints
        for i in range(len(waypoints) - 1):
            self.interpolate_and_publish(waypoints[i], waypoints[i + 1])

        rospy.sleep(1.0)
        self.enable_controller_publisher.publish(Bool(True))

    def interpolate_and_publish(self, waypoint1, waypoint2):
        """Interpolate between two waypoints and publish them."""
        dx = waypoint2.x - waypoint1.x
        dy = waypoint2.y - waypoint1.y
        distance = np.hypot(dx, dy)

        num_intermediate_points = int(
            np.ceil(distance / INTERPOLATION_STEP_SIZE))

        for i in range(num_intermediate_points +
                       1):  # +1 to include the endpoint
            ratio = i / num_intermediate_points  # this will be 1 for the endpoint
            intermediate_point = Point(
                waypoint1.x + ratio * dx,
                waypoint1.y + ratio * dy,
                waypoint1.z  # assuming constant heading
            )
            self.add_waypoint_publisher.publish(intermediate_point)
            rospy.sleep(0.01)  # you can adjust this delay as necessary


if __name__ == "__main__":

    path_generator = LQRControllerBoxTest()

    rospy.spin()
