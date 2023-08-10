#!/usr/bin/python3

from dataclasses import dataclass

import numpy as np

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Point

from lqr_interface import LQRInterface


@dataclass
# TODO: Add heading
class Waypoint:
    north: float
    east: float


INTERPOLATION_STEP_SIZE = 1.0


class LQRControllerBoxTest:

    def __init__(self):
        rospy.init_node("lqr_box_path_generator")

        self.lqr_interface = LQRInterface()


        north = 3
        east = 3

        rospy.sleep(2)  # TODO: wait for signal or smth

        self.generate_and_publish_box(north, east)

    def generate_and_publish_box(self, north, east):
        waypoints = [
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
        for waypoint in waypoints:
            self.lqr_interface.add_point(waypoint)
            rospy.sleep(0.1)

        self.lqr_interface.turn_on_controller()


if __name__ == "__main__":

    path_generator = LQRControllerBoxTest()

    rospy.spin()
