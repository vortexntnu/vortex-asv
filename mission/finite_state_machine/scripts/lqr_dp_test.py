#!/usr/bin/python3

"""
DP mode requires that you set a point, and that you turn off path-dependent heading
"""

import rospy
from geometry_msgs.msg import Point
from lqr_interface import LQRInterface

class LQRControllerDPTest:

    def __init__(self):
        rospy.init_node("lqr_dp_test")

        self.lqr_interface = LQRInterface()

        rospy.sleep(2)

        waypoint = Point(0.0, 0.0, 0.0)

        self.lqr_interface.add_point(waypoint, path_dependent_heading=True)
        self.lqr_interface.set_path_dependent_heading(False)
        self.lqr_interface.turn_on_controller()


if __name__ == "__main__":

    dp_test = LQRControllerDPTest()

    rospy.spin()
