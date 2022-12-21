#!/usr/bin/python3

import rospy
import numpy as np


from geometry_msgs.msg import PoseStamped, Point

"""
Task: Estimate position and velocity of surronding boats. 
Sub task: Detect boats based on LIDAR data. 
Sub task: Distinguish boats from each other and other objects.
Sub task: Estimate position and velocity for each boat realtive to the vessel (given measurements for the boats position).
    - Set up KF. 
        Use Kalmanfilter with 4 states: r - radius, theta - angle, r_der - change in radius, thetha_der - change in angle.
        There are 2 measurments: r and theta. 
        Define A and C matricies. B and u are 0. 
        Define Q and R matricies. Can R be updated real time based on the certanty of the detections? 
    - Should we use a different type of KF? 
"""


class Tracker:
    """
    Nodes created:
    Subscribes to:
    Publishes to:
    """

    def __init__(self):
        rospy.init_node("Tracker")

        #x = [r, thetha, r', theta']

        self.time_step = 0.002
        self.x_pri = np.ones(4)
        self.P_pri = np.matrix([
            [0.1,0,  0,  0], 
            [0,  0.1,0,  0], 
            [0,  0,  0.1,0], 
            [0,  0,  0,  0.1]])

        self.x_post = np.ones(4)
        self.P_post = np.matrix([
            [0.1,0,  0,  0], 
            [0,  0.1,0,  0], 
            [0,  0,  0.1,0], 
            [0,  0,  0,  0.1]])

        self.A = np.matrix([
            [1, 0, 1, 0], 
            [0, 1, 0, 1],
            [0, 0, 1, 0], #assuming constnat velocity
            [0, 0, 0, 1]])#assuming constnat velocity

        self.C = np.matrix([
            [1, 0, 0, 0], 
            [0, 1, 0, 0]]
        )

        self.Q = np.matrix([
            [0.001, 0, 0, 0], 
            [0, 0.001, 0, 0],
            [0, 0, 0.1, 0], 
            [0, 0, 0, 0.1]])

        self.R = np.matrix([
            [0.1, 0], 
            [0, 0.1]])

    def spin(self):

        while not rospy.is_shutdown():
            write_something = 2



if __name__ == "__main__":
    try:
        tracker = Tracker()

        tracker.spin()
    except rospy.ROSInterruptException:
        pass
