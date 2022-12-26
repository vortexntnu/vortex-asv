#!/usr/bin/python3

import rospy
import numpy as np


from geometry_msgs.msg import PoseStamped, Point

"""
Estimate position and velocity for each boat realtive to the vessel (given measurements for the boats position).
"""


class Tracker:
    """
    Nodes created:
    Subscribes to:
    Publishes to:
    """

    def __init__(self):

        rospy.init_node("Tracker")
        rospy.Subscriber("position_measurments", Point, self.cb_function)
        self.pub = rospy.Publisher("position_velocity_estimates", Point, queue_size=10)

        # x = [r, thetha, r', theta']

        self.time_step = 0.1
        self.x_pri = np.ndarray(
            (4,), buffer=np.array([0.0, 0.0, 0.0, 0.0]), dtype=float
        )
        self.P_pri = np.ndarray(
            (4, 4),
            buffer=np.array(
                [
                    [0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0],
                ]
            ),
            dtype=float,
        )

        self.x_post = np.ndarray((4,), dtype=float)
        self.P_post = np.ndarray((4, 4), dtype=float)

        self.L = np.ndarray((2, 2), dtype=float)

        self.C = np.ndarray(
            (2, 4),
            buffer=np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]]),
            dtype=float,
        )

        self.A = np.ndarray(
            (4, 4),
            buffer=np.array(
                [
                    [1.0, 0.0, self.time_step, 0],
                    [0, 1.0, 0, self.time_step],
                    [0, 0, 1.0, 0],  # assuming constnat velocity
                    [0, 0, 0, 1.0],
                ]
            ),  # assuming constnat velocity
            dtype=float,
        )

        self.Q = np.ndarray(
            (4, 4),
            buffer=np.array(
                [[0.001, 0, 0, 0], [0, 0.001, 0, 0], [0, 0, 0.1, 0], [0, 0, 0, 0.1]]
            ),
            dtype=float,
        )

        self.R = np.ndarray((2, 2), buffer=np.array([[0.1, 0], [0, 0.1]]), dtype=float)

    def prediction_step(self):
        self.x_pri = np.matmul(self.A, self.x_post)
        self.P_pri = (
            np.matmul(self.A, np.matmul(self.P_post, np.transpose(self.A))) + self.Q
        )

    def correction_step(self, y):

        temp1 = np.matmul(self.P_pri, np.transpose(self.C))
        temp2 = np.matmul(self.C, temp1)
        self.L = np.matmul(temp1, np.linalg.inv(temp2 + self.R))

        temp3 = np.matmul(self.C, self.x_pri)
        self.x_post = self.x_pri + np.matmul(self.L, (y - temp3))

        temp4 = np.identity(len(self.x_pri)) - np.matmul(self.L, self.C)
        temp5 = np.matmul(self.L, np.matmul(self.R, np.transpose(self.L)))
        self.P_post = (
            np.matmul(temp4, np.matmul(self.P_pri, np.transpose(temp4))) + temp5
        )

    def cb_function(self, data):

        self.correction_step(data)
        self.prediction_step()

        self.pub.publish(self.x_post)


if __name__ == "__main__":
    try:
        tracker = Tracker()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
