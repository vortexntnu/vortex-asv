#!/usr/bin/python3

import rospy
import numpy as np
from track_manager import TRACK_MANAGER


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
        rospy.Subscriber("position_measurments", Point, self.cb_function) #what type should be used? 
        self.pub = rospy.Publisher("position_velocity_estimates", Point, queue_size=10) #use nav_msgs/odometry

        self.track_manager = TRACK_MANAGER()


    def cb_function(self, data):

        self.track_manager.cb(data)
        self.publish()

    def publish(self):
        a = 3
        #if track is confirmed, publish odometry


if __name__ == "__main__":
    try:
        tracker = Tracker()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
