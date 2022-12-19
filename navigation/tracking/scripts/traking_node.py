#!/usr/bin/python3

import rospy


from geometry_msgs.msg import PoseStamped, Point


class Tracker:
    """
    Nodes created:
    Subscribes to:
    Publishes to:
    """

    def __init__(self):
        rospy.init_node("Tracker")

    def spin(self):

        while not rospy.is_shutdown():
            write_something = 2



if __name__ == "__main__":
    try:
        waypoint_manager = Tracker()

        waypoint_manager.spin()
    except rospy.ROSInterruptException:
        pass
