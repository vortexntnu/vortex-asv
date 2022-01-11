#!/usr/bin/python3
# written by Ronja Kræmer, Student

import rospy
import actionlib
from vortex_msgs.msg import LosPathFollowingAction, LosPathFollowingGoal, LosPathFollowingResult, LosPathFollowingFeedback


class WaypointManager():
    def __init__(self):
        rospy.init_node('WaypointManager')
        self.client = actionlib.SimpleActionClient(name='los_action_server', ActionSpec=LosPathFollowingAction)

        
        self.waypoint_list = []

    def spin(self):
        while not rospy.is_shutdown():
            pass

























if __name__ == '__main__':
	try:
		waypoint_manager = WaypointManager()

		waypoint_manager.spin()
	except rospy.ROSInterruptException:
		pass