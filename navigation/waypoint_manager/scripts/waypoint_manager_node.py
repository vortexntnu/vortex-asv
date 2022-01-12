#!/usr/bin/python3
# written by Ronja Kræmer, Student

import rospy
import actionlib
from vortex_msgs.msg import LosPathFollowingAction, LosPathFollowingGoal, LosPathFollowingResult, LosPathFollowingFeedback


class WaypointManager():
    """"
    

    Attributes:
        _goal       A vortex_msgs action that contains the next waypoint

    Nodes created:
        WaypointManager

    Subscribes to:

    Publishes to:


    """
                

    def __init__(self):
        rospy.init_node('WaypointManager')

        self.waypoint_list = []

        # Action client
        self.action_client = actionlib.SimpleActionClient(name='los_action_server', ActionSpec=LosPathFollowingAction)
        self.action_client.wait_for_server()

    
    def addWaypointToList():
       pass

    def removeWaypointFromList():
        pass




    def spin(self):
        index_waypoint_k = 0 
        while not rospy.is_shutdown():
            if len(self.waypoint_list) >= 2:
                if index_waypoint_k < len(self.waypoint_list)-1:
                    goal = LosPathFollowingGoal()  
                    self.goal.waypoints[0]=self.waypoint_list[index_waypoint_k]
                    self.goal.waypoints[1]=self.waypoint_list[index_waypoint_k+1]
                    self.action_client.send_goal(self._goal)
                    self.action_client.wait_for_result()
                    index_waypoint_k += 1
                else:
                    self.waypoint_list.clear()




if __name__ == '__main__':
	try:
		waypoint_manager = WaypointManager()

		waypoint_manager.spin()
	except rospy.ROSInterruptException:
		pass