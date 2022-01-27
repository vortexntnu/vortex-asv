#!/usr/bin/python3
# written by Ronja Kræmer, Student

import rospy
import actionlib
from rospy.core import rospydebug
from vortex_msgs.msg import (
    LosPathFollowingAction,
    LosPathFollowingGoal,
    LosPathFollowingResult,
    LosPathFollowingFeedback,
)
from vortex_msgs.srv import Waypoint, WaypointRequest, WaypointResponse


class WaypointManager:
    """ 
    Nodes created:
        WaypointManager

    Subscribes to:

    Publishes to:

    """

    def __init__(self):
        rospy.init_node("WaypointManager")

        self.waypoint_list = []

        #Action client
        self.action_client = actionlib.SimpleActionClient(
            "los_action_server", LosPathFollowingAction
        )
        self.action_client.wait_for_server()

        #Services
        self.add_waypoint_service = rospy.Service(
            "add_waypoint", Waypoint, self.add_waypoint_to_list
        )
        self.remove_waypoint_service = rospy.Service(
            "remove_waypoint", Waypoint, self.remove_waypoint_from_list
        )

    #req is a waypoint
    def add_waypoint_to_list(self, req):        
        self.waypoint_list.append(req)
        rospy.loginfo("add waypoint to waypoint_list")

    def remove_waypoint_from_list(self, req):
        self.waypoint_list.remove(req)
        rospy.loginfo("remove waypoint from waypoint_list")


    def spin(self):
        index_waypoint_k = 0
        while not rospy.is_shutdown():
            if len(self.waypoint_list) >= 2:
                if index_waypoint_k < len(self.waypoint_list) - 1:
                    goal = LosPathFollowingGoal()
                    rospy.loginfo("define goal to send to los_guidance_node")
            
                    goal.waypoints[0] = self.waypoint_list[index_waypoint_k]
                    goal.waypoints[1] = self.waypoint_list[index_waypoint_k + 1]
                    rospy.loginfo("add waypoints to goal")

                    self.action_client.send_goal(self._goal)
                    rospy.loginfo("send goal to los_guidance_node")
                    self.action_client.wait_for_result()
                    rospy.loginfo("receive result from los_guidance_node")
                    index_waypoint_k += 1
                else:
                    self.waypoint_list.clear()
                    rospy.loginfo("clear waypoint_list")


if __name__ == "__main__":
    try:
        waypoint_manager = WaypointManager()

        waypoint_manager.spin()
    except rospy.ROSInterruptException:
        pass
