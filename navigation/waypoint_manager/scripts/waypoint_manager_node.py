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
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point


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

        # Action client
        self.action_client = actionlib.SimpleActionClient(
            "/guidance/los_action_server", LosPathFollowingAction
        )
        self.action_client.wait_for_server()

        # Services
        self.add_waypoint_service = rospy.Service(
            "add_waypoint", Waypoint, self.add_waypoint_to_list
        )
        self.remove_waypoint_service = rospy.Service(
            "remove_waypoint", Waypoint, self.remove_waypoint_from_list
        )
        # Suggested function for simplicity in the mission/BouysTasksNjord.
        self.overwrite_waypoint_list_with_new_waypoint_service = rospy.Service(
            "overwrite_waypoint_list_with_new_waypoint", Waypoint, self.overwrite_waypoint_list_with_new_waypoint
        )


        # nav_msgs Path to visualize LOS in Rviz
        self.path_pub = rospy.Publisher("los_path", Path, queue_size=10)
        self.path = Path()
        self.path.header.frame_id = "world"

    def add_waypoint_to_list(self, req):
        self.waypoint_list.append(req.waypoint)
        rospy.loginfo("add waypoint to waypoint_list")
        newpose = PoseStamped()
        newpose.pose.position = Point(req.waypoint[0], req.waypoint[1], 0)
        self.path.poses.append(newpose)
        self.path_pub.publish(self.path)

        return WaypointResponse(True)

    def remove_waypoint_from_list(self, req):
        self.waypoint_list.remove(req)
        rospy.loginfo("remove waypoint from waypoint_list")
        self.path.poses.reverse()
        self.path.poses.pop()
        self.path.poses.reverse()
        self.path_pub.publish(self.path)

    # Suggested function for simplicity in the mission/BouysTasksNjord.
    def overwrite_waypoint_list_with_new_waypoint(self, req):
        self.waypoint_list = [req.waypoint]  # Replace existing waypoints with new waypoint
        rospy.loginfo("overwrite waypoint_list with new waypoint")
        newpose = PoseStamped()
        newpose.pose.position = Point(req.waypoint[0], req.waypoint[1], 0)
        self.path.poses = [newpose]  # Replace existing poses with new pose
        self.path_pub.publish(self.path)
        return WaypointResponse(True)
    

    def spin(self):
        index_waypoint_k = 0
        while not rospy.is_shutdown():
            if len(self.waypoint_list) >= 2:
                if index_waypoint_k < len(self.waypoint_list) - 1:
                    goal = LosPathFollowingGoal()
                    rospy.loginfo("define goal to send to los_guidance_node")

                    goal.waypoints[0].x = self.waypoint_list[index_waypoint_k][0]
                    goal.waypoints[0].y = self.waypoint_list[index_waypoint_k][1]
                    goal.waypoints[1].x = self.waypoint_list[index_waypoint_k + 1][0]
                    goal.waypoints[1].y = self.waypoint_list[index_waypoint_k + 1][1]
                    rospy.loginfo("add waypoints to goal")
                    rospy.loginfo(
                        "current points are: \n("
                        + str(goal.waypoints[0].x)
                        + ","
                        + str(goal.waypoints[0].y)
                        + ")\n"
                        + "("
                        + str(goal.waypoints[1].x)
                        + ","
                        + str(goal.waypoints[1].y)
                        + ")"
                    )

                    self.action_client.send_goal(goal)
                    rospy.loginfo("send goal to los_guidance_node")
                    self.action_client.wait_for_result()
                    rospy.loginfo("receive result from los_guidance_node")
                    index_waypoint_k += 1
                else:
                    self.waypoint_list.clear()
                    self.path.poses.clear()
                    self.path_pub.publish(self.path)
                    rospy.loginfo("clear waypoint_list")


if __name__ == "__main__":
    try:
        waypoint_manager = WaypointManager()

        waypoint_manager.spin()
    except rospy.ROSInterruptException:
        pass
