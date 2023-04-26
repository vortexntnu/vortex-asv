#!/usr/bin/python3
# written by Ronja Kræmer, student
# edited by Sigurd von Brandis, student

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from vortex_msgs.srv import Waypoint, WaypointRequest, WaypointResponse


class LosClient:
    """
    Nodes created:
        los_client

    Subscribes to:

    Publishes to:

    """

    def __init__(self):
        rospy.init_node("LosClient")

    def send_wp(self, waypoint_in):
        wp = WaypointRequest()
        wp.waypoint = waypoint_in
        response = WaypointResponse()
        response.success = False
        try:
            rospy.loginfo("Starting wait for service")
            rospy.wait_for_service("/navigation/add_waypoint")
            rospy.loginfo("Service server found")
            waypoint_client = rospy.ServiceProxy("/navigation/add_waypoint",
                                                 Waypoint)
            response = waypoint_client(wp)
        except rospy.ServiceException as e:
            print("Service call failed: {}".format(e))

        if response.success:
            rospy.loginfo(f"Waypoint {wp.waypoint} sent successfully!")
        else:
            rospy.logwarn(f"Waypoint {wp.waypoint} could not be set!")

    def overwrite_with_new_waypoint(self, waypoint_in):
        wp = WaypointRequest()
        wp.waypoint = waypoint_in
        response = WaypointResponse()
        response.success = False
        try:
            rospy.loginfo("Starting wait for service")
            rospy.wait_for_service(
                "/navigation/overwrite_waypoint_list_with_new_waypoint")
            rospy.loginfo("Service server found")
            overwrite_waypoint_client = rospy.ServiceProxy(
                "/navigation/overwrite_waypoint_list_with_new_waypoint",
                Waypoint)
            response = overwrite_waypoint_client(wp)
        except rospy.ServiceException as e:
            print("Service call failed: {}".format(e))

        if response.success:
            rospy.loginfo(f"Waypoint {wp.waypoint} sent successfully!")
        else:
            rospy.logwarn(f"Waypoint {wp.waypoint} could not be set!")


if __name__ == "__main__":
    try:
        los_client = LosClient()
        los_client.send_wp([1, 1])
        los_client.send_wp([5, 2])
        los_client.send_wp([5, 3])
        los_client.overwrite_with_new_waypoint([6, 3])
        #los_client.send_wp([5, 5])
        #los_client.send_wp([0, 5])
        # los_client.send_wp([0, 0])
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
