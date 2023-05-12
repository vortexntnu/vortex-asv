#!/usr/bin/python3
import rospy
from nav_msgs.msg import Odometry
import actionlib
from vortex_msgs.srv import Waypoint, WaypointRequest, WaypointResponse


class ColavTask:

    def __init__(self):
        rospy.init_node("colav_fsm", anonymous=False)
        self.vessel = Odometry()
        self.goal = WaypointRequest()
        self.server = actionlib.SimpleActionServer('StartColavTask',
                                                   Odometry,
                                                   self.goal_cb,
                                                   auto_start=True)
        self.vessel_sub = rospy.Subscriber("/pose_gt",
                                           Odometry,
                                           self.vessel_callback,
                                           queue_size=1)  # 20hz

        self.enabled = rospy.get_param("/tasks/task_1")

    def vessel_callback(self, data):
        """
        Update the postion of the ASV
        Args:
            data: An odometry of the ASV position
        """
        self.vessel = data

    def goal_cb(self, data):
        if self.enabled:
            goal_wp = WaypointRequest()
            goal_wp.waypoint = [
                data.pose.pose.position.x, data.pose.pose.position.y
            ]

            vessel_wp = WaypointRequest()
            vessel_wp.waypoint = [
                self.vessel.pose.pose.position.x,
                self.vessel.pose.pose.position.y
            ]
            response = WaypointResponse()
            response.success = False
            try:
                rospy.loginfo("Sending waypoints")
                rospy.wait_for_service("/navigation/add_waypoint")
                waypoint_client = rospy.ServiceProxy(
                    "/navigation/add_waypoint", Waypoint)
                first_response = waypoint_client(vessel_wp)
                second_response = waypoint_client(goal_wp)
                response.success = first_response.success and second_response.success
            except rospy.ServiceException as e:
                rospy.loginfo("Service call failed: {}".format(e))
        else:
            rospy.loginfo("FAILURE!: Task is not enabled yet!")

    def spin(self):
        self.enabled = rospy.get_param("/tasks/task_1")

        while not self.enabled:
            rospy.sleep(0.1)


if __name__ == "__main__":
    my_node_1 = ColavTask()
    my_node_1.spin()
    rospy.spin()
