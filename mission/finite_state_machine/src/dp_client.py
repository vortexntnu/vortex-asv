#!/usr/bin/python3
# written by Viktor Naas and Ronja Kræmer

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from rospy.core import rospydebug
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from vortex_msgs.srv import ControlMode, ControlModeRequest, ControlModeResponse

# ENUM
OPEN_LOOP = 0
POSITION_HOLD = 1
HEADING_HOLD = 2
POSE_HOLD = 3
CONTROL_MODE_END = 4


def change_control_mode_client(requested_mode):

    rospy.loginfo("Waiting for server")
    rospy.wait_for_service("/controllers/control_mode_service")
    rospy.loginfo("server available")
    try:
        control_mode = rospy.ServiceProxy("/controllers/control_mode_service",
                                          ControlMode)
        response = control_mode(requested_mode)
        return True
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def dp_move_base(x, y, yaw):

    # action client
    action_client = actionlib.SimpleActionClient("/controllers/move_base",
                                                 MoveBaseAction)
    action_client.wait_for_server()
    rospy.loginfo("MoveBaseAction client made.\n")
    goal = MoveBaseGoal()
    rospy.loginfo("Created MoveBaseGoal")

    # create goal
    goal.target_pose.pose.position = Point(x, y, 0)
    rospy.loginfo("Set x,y coordinate")
    goal.target_pose.pose.orientation = Quaternion(
        *quaternion_from_euler(0, 0, yaw))
    rospy.loginfo("Set yaw")
    action_client.send_goal(goal)
    # wait for server to finish action
    rospy.loginfo("waiting for result")
    action_client.wait_for_result()
    rospy.loginfo("action complete!")
    return action_client.get_result()


if __name__ == "__main__":
    try:
        rospy.init_node("dp_move_base")
        change_control_mode_client(POSITION_HOLD)
        dp_move_base(2.0, 1.0, -1.0)
    except rospy.ROSInternalException as e:
        rospy.logerr(e)
