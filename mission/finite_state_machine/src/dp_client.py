#!/usr/bin/python3
#written by Viktor Naas and Ronja Kræmer

import rospy
import actionlib
from geometry_msgs import Pose2D
from rospy.core import rospydebug
from vortex_msgs.msg import (
    DpMoveBaseAction,
    DpMoveBaseGoal,
    DpMoveBaseResult,
    DpMoveBaseFeedback
)
from vortex_msgs.srv import ControlMode, ControlModeRequest, ControlModeResponse

#ENUM
OPEN_LOOP           = 0
POSE_HOLD           = 1
HEADING_HOLD        = 2
CONTROL_MODE_END    = 3


def change_control_mode_client(requested_mode):

    rospy.wait_for_service('controlmode_service')
    try:
        control_mode = rospy.ServiceProxy("controlmode_service", ControlMode)
        response = control_mode(requested_mode)
        return response.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def dp_move_base(x, y, yaw, controlMode):
    #action client
    action_client = actionlib.SimpleActionClient("move_base", DpMoveBaseAction)
    action_client.wait_for_server()
    goal = DpMoveBaseGoal()

    goal.dpBase.x = x
    goal.dpBase.y = y
    goal.dpBase.theta = yaw
    action_client.send_goal(goal)
    
    action_client.wait_for_result()

    #service
    control_mode_service = rospy.Service(
        "set_control_mode", ControlMode, change_control_mode_client
    )

