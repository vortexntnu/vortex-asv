import numpy as np

from transforms3d.euler import euler2quat, quat2euler
from nav_msgs.msg import Odometry


def odometrymsg_to_state(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    orientation_list = [
        orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z
    ]

    # Convert quaternion to Euler angles
    (roll, pitch, yaw) = quat2euler(orientation_list)

    vx = msg.twist.twist.linear.x
    vy = msg.twist.twist.linear.y
    vyaw = msg.twist.twist.angular.z

    state = np.array([x, y, yaw, vx, vy, vyaw])
    return state

def state_to_odometrymsg(state):
    orientation_list_next = euler2quat(0, 0, state[2])
    
    odometry_msg = Odometry()
    odometry_msg.pose.pose.position.x = state[0]
    odometry_msg.pose.pose.position.y = state[1]
    odometry_msg.pose.pose.position.z = 0.0
    odometry_msg.pose.pose.orientation.w = orientation_list_next[0]
    odometry_msg.pose.pose.orientation.x = orientation_list_next[1]
    odometry_msg.pose.pose.orientation.y = orientation_list_next[2]
    odometry_msg.pose.pose.orientation.z = orientation_list_next[3]
    
    return odometry_msg

def Rot(psi):
    R = np.array([[np.cos(psi), -np.sin(psi), 0],
                [np.sin(psi), np.cos(psi), 0],
                [0, 0, 1]])
    R_T = np.transpose(R)
    return R, R_T