import numpy as np

from transforms3d.euler import euler2quat, quat2euler
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


def odometrymsg_to_state(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    orientation_list = [
        orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z
    ]

    # Convert quaternion to Euler angles, extract yaw
    yaw = quat2euler(orientation_list)[2]

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

def state_to_posestamped(state, frame_id, stamp):
    orientation_list_next = euler2quat(0, 0, state[2])

    posestamped_msg = PoseStamped()
    
    posestamped_msg.header.frame_id = frame_id
    posestamped_msg.header.stamp = stamp
    
    posestamped_msg.pose.position.x = state[0]
    posestamped_msg.pose.position.y = state[1]
    posestamped_msg.pose.position.z = 0.0
    posestamped_msg.pose.orientation.w = orientation_list_next[0]
    posestamped_msg.pose.orientation.x = orientation_list_next[1]
    posestamped_msg.pose.orientation.y = orientation_list_next[2]
    posestamped_msg.pose.orientation.z = orientation_list_next[3]
    
    return posestamped_msg

def ssa(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi