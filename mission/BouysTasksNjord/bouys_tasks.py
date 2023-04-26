#!/usr/bin/python3
#Written by Sigurd von Brandis, Student

import rospy
import smach
import math
import numpy as np
from vortex_msgs.srv import Waypoint, WaypointRequest, WaypointResponse
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from tf.transformations import (
    euler_from_quaternion,
    quaternion_multiply,
)


class Idle(smach.State):

    def __init__(self, data):
        smach.State.__init__(self,
                             outcomes=['desideNextState', 'search', 'stop'])
        self.data = data

    def execute(self):
        rospy.loginfo('Executing Idle')
        if self.data.ObjectSearchAttempts == 5:
            return 'stop'
        elif self.data.closest_object[1] == '':
            self.data.ObjectSearchAttempts += 1
            return 'search'
        else:
            self.data.ObjectSearchAttempts = 0
            return 'desideNextState'


class Search(smach.State):

    def __init__(self, data):
        smach.State.__init__(self, outcomes=['idle'])
        self.odom = Odometry()
        self.rate = rospy.Rate(10)
        self.data = data

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)

    # TODO: This is an absolute mess, with code directly copied from AUV repo. Fix ASAP

    def odom_cb(self, msg):
        self.odom = msg

    def rotate_certain_angle(pose, angle):
        """Angle in degrees"""

        orientation = R.from_quat([
            pose.orientation.x, pose.orientation.y, pose.orientation.z,
            pose.orientation.w
        ])
        rotation = R.from_rotvec(angle * math.pi / 180 * np.array([0, 0, 1]))
        new_orientation = R.as_quat(orientation * rotation)
        new_pose = Pose()
        new_pose.position = pose.position
        new_pose.orientation.x = new_orientation[0]
        new_pose.orientation.y = new_orientation[1]
        new_pose.orientation.z = new_orientation[2]
        new_pose.orientation.w = new_orientation[3]

        return new_pose

    def within_acceptance_margins(setpoint, odom_msg):
        acceptance_margins = [100, 100, 100, 100, 100, 0.1]
        acceptance_velocities = [100, 100, 100, 100, 100, 100]

        current_pose = odom_msg.pose.pose

        current_velocity_list = [
            odom_msg.twist.twist.linear.x,
            odom_msg.twist.twist.linear.y,
            odom_msg.twist.twist.linear.z,
            odom_msg.twist.twist.angular.x,
            odom_msg.twist.twist.angular.y,
            odom_msg.twist.twist.angular.z,
        ]

        # create quats from msg
        goal_quat_list = [
            setpoint.orientation.x,
            setpoint.orientation.y,
            setpoint.orientation.z,
            -setpoint.orientation.w,  # invert goal quat
        ]
        current_quat_list = [
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w,
        ]
        q_r = quaternion_multiply(current_quat_list, goal_quat_list)

        # convert relative quat to euler
        (roll_diff, pitch_diff, yaw_diff) = euler_from_quaternion(q_r)

        # check if close to goal
        diff_list = [
            abs(setpoint.position.x - current_pose.position.x),
            abs(setpoint.position.y - current_pose.position.y),
            abs(setpoint.position.z - current_pose.position.z),
            abs(roll_diff),
            abs(pitch_diff),
            abs(yaw_diff),
        ]
        is_close = True
        for i in range(len(diff_list)):
            if diff_list[i] > acceptance_margins[i]:
                is_close = False
            elif current_velocity_list[i] > acceptance_velocities[i]:
                is_close = False

        return is_close

    def yaw_to_angle(self, angle):
        goal = Pose()
        goal.position = self.odom.pose.pose.position
        goal.orientation = self.odom.pose.pose.orientation
        goal = self.rotate_certain_angle(goal, angle)

        vel_goal = Twist()
        vel_goal.angular.z = np.sign(angle) * rospy.get_param(
            "/fsm/turn_speed")
        vel_goal.linear.z = -0.01
        vel_goal.linear.x = 0.01

        self.velocity_ctrl_client(vel_goal, True)
        print(f"Searching for {self.task}, angle: ({angle}) ...")
        while not self.within_acceptance_margins(goal, self.odom):
            self.object = self.landmarks_client(self.task).object
            if self.object.isDetected:
                return True
            self.rate.sleep()

        self.velocity_ctrl_client(vel_goal, False)

        return False

    def execute(self):
        rospy.loginfo('Executing Search')
        path_segment_counter = 1

        while not self.object.isDetected:
            while (self.vtf_client.simple_state
                   != actionlib.simple_action_client.SimpleGoalState.DONE):
                if self.object.isDetected:
                    break
                self.rate.sleep()

            detection = self.yaw_to_angle(45)
            if detection:
                break

            detection = self.yaw_to_angle(-90)
            if detection:
                break

            detection = self.yaw_to_angle(45)
            if detection:
                break

            path_segment_counter += 1

        #Copy search pattern from AUV?
        return 'idle'


class DesideNextState(smach.State):

    def __init__(self, data):
        self.data = data

    def execute(self):
        rospy.loginfo('Finding next state')

        if self.data.closest_object[
                1] == 'red' and self.data.second_closest_object[1] == 'green':
            return 'greenAndRedBouyNav'
        if self.data.closest_object[
                1] == 'green' and self.data.second_closest_object[1] == 'red':
            return 'greenAndRedBouyNav'
        if self.data.closest_object[1] == 'red':
            return 'red'
        if self.data.closest_object[1] == 'green':
            return 'green'
        if self.data.closest_object[1] == 'north':
            return 'north'
        if self.data.closest_object[1] == 'south':
            return 'south'
        if self.data.closest_object[1] == 'east':
            return 'east'
        if self.data.closest_object[1] == 'west':
            return 'west'
        if self.data.closest_object[1] == '':
            return 'search'


class OneRedBouyNav(smach.State):

    def __init__(self, data):
        smach.State.__init__(self, outcomes=['desideNextState'])
        self.data = data

    def execute(self):
        rospy.loginfo('OneRedBouyNav')

        next_waypoint = NavAroundOneObject(self.data.vessel_position,
                                           self.data.current_red_bouy,
                                           self.data.DistanceRadius,
                                           self.data.DirectionWithLeia)
        send_wp(self.data.vessel_position)
        overwrite_with_new_waypoint(next_waypoint)

        return 'desideNextState'


class OneGreenBouyNav(smach.State):

    def __init__(self, data):
        smach.State.__init__(self, outcomes=['desideNextState'])
        self.data = data

    def execute(self):
        rospy.loginfo('OneGreenBouyNav')

        next_waypoint = NavAroundOneObject(self.data.vessel_position,
                                           self.data.current_green_bouy,
                                           self.data.DistanceRadius,
                                           self.data.DirectionWithLeia)
        send_wp(self.data.vessel_position)
        overwrite_with_new_waypoint(next_waypoint)

        return 'desideNextState'


class GreenAndReadBouyNav(smach.State):

    def __init__(self, data):
        smach.State.__init__(self,
                             outcomes=['desideNextState'],
                             input_keys=['userdata'])
        self.data = data

    def execute(self):
        rospy.loginfo('GreenAndReadBouyNav')
        GreenBouy = self.data.current_green_bouy
        RedBouy = self.data.current_red_bouy
        xWP = 0
        yWP = 0

        distance_between_bouys = math.sqrt((GreenBouy[0] - RedBouy[0])**2 +
                                           (GreenBouy[1] - RedBouy[1])**2)

        if self.data.DirectionWithLeia == True:
            #Distance from Green to WP
            distance_between_Green_and_WP = 0.1 * distance_between_bouys
            # Calculate the coordinates of point WP
            xWP = 0.9 * RedBouy[0] + 0.1 * GreenBouy[0]
            yWP = 0.9 * RedBouy[1] + 0.1 * GreenBouy[1]
        else:
            # Distance from Red to WP
            distance_between_Red_and_WP = 0.1 * distance_between_bouys
            # Calculate the coordinates of point WP
            xWP = 0.9 * GreenBouy[0] + 0.1 * RedBouy[0]
            yWP = 0.9 * GreenBouy[1] + 0.1 * RedBouy[1]

        next_waypoint = (xWP, yWP)
        send_wp(self.data.vessel_position)
        overwrite_with_new_waypoint(next_waypoint)

        return 'desideNextState'


class NorthMarkerNav(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['desideNextState'],
                             input_keys=['userdata'])

    def execute(self, userdata):
        rospy.loginfo('NorthMarkerNav')
        pass


class SouthMarkerNav(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['desideNextState'],
                             input_keys=['userdata'])

    def execute(self, userdata):
        rospy.loginfo('NorthMarkerNav')
        pass


class WestMarkerNav(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['desideNextState'],
                             input_keys=['userdata'])

    def execute(self, userdata):
        rospy.loginfo('NorthMarkerNav')
        pass


class EastMarkerNav(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['desideNextState'],
                             input_keys=['userdata'])

    def execute(self, userdata):
        rospy.loginfo('NorthMarkerNav')
        pass


def NavAroundOneObject(ASVPos, object, radius, directionWithLeia):
    """
        object (B)
               /|
              / |
             /  | radius
            /   |
           /    |
          /     |
    ASV (A)----(C) WP

    """
    objectType = object[2]
    xWP = 0
    yWP = 0

    if directionWithLeia == True:
        # Calculate the coordinates of next waypoint.
        xAC = ASVPos[0] - object[0]  #xB - xA
        yAC = ASVPos[1] - object[1]  #yB - yA
        AC_length = math.sqrt(xAC**2 + yAC**2)
        xAC_normalized = xAC / AC_length
        yAC_normalized = yAC / AC_length
        if objectType == 'red' or objectType == 'west' or objectType == 'south':
            # Adjust the sign of the vector AC for a reflex angle
            if xAC_normalized > 0:
                xAC_normalized *= -1
            if yAC_normalized > 0:
                yAC_normalized *= -1
        xWP = object[0] + radius * xAC_normalized
        yWP = object[1] + radius * yAC_normalized
    else:
        # Calculate the coordinates of next waypoint.
        xAC = ASVPos[0] - object[0]  #xB - xA
        yAC = ASVPos[1] - object[1]  #yB - yA
        AC_length = math.sqrt(xAC**2 + yAC**2)
        xAC_normalized = xAC / AC_length
        yAC_normalized = yAC / AC_length
        if objectType == 'green' or objectType == 'east' or objectType == 'north':
            # Adjust the sign of the vector AC for a reflex angle
            if xAC_normalized > 0:
                xAC_normalized *= -1
            if yAC_normalized > 0:
                yAC_normalized *= -1
        xWP = object[0] + radius * xAC_normalized
        yWP = object[1] + radius * yAC_normalized

    return (xWP, yWP)


def send_wp(waypoint_in):
    wp = WaypointRequest()
    wp.waypoint = waypoint_in
    response = WaypointResponse()
    response.success = False
    try:
        rospy.wait_for_service("/navigation/add_waypoint")
        waypoint_client = rospy.ServiceProxy("/navigation/add_waypoint",
                                             Waypoint)
        response = waypoint_client(wp)
    except rospy.ServiceException as e:
        print("Service call failed: {}".format(e))
    if response.success:
        rospy.loginfo(f"Waypoint {wp.waypoint} sent successfully!")
    else:
        rospy.logwarn(f"Waypoint {wp.waypoint} could not be set!")


def overwrite_with_new_waypoint(waypoint_in):
    wp = WaypointRequest()
    wp.waypoint = waypoint_in
    response = WaypointResponse()
    response.success = False
    try:
        rospy.wait_for_service(
            "/navigation/overwrite_waypoint_list_with_new_waypoint")
        overwrite_waypoint_client = rospy.ServiceProxy(
            "/navigation/overwrite_waypoint_list_with_new_waypoint", Waypoint)
        response = overwrite_waypoint_client(wp)
    except rospy.ServiceException as e:
        print("Service call failed: {}".format(e))

    if response.success:
        rospy.loginfo(f"Waypoint {wp.waypoint} sent successfully!")
    else:
        rospy.logwarn(f"Waypoint {wp.waypoint} could not be set!")
