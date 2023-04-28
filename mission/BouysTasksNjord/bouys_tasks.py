#!/usr/bin/python3
#Written by Sigurd von Brandis, Student

import rospy
import smach
import math
from vortex_msgs.srv import Waypoint, WaypointRequest, WaypointResponse
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion


class Idle(smach.State):

    def __init__(self, data):
        smach.State.__init__(self,
                             outcomes=['desideNextState', 'search']) #, 'stop'
        self.data = data

    def execute(self):
        rospy.loginfo('Executing Idle')

        # if self.data.ObjectSearchAttempts == 5:
        #     return 'stop'
        if self.data.closest_object[1] == '':
            self.data.ObjectSearchAttempts += 1
            return 'search'
        else:
            self.data.ObjectSearchAttempts = 0
            return 'desideNextState'


class Search(smach.State):

    def __init__(self, data):
        smach.State.__init__(self, outcomes=['idle'])
        self.task = "Buoy"
        self.odom = Odometry()
        self.rate = rospy.Rate(10)
        self.data = data

        self.heading_pub = rospy.Publisher(
            "/guidance_interface/desired_heading", Float64, queue_size=1)

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)

        # TODO: some of these functions should be generalized, adn

    def odom_cb(self, msg):
        self.odom = msg

    def within_acceptance_margins(setpoint, current):
        error = abs(setpoint - current)
        if error < 0.1:
            return True
        return False

    def yaw_to_angle(self, angle):
        orientation = self.odom.pose.pose.orientation
        orientation_list = [
            orientation.x, orientation.y, orientation.z, orientation.w
        ]
        yaw = euler_from_quaternion(orientation_list)[2]
        heading_goal = yaw + angle

        self.heading_pub.Publish(heading_goal)
        print(f"Searching for {self.task}, angle: ({angle}) ...")
        while not self.within_acceptance_margins(heading_goal, yaw):
            self.rate.sleep()
            orientation = self.odom.pose.pose.orientation
            orientation_list = [
                orientation.x, orientation.y, orientation.z, orientation.w
            ]
            yaw = euler_from_quaternion(orientation_list)[2]

    def execute(self):
        rospy.loginfo('Executing Search')

        self.yaw_to_angle(45)
        self.yaw_to_angle(-90)
        self.yaw_to_angle(45)

        return 'idle'


class DesideNextState(smach.State):

    def __init__(self, data):
        smach.State.__init__(self, outcomes=['greenAndReadBouyNav',
                                              'red',
                                              'green',
                                              'north',
                                              'south',
                                              'east',
                                              'west',
                                              'idle'])
        self.data = data

    def execute(self):
        rospy.loginfo('DesideNextState')

        if self.data.closest_object[
                1] == 'red' and self.data.second_closest_object[1] == 'green':
            return 'greenAndRedBouyNav'
        elif self.data.closest_object[1] == 'green' and self.data.second_closest_object[
                1] == 'red':
            return 'greenAndRedBouyNav'
        elif self.data.closest_object[1] == 'red':
            return 'red'
        elif self.data.closest_object[1] == 'green':
            return 'green'
        elif self.data.closest_object[1] == 'north':
            return 'north'
        elif self.data.closest_object[1] == 'south':
            return 'south'
        elif self.data.closest_object[1] == 'east':
            return 'east'
        elif self.data.closest_object[1] == 'west':
            return 'west'
        else: #self.data.closest_object[1] == ''
            return 'idle'


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

    def __init__(self, data):
        smach.State.__init__(self,
                             outcomes=['desideNextState'],
                             )
        self.data = data

    def execute(self):
        rospy.loginfo('NorthMarkerNav')
        pass


class SouthMarkerNav(smach.State):

    def __init__(self, data):
        smach.State.__init__(self,
                             outcomes=['desideNextState'],
                             )
        self.data = data

    def execute(self):
        rospy.loginfo('SouthMarkerNav')
        pass


class WestMarkerNav(smach.State):

    def __init__(self, data):
        smach.State.__init__(self,
                             outcomes=['desideNextState'],
                             )
        self.data = data

    def execute(self):
        rospy.loginfo('EastMarkerNav')
        pass


class EastMarkerNav(smach.State):

    def __init__(self, data):
        smach.State.__init__(self,
                             outcomes=['desideNextState'],
                             )
        self.data = data

    def execute(self):
        rospy.loginfo('WestMarkerNav')
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
