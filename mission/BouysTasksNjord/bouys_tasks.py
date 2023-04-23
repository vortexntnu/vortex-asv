#!/usr/bin/python3

import rospy
import smach
import math
from vortex_msgs.srv import Waypoint, WaypointRequest, WaypointResponse

class Idle(smach.State):
    def __init__(self, data):
        smach.State.__init__(self, outcomes=['desideNextState', 'search', 'stop'])
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
        self.data = data

    def execute(self):
        rospy.loginfo('Executing Search')
        #Copy search pattern from AUV?
        return 'idle'
    

class DesideNextState(smach.State):
    def __init__(self, data):
        self.data = data

    def execute(self):
        rospy.loginfo('Finding next state')

        if self.data.closest_object[1] == 'red' and self.data.second_closest_object[1] == 'green':
            return 'greenAndRedBouyNav'
        if self.data.closest_object[1] == 'green' and self.data.second_closest_object[1] == 'red':
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

        next_waypoint = NavAroundOneObject(self.data.vessel_position, self.data.current_red_bouy, self.data.DistanceRadius, self.data.DirectionWithLeia)
        
        send_wp(self.data.vessel_position)
        overwrite_with_new_waypoint(next_waypoint)
        
        return 'desideNextState'

class OneGreenBouyNav(smach.State):
    def __init__(self, data):
        smach.State.__init__(self, outcomes=['desideNextState'])
        self.data = data

    def execute(self):
        rospy.loginfo('OneGreenBouyNav')

        next_waypoint = NavAroundOneObject(self.data.vessel_position, self.data.current_green_bouy, self.data.DistanceRadius, self.data.DirectionWithLeia)
        
        send_wp(self.data.vessel_position)
        overwrite_with_new_waypoint(next_waypoint)

        return 'desideNextState'

class GreenAndReadBouyNav(smach.State):
    def __init__(self, data):
        smach.State.__init__(self, outcomes=['desideNextState'], input_keys=['userdata'])
        self.data = data

    def execute(self):
        rospy.loginfo('GreenAndReadBouyNav')
        GreenBouy = self.data.current_green_bouy
        RedBouy = self.data.current_red_bouy
        xWP = 0
        yWP = 0

        distance_between_bouys = math.sqrt((GreenBouy[0] - RedBouy[0]) ** 2 + (GreenBouy[1] - RedBouy[1]) ** 2)

        #Calculate a new waypoint between bouys that is 10% of center based on direction.
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
        smach.State.__init__(self, outcomes=['desideNextState'], input_keys=['userdata'])

    def execute(self, userdata):
        rospy.loginfo('NorthMarkerNav')
        pass

class SouthMarkerNav(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['desideNextState'], input_keys=['userdata'])

    def execute(self, userdata):
        rospy.loginfo('NorthMarkerNav')
        pass

class WestMarkerNav(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['desideNextState'], input_keys=['userdata'])

    def execute(self, userdata):
        rospy.loginfo('NorthMarkerNav')
        pass

class EastMarkerNav(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['desideNextState'], input_keys=['userdata'])

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
        xAC = ASVPos[0] - object[0] #xB - xA
        yAC = ASVPos[1] - object[1] #yB - yA
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
        xAC = ASVPos[0] - object[0] #xB - xA
        yAC = ASVPos[1] - object[1] #yB - yA
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
        waypoint_client = rospy.ServiceProxy("/navigation/add_waypoint", Waypoint)
        response = waypoint_client(wp)
    except rospy.ServiceException as e:
        print("Service call failed: {}".format(e))
    if response.success:
        rospy.loginfo(f"Waypoint {wp.waypoint} sent successfully!")
    else:
        rospy.logwarn(f"Waypoint {wp.waypoint} could not be set!")

def overwrite_with_new_waypoint(waypoint_in):
        wp= WaypointRequest()
        wp.waypoint = waypoint_in
        response = WaypointResponse()
        response.success = False
        try:
            rospy.wait_for_service("/navigation/overwrite_waypoint_list_with_new_waypoint")
            overwrite_waypoint_client = rospy.ServiceProxy("/navigation/overwrite_waypoint_list_with_new_waypoint", Waypoint)
            response = overwrite_waypoint_client(wp)
        except rospy.ServiceException as e:
            print("Service call failed: {}".format(e))

        if response.success:
            rospy.loginfo(f"Waypoint {wp.waypoint} sent successfully!")
        else:
            rospy.logwarn(f"Waypoint {wp.waypoint} could not be set!")