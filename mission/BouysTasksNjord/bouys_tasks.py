#!/usr/bin/python3

import rospy
import smach
import smach_ros
import math

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
        next_waypoint = NavAroundOneObject(self.data.vessel_position, self.data.current_red_bouy)
        #Feed next waypoint to LOS ...
        return 'desideNextState'


#Can be done in a simmilar way as OneRedBouyNav.
class OneGreenBouyNav(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['desideNextState'], input_keys=['userdata'])

    def execute(self, userdata):
        rospy.loginfo('OneGreenBouyNav')
        pass

class GreenAndReadBouyNav(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['desideNextState'], input_keys=['userdata'])

    def execute(self, userdata):
        rospy.loginfo('GreenAndReadBouyNav')
        pass

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


def NavAroundOneObject(ASVPos, objectPos): #Does not currently deside if it sould pass at right or left side of a bouy based on what type of bouy we navigate around.
    #Calculate the angle between the ASV and the object.
    angle = math.atan2(objectPos[1] - ASVPos[1], objectPos[0] - ASVPos[0])  # atan2(y - y_self ,x - x_self)

    #Calculate the new point on the circle 
    new_x = objectPos[0] + 2 * math.cos(angle)
    new_y = objectPos[1] + 2 * math.sin(angle)

    return (new_x, new_y)
