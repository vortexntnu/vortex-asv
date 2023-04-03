#!/usr/bin/python3

import rospy
import smach
import smach_ros
import math

from dataclasses import dataclass



## ToDo
# Delete self.Current"objects" that we do not want or need anymore. Ex; objects we have or are currently passing.
# This sould be done in individual states, not in search, or by the info given from perseption. 

class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['search', 'stop'])
        self.NoGoSircleRadius = 2 #Meters. Used to define area around bouy that ASV must absolutely NOT enter. 
        self.CircleRadius = 4 #Meters. Used to define curve ASV can follow when it only knows one bouy.
        self.DirectionWithLeia = True #Used to descide which side the ASV should be regarding Green and Read "Staker".
        self.ObjectSearchAttempts = 0

    def execute(self):
        # while not rospy.is_shutdown():
        #     self.enabled = rospy.get_param("/tasks/maneuvering_navigation_tasks")

        #     if self.enabled:
        rospy.loginfo('Executing Idle')
        if self.ObjectSearchAttempts == 5:
            return 'stop'
        else:
            return 'search'

class Search(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['oneRedBouyNav',
                                             'oneGreenBouyNav',
                                             'greenAndReadBouyNav',
                                             'northMarkerNav',
                                             'southMarkerNav',
                                             'eastMarkerNav',
                                             'westMarkerNav',
                                             'idle'],
                             input_keys=['userdata'])
        self.info = Idle()

    def execute(self, userdata):
        # while not rospy.is_shutdown():
        #     self.enabled = rospy.get_param("/tasks/maneuvering_navigation_tasks")

        #     if self.enabled:
        rospy.loginfo('Executing Search')

        #Copy search pattern from AUV

        #Update ClosestObject and SecondClosestObject in userdata
        userdata = process_new_objects(userdata)

        #Choosing closest point for navigational behaviour/next state
        if userdata[7][0] == 'red' and userdata[8][0] == 'green':
            return 'greenAndRedBouyNav'
        if userdata[7][0] == 'green' and userdata[8,0] == 'red':
            return 'greenAndRedBouyNav'
        if userdata[7][0] == 'red':
            return 'red'
        if userdata[7][0] == 'green':
            return 'green'
        if userdata[7][0] == 'north':
            return 'north'
        if userdata[7][0] == 'south':
            return 'south'
        if userdata[7][0] == 'east':
            return 'east'
        if userdata[7][0] == 'west':
            return 'west'
        if userdata[7][0] == '':
            self.ObjectSearchAttempts  = self.ObjectSearchAttempts + 1
            return 'idle'
    
class OneRedBouyNav(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['search'], input_keys=['userdata'])
        self.treshhold = 1 #meter
        self.Distance = 100 #meter. Just some lagre init value.
        self.info = Idle() #Must be switched out. This only copies class, not new data that we need. 

    def execute(self, userdata):
        # while not rospy.is_shutdown():
        #     self.enabled = rospy.get_param("/tasks/maneuvering_navigation_tasks")

        #     if self.enabled:
        rospy.loginfo('OneRedBouyNav')

        if self.Distance > self.treshhold:

            #userdata.ASVPos = (0,0) #Update asv position
            self.Distance = Distance(userdata.ASV, userdata.ClosestObject)

            next_waypoint = NavAroundOneObject(userdata.ASVPos, userdata.CurrentRedBouy)

            #Feed next waypoint to LOS ...

            return 'search'
        else:
            return 'search'  #Change out with emergency state or something


#Can be done in a simmilar way as OneRedBouyNav.
class OneGreenBouyNav(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['search'], input_keys=['userdata'])

    def execute(self, userdata):
        # while not rospy.is_shutdown():
        #     self.enabled = rospy.get_param("/tasks/maneuvering_navigation_tasks")

        #     if self.enabled:
        rospy.loginfo('OneGreenBouyNav')
        pass

class GreenAndReadBouyNav(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['search'], input_keys=['userdata'])

    def execute(self, userdata):
        # while not rospy.is_shutdown():
        #     self.enabled = rospy.get_param("/tasks/maneuvering_navigation_tasks")

        #     if self.enabled:
        rospy.loginfo('GreenAndReadBouyNav')
        pass

class NorthMarkerNav(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['search'], input_keys=['userdata'])

    def execute(self, userdata):
        # while not rospy.is_shutdown():
        #     self.enabled = rospy.get_param("/tasks/maneuvering_navigation_tasks")

        #     if self.enabled:
        rospy.loginfo('NorthMarkerNav')
        pass

class SouthMarkerNav(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['search'], input_keys=['userdata'])

    def execute(self, userdata):
        # while not rospy.is_shutdown():
        #     self.enabled = rospy.get_param("/tasks/maneuvering_navigation_tasks")

        #     if self.enabled:
        rospy.loginfo('NorthMarkerNav')
        pass

class WestMarkerNav(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['search'], input_keys=['userdata'])

    def execute(self, userdata):
        # while not rospy.is_shutdown():
        #     self.enabled = rospy.get_param("/tasks/maneuvering_navigation_tasks")

        #     if self.enabled:
        rospy.loginfo('NorthMarkerNav')
        pass

class EastMarkerNav(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['search'], input_keys=['userdata'])

    def execute(self, userdata):
        # while not rospy.is_shutdown():
        #     self.enabled = rospy.get_param("/tasks/maneuvering_navigation_tasks")

        #     if self.enabled:
        rospy.loginfo('NorthMarkerNav')
        pass


def Distance(ObjectOnePosition, ObjectTwoPosition):
    x1, y1 = ObjectOnePosition
    x2, y2 = ObjectTwoPosition
    distanceX = x1 - x2
    distanceY = y1 - y2
    hypDistance = math.sqrt(distanceX**2 + distanceY**2)
    return hypDistance


def process_new_objects(self, userdata):
    ASVPos = userdata[0]
    OldClosestObject = userdata[7]
    OldSecondClosestObject = userdata[8]

    for i in range(1, 6): #Current object positions in userdata
        new_object = userdata[i]
        object_type = new_object[2]

        # Store the object position and update the closest object
        new_object_pos = (new_object[0], new_object[1])
        distance_to_new_object = Distance(ASVPos, new_object_pos)
        distance_to_OldClosestObject = Distance(ASVPos, OldClosestObject)
        distance_to_OldSecondClosestObject = Distance(ASVPos, OldSecondClosestObject)

        if distance_to_new_object < distance_to_OldClosestObject:
            userdata.SecondClosestObject = OldClosestObject
            userdata.ClosestObject = (distance_to_new_object, object_type)
        
        elif distance_to_new_object < distance_to_OldSecondClosestObject:
            userdata.SecondClosestObject = (distance_to_new_object, object_type)
            userdata.ClosestObject = OldClosestObject
        
        return userdata



def NavAroundOneObject(ASVPos, objectPos): #Does not currently deside if it sould pass at right or left side of a bouy based on what type of bouy we navigate around.
    #Calculate the angle between the ASV and the object.
    angle = math.atan2(objectPos[1] - ASVPos[1], objectPos[0] - ASVPos[0])  # atan2(y - y_self ,x - x_self)

    #Calculate the new point on the circle 
    new_x = objectPos[0] + 2 * math.cos(angle)
    new_y = objectPos[1] + 2 * math.sin(angle)

    return (new_x, new_y)
