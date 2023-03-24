#!/usr/bin/python3

import rospy
import smach
import smach_ros
import math

from dataclasses import dataclass


# @dataclass
# class ObjectPositions:
#     current_red_bouy: tuple = (0, 0)
#     current_green_buoy: tuple = (0, 0)
#     current_north_marker: tuple = (0, 0)
#     current_south_marker: tuple = (0, 0)
#     current_east_marker: tuple = (0, 0)
#     current_west_marker: tuple = (0, 0)
#     closest_objects: ...

## ToDo
# Delete self.Current"objects" that we do not want or need anymore. Ex; objects we have or are currently passing.
# This sould be done in individual states, not in search. 

class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['search', 'stop'])
        self.NoGoSircleRadius = 2 #Meters. Used to define area around bouy that ASV must absolutely NOT enter. 
        self.CircleRadius = 4 #Meters. Used to define curve ASV can follow when it only knows one bouy.
        self.DirectionWithLeia = True #Used to descide which side the ASV should be regarding Green and Read "Staker".
        self.ObjectSearchAttempts = 0

        #These values must be moved to a dataclass or something else!
        self.ASVPos = (0, 0) #Switch out with actual position.
        self.CurrentRedBouyPos     = (0,0)
        self.CurrentGreenBouyPos   = (0,0)
        self.CurrentNorthMarkerPos = (0,0)
        self.CurrentSouthMarkerPos = (0,0)
        self.CurrentEastMarkerPos  = (0,0)
        self.CurrentWestMarkerPos  = (0,0)
        self.ClosestObjects = [(0,'') for i in range(2)] #Distance and type of closest and second closest object.


    def execute(self, userdata):
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
        smach.State.__init__(self, outcomes=['oneRedBouyNav',
                                             'oneGreenBouyNav',
                                             'greenAndReadBouyNav',
                                             'northMarkerNav',
                                             'southMarkerNav',
                                             'eastMarkerNav',
                                             'westMarkerNav',
                                             'idle'
                                             ])
        self.info = Idle()

    def execute(self):
        # while not rospy.is_shutdown():
        #     self.enabled = rospy.get_param("/tasks/maneuvering_navigation_tasks")

        #     if self.enabled:
        rospy.loginfo('Executing Search')

        #Copy search pattern from AUV

        NewObject = (2,5,'red') #Switch out with actual discovered bouy from preseption algorithm. 
        
        process_new_object(NewObject)

        #Choosing closest point for navigational behaviour/next state
        if self.info.ClosestObjects[0][1] == 'red' and self.info.ClosestObjects[1][1] == 'green':
            return 'greenAndRedBouyNav'
        if self.info.ClosestObjects[0][1] == 'green' and self.info.ClosestObjects[1][1] == 'red':
            return 'greenAndRedBouyNav'
        if self.info.ClosestObjects[0][1] == 'red':
            return 'red'
        if self.info.ClosestObjects[0][1] == 'green':
            return 'green'
        if self.info.ClosestObjects[0][1] == 'north':
            return 'north'
        if self.info.ClosestObjects[0][1] == 'south':
            return 'south'
        if self.info.ClosestObjects[0][1] == 'east':
            return 'east'
        if self.info.ClosestObjects[0][1] == 'west':
            return 'west'
        if self.info.ClosestObjects[0][1] == '':
            self.ObjectSearchAttempts  = self.ObjectSearchAttempts + 1
            return 'idle'
    
class OneRedBouyNav(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['search'])
        self.treshhold = 1 #meter
        self.Distance = 100 #meter. Just some lagre init value.
        self.info = Idle() #Must be switched out. This only copies class, not new data that we need. 

    def execute(self):
        # while not rospy.is_shutdown():
        #     self.enabled = rospy.get_param("/tasks/maneuvering_navigation_tasks")

        #     if self.enabled:
        rospy.loginfo('OneRedBouyNav')

        while self.Distance > self.treshhold:

            self.info.ASVPos = (0,0) #Update asv position
            self.Distance = Distance(self.info.ASVPos - self.info.ClosestObjects[0])

            next_waypoint = NavAroundOneObject(self.info.CurrentRedBouyPos)

            #Feed next waypoint to LOS


#Can be done in a simmilar way as OneRedBouyNav.
class OneGreenBouyNav(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['search'])

    def execute(self):
        # while not rospy.is_shutdown():
        #     self.enabled = rospy.get_param("/tasks/maneuvering_navigation_tasks")

        #     if self.enabled:
        rospy.loginfo('OneGreenBouyNav')
        pass

class GreenAndReadBouyNav(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['search'])

    def execute(self):
        # while not rospy.is_shutdown():
        #     self.enabled = rospy.get_param("/tasks/maneuvering_navigation_tasks")

        #     if self.enabled:
        rospy.loginfo('GreenAndReadBouyNav')
        pass

class NorthMarkerNav(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['search'])

    def execute(self):
        # while not rospy.is_shutdown():
        #     self.enabled = rospy.get_param("/tasks/maneuvering_navigation_tasks")

        #     if self.enabled:
        rospy.loginfo('NorthMarkerNav')
        pass

class SouthMarkerNav(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['search'])

    def execute(self):
        # while not rospy.is_shutdown():
        #     self.enabled = rospy.get_param("/tasks/maneuvering_navigation_tasks")

        #     if self.enabled:
        rospy.loginfo('NorthMarkerNav')
        pass

class WestMarkerNav(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['search'])

    def execute(self):
        # while not rospy.is_shutdown():
        #     self.enabled = rospy.get_param("/tasks/maneuvering_navigation_tasks")

        #     if self.enabled:
        rospy.loginfo('NorthMarkerNav')
        pass

class EastMarkerNav(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['search'])

    def execute(self):
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


def process_new_object(self, new_object):
    # Define a dictionary to map object types to attributes
    type_to_attr = {
        'red': ('CurrentRedBouyPos', 'red'),
        'green': ('CurrentGreenBouyPos', 'green'),
        'north': ('CurrentNorthMarkerPos', 'north'),
        'south': ('CurrentSouthMarkerPos', 'south'),
        'east': ('CurrentEastMarkerPos', 'east'),
        'west': ('CurrentWestMarkerPos', 'west')
    }
    
    # Get the object type
    object_type = new_object[2]
    
    # Get the attribute names from the dictionary
    pos_attr, type_name = type_to_attr[object_type]
    
    # Store the object position and update the closest object
    new_object_pos = (new_object[0], new_object[1])
    distance_to_new_object = Distance(self.ASVPos, new_object_pos)
    distance_to_object = Distance(self.ASVPos, getattr(self, pos_attr))
    
    if distance_to_new_object < distance_to_object: # getattr(self, pos_attr) == (0, 0) or
        setattr(self, pos_attr, new_object_pos)
    
    if distance_to_object < self.ClosestObject[0][0]:
        self.ClosestObject[0] = (distance_to_object, type_name)
    elif distance_to_object < self.ClosestObject[1][0]:
        self.ClosestObject[1] = (distance_to_object, type_name)


def NavAroundOneObject(self, objectpos): #Does not currently deside if it sould pass at right or left side of a bouy based on what type of bouy we navigate around.
    #Calculate the angle between the ASV and the object.
    angle = math.atan2(objectpos[1] - self.ASVPos[1], objectpos[0] - self.ASVPos[0])  # atan2(y - y_self ,x - x_self)

    #Calculate the new point on the circle 
    new_x = objectpos[0] + 2 * math.cos(angle)
    new_y = objectpos[1] + 2 * math.sin(angle)

    self.x = new_x
    self.y = new_y

    return (new_x, new_y)

#Old code from search class
# if NewObject[2] == 'red':
        #     #Store Bouy in a list?
        #     NewObjectPos = (NewObject[0], NewObject[1])
        #     distanceToNewObject = Distance(self.ASVPos, NewObjectPos)
        #     distanceToObject = Distance(self.ASVPos, self.CurrentRedBouyPos)
        #     if self.CurrentRedBouyPos == (0,0) or distanceToNewObject < distanceToObject:
        #         self.CurrentRedBouyPos = NewObjectPos
        #     if distanceToObject < self.ClosestObject[0]:
        #         self.ClosestObject = (distanceToObject, 'red')
        
        # if NewObject[2] == 'green':
        #     #Store Bouy in a list?
        #     NewObjectPos = (NewObject[0], NewObject[1])
        #     distanceToNewObject = Distance(self.ASVPos, NewObjectPos)
        #     distanceToObject = Distance(self.ASVPos, self.CurrentGreenBouyPos)
        #     if self.CurrentGreenBouyPos == (0,0) or distanceToNewObject < distanceToObject:
        #         self.CurrentGreenBouy = NewObjectPos
        #     if distanceToObject < self.ClosestObject[0]:
        #         self.ClosestObject = (distanceToObject, 'green')
        
        # if NewObject[2] == 'north':
        #     #Store Bouy in a list?
        #     NewObjectPos = (NewObject[0], NewObject[1])
        #     distanceToNewObject = Distance(self.ASVPos, NewObjectPos)
        #     distanceToObject = Distance(self.ASVPos, self.CurrentNorthMarkerPos)
        #     if self.CurrentNorthMarkerPos == (0,0) or distanceToNewObject < distanceToObject:
        #         self.CurrentNorthMarkerPos = NewObjectPos
        #     if distanceToObject < self.ClosestObject[0]:
        #         self.ClosestObject = (distanceToObject, 'north')
        
        # if NewObject[2] == 'south':
        #     #Store Bouy in a list?
        #     NewObjectPos = (NewObject[0], NewObject[1])
        #     distanceToNewObject = Distance(self.ASVPos, NewObjectPos)
        #     distanceToObject = Distance(self.ASVPos, self.CurrentSouthMarkerPos)
        #     if self.CurrentSouthMarkerPos[2] == (0,0) or distanceToNewObject < distanceToObject:
        #         self.CurrentSouthMarkerPos = NewObjectPos
        #     if distanceToObject < self.ClosestObject[0]:
        #         self.ClosestObject = (distanceToObject, 'south')
        
        # if NewObject[2] == 'east':
        #     #Store Bouy in a list?
        #     NewObjectPos = (NewObject[0], NewObject[1])
        #     distanceToNewObject = Distance(self.ASVPos, NewObjectPos)
        #     distanceToObject = Distance(self.ASVPos, self.CurrentEastMarkerPos)
        #     if self.CurrentEastMarkerPos[2] == (0,0) or distanceToNewObject < distanceToObject:
        #         self.CurrentEastMarkerPos = NewObjectPos
        #     if distanceToObject < self.ClosestObject[0]:
        #         self.ClosestObject = (distanceToObject, 'east')
        
        # if NewObject[2] == 'west':
        #     #Store Bouy in a list?
        #     NewObjectPos = (NewObject[0], NewObject[1])
        #     distanceToNewObject = Distance(self.ASVPos, NewObjectPos)
        #     distanceToObject = Distance(self.ASVPos, self.CurrentWestMarkerPos)
        #     if self.CurrentWestMarkerPos[2] == (0,0) or distanceToNewObject < distanceToObject:
        #         self.CurrentWestMarkerPos = NewObjectPos
        #     if distanceToObject < self.ClosestObject[0]:
        #         self.ClosestObject = (distanceToObject, 'west')