#!/usr/bin/python3

import rospy
import smach
import smach_ros
import math

# Right now outcomex ,which means that the STATE MACHINE should stop, never happpens.

class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['search', 'stop'])
        self.NoGoSircleRadius = 2 #Meters. Used to define area around bouy that ASV must absolutely NOT enter. 
        self.CircleRadius = 4 #Meters. Used to define curve ASV can follow when it only knows one bouy.
        self.DirectionWithLeia = True #Used to descide which side the ASV should be regarding Green and Read "Staker".
        self.ObjectSearchAttempts = 0
        self.ASVPos = (0, 0) #Switch out with actual position.


    def execute(self, userdata):
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
        self.CurrentRedBouyPos     = (0,0)
        self.CurrentGreenBouyPos   = (0,0)
        self.CurrentNorthMarkerPos = (0,0)
        self.CurrentSouthMarkerPos = (0,0)
        self.CurrentEastMarkerPos  = (0,0)
        self.CurrentWestMarkerPos  = (0,0)
        self.ClosestObject = (0, '')

    def execute(self, userdata):
        rospy.loginfo('Executing Search')
        self.ObjectSearchAttempts  = self.ObjectSearchAttempts + 1
        #Copy search pattern from AUV

        NewObject = (2,5,'red') #Switch out with actual discovered bouy from search
        
        if NewObject[2] == 'red':
            #Store Bouy in a list?
            NewObjectPos = (NewObject[0], NewObject[1])
            distanceToNewObject = Distance(self.ASVPos, NewObjectPos)
            distanceToObject = Distance(self.ASVPos, self.CurrentRedBouyPos)
            if self.CurrentRedBouyPos == (0,0) or distanceToNewObject < distanceToObject:
                self.CurrentRedBouyPos = NewObjectPos
            if distanceToObject < self.ClosestObject[0]:
                self.ClosestObject = (distanceToObject, 'red')
        
        if NewObject[2] == 'green':
            #Store Bouy in a list?
            NewObjectPos = (NewObject[0], NewObject[1])
            distanceToNewObject = Distance(self.ASVPos, NewObjectPos)
            distanceToObject = Distance(self.ASVPos, self.CurrentGreenBouyPos)
            if self.CurrentGreenBouyPos == (0,0) or distanceToNewObject < distanceToObject:
                self.CurrentGreenBouy = NewObjectPos
            if distanceToObject < self.ClosestObject[0]:
                self.ClosestObject = (distanceToObject, 'green')
        
        if NewObject[2] == 'north':
            #Store Bouy in a list?
            NewObjectPos = (NewObject[0], NewObject[1])
            distanceToNewObject = Distance(self.ASVPos, NewObjectPos)
            distanceToObject = Distance(self.ASVPos, self.CurrentNorthMarkerPos)
            if self.CurrentNorthMarkerPos == (0,0) or distanceToNewObject < distanceToObject:
                self.CurrentNorthMarkerPos = NewObjectPos
            if distanceToObject < self.ClosestObject[0]:
                self.ClosestObject = (distanceToObject, 'north')
        
        if NewObject[2] == 'south':
            #Store Bouy in a list?
            NewObjectPos = (NewObject[0], NewObject[1])
            distanceToNewObject = Distance(self.ASVPos, NewObjectPos)
            distanceToObject = Distance(self.ASVPos, self.CurrentSouthMarkerPos)
            if self.CurrentSouthMarkerPos[2] == (0,0) or distanceToNewObject < distanceToObject:
                self.CurrentSouthMarkerPos = NewObjectPos
            if distanceToObject < self.ClosestObject[0]:
                self.ClosestObject = (distanceToObject, 'south')
        
        if NewObject[2] == 'east':
            #Store Bouy in a list?
            NewObjectPos = (NewObject[0], NewObject[1])
            distanceToNewObject = Distance(self.ASVPos, NewObjectPos)
            distanceToObject = Distance(self.ASVPos, self.CurrentEastMarkerPos)
            if self.CurrentEastMarkerPos[2] == (0,0) or distanceToNewObject < distanceToObject:
                self.CurrentEastMarkerPos = NewObjectPos
            if distanceToObject < self.ClosestObject[0]:
                self.ClosestObject = (distanceToObject, 'east')
        
        if NewObject[2] == 'west':
            #Store Bouy in a list?
            NewObjectPos = (NewObject[0], NewObject[1])
            distanceToNewObject = Distance(self.ASVPos, NewObjectPos)
            distanceToObject = Distance(self.ASVPos, self.CurrentWestMarkerPos)
            if self.CurrentWestMarkerPos[2] == (0,0) or distanceToNewObject < distanceToObject:
                self.CurrentWestMarkerPos = NewObjectPos
            if distanceToObject < self.ClosestObject[0]:
                self.ClosestObject = (distanceToObject, 'west')

        #Choosing closest point
        return self.ClosestObject[1]
        ## ToDo
        # Delete self.Current"objects" that we do not want or need anymore. Ex; objects we have or are currently passing.
        # These objects can potesially be stored in a list. 
        # If we have a green and red bouy position, return 'greenAndRedBouyNav'
    
class OneRedBouyNav(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['search'])

    def execute(self, userdata):
        rospy.loginfo('OneRedBouyNav')
        pass

class OneGreenBouyNav(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['search'])

    def execute(self, userdata):
        rospy.loginfo('OneGreenBouyNav')
        pass

class GreenAndReadBouyNav(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['search'])

    def execute(self, userdata):
        rospy.loginfo('GreenAndReadBouyNav')
        pass

class NorthMarkerNav(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['search'])

    def execute(self, userdata):
        rospy.loginfo('NorthMarkerNav')
        pass

class SouthMarkerNav(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['search'])

    def execute(self, userdata):
        rospy.loginfo('NorthMarkerNav')
        pass

class WestMarkerNav(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['search'])

    def execute(self, userdata):
        rospy.loginfo('NorthMarkerNav')
        pass

class EastMarkerNav(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['search'])

    def execute(self, userdata):
        rospy.loginfo('NorthMarkerNav')
        pass

def Distance(ObjectOnePosition, ObjectTwoPosition):
    x1, y1 = ObjectOnePosition
    x2, y2 = ObjectTwoPosition
    distanceX = x1 - x2
    distanceY = y1 - y2
    hypDistance = math.sqrt(distanceX**2 + distanceY**2)
    return hypDistance