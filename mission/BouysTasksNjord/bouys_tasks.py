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
        self.CurrentRedBouyPos     = (0, 0)
        self.CurrentGreenBouyPos   = (0, 0)
        self.CurrentNorthMarkerPos = (0, 0)
        self.CurrentSouthMarkerPos = (0, 0)
        self.CurrentEastMarkerPos  = (0, 0)
        self.CurrentWestMarkerPos  = (0, 0)

    def execute(self, userdata):
        rospy.loginfo('Executing Search')
        self.ObjectSearchAttempts  = self.ObjectSearchAttempts + 1
        #Copy search pattern from AUV

        NewObject = (2,5,'red') #Switch out with actual discovered bouy from search
        
        if NewObject[2] == 'red':
            #Store Bouy in a list?
            NewObjectPos = (NewObject[0], NewObject[1])
            distanceToNewObject = Distance(self.ASVPos, NewObjectPos)
            distanceToCurrentRedBouy = Distance(self.ASVPos, self.CurrentRedBouyPos)
            if self.CurrentRedBouy == (0,0) or distanceToNewObject < distanceToCurrentRedBouy:
                self.CurrentRedBouy = NewObjectPos
        
        if NewObject[2] == 'green':
            #Store Bouy in a list?
            NewObjectPos = (NewObject[0], NewObject[1])
            distanceToNewObject = Distance(self.ASVPos, NewObjectPos)
            distanceToCurrentGreenBouy = Distance(self.ASVPos, self.CurrentGreenBouyPos)
            if self.CurrentGreenBouy[2] == 'noType' or distanceToNewObject < distanceToCurrentGreenBouy:
                self.CurrentGreenBouy = NewObjectPos
        
        if NewObject[2] == 'north':
            #Store Bouy in a list?
            NewObjectPos = (NewObject[0], NewObject[1])
            distanceToNewObject = Distance(self.ASVPos, NewObjectPos)
            distanceToCurrentNorthMarker = Distance(self.ASVPos, self.CurrentNorthMarkerPos)
            if self.CurrentNorthMarker[2] == 'noType' or distanceToNewObject < distanceToCurrentNorthMarker:
                self.CurrentNorthMarker = NewObjectPos
        
        


        pass
    
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
    distanceX = (x1 + x2)/2
    distanceY = (y1 + y2)/2
    hypDistance = math.sqrt(distanceX**2 + distanceY**2)
    return hypDistance