#!/usr/bin/env python

import rospy
import smach
import smach_ros

# Right now outcomex ,which means that the STATE MACHINE should stop, never happpens.

class FirstState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcomex'])

    def execute(self, userdata):
        rospy.loginfo('Executing First State')
        return 'outcome1'

class SecondState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2', 'outcomex'])

    def execute(self, userdata):
        rospy.loginfo('Executing Second State')
        return 'outcome2'
    
class ThirdState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome3', 'outcomex'])

    def execute(self, userdata):
        rospy.loginfo('Executing state 3')
        return 'outcome3'