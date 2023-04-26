#!/usr/bin/python3
#Written by Sigurd von Brandis, Student

import rospy
import smach
import smach_ros
import math
from bouys_tasks import Idle, Search, DesideNextState, OneRedBouyNav, OneGreenBouyNav, GreenAndReadBouyNav, NorthMarkerNav, SouthMarkerNav, EastMarkerNav, WestMarkerNav
from nav_msgs.msg import Odometry
from update_objects_data import DetectedObjectsData  # custom message type for the combined data.
from vortex_msgs import ObjectMarkerNavigation

class ManeuveringNavigationTasks:

    def __init__(self):
        #self.enabled = rospy.get_param("/tasks/maneuvering_navigation_tasks")
        self.sub_bouy_info = rospy.Subscriber('object_data_Njord',
                                              DetectedObjectsData, #ToDo; Switch out with new message
                                              self.bouy_data_callback)

        self.info = DetectedObjectsData()

    def bouy_data_callback(self, data):
        # Parse the received message and set class attributes
        self.info.current_red_bouy = data.objects.CurrentRedBouy
        self.info.current_green_bouy = data.objects.CurrentGreenBouy
        self.info.current_north_marker = data.objects.CurrentNorthMarker
        self.info.current_south_marker = data.objects.CurrentSouthMarker
        self.info.current_east_marker = data.objects.CurrentEastMarker
        self.info.current_west_marker = data.objects.CurrentWestMarker

    def spin(self):
        # Create the state machine
        sm = smach.StateMachine(outcomes=['STOP'])

        # Add states to the state machine
        with sm:
            smach.StateMachine.add('Idle',
                                   Idle(self.info),
                                   transitions={
                                       'search': 'Search',
                                       'desideNextState': 'DesideNextState',
                                       'stop': 'STOP'
                                   })

            smach.StateMachine.add('Search',
                                   Search(self.info),
                                   transitions={
                                       'idle': 'Idle',
                                       'stop': 'STOP'
                                   })

            smach.StateMachine.add('DisideNextState',
                                   Search(self.info),
                                   transitions={
                                       'greenAndReadBouyNav':
                                       'GreenAndReadBouyNav',
                                       'red': 'OneRedBouyNav',
                                       'green': 'OneGreenBouyNav',
                                       'north': 'CardinalMarkerNav',
                                       'south': 'SouthMarkerNav',
                                       'east': 'EastMarkerNav',
                                       'west': 'WestMarkerNav',
                                       'idle': 'Idle'
                                   })

            smach.StateMachine.add(
                'OneRedBouyNav',
                OneRedBouyNav(self.info),
                transitions={'desideNextState': 'DesideNextState'})

            smach.StateMachine.add(
                'OneGreenBouyNav',
                OneGreenBouyNav(self.info),
                transitions={'desideNextState': 'DesideNextState'})

            smach.StateMachine.add(
                'GreenAndReadBouyNav',
                GreenAndReadBouyNav(self.info),
                transitions={'desideNextState': 'DesideNextState'})

            smach.StateMachine.add(
                'NorthMarkerNav',
                NorthMarkerNav(self.info),
                transitions={'desideNextState': 'DesideNextState'})

            smach.StateMachine.add(
                'SouthMarkerNav',
                SouthMarkerNav(self.info),
                transitions={'desideNextState': 'DesideNextState'})

            smach.StateMachine.add(
                'EastMarkerNav',
                EastMarkerNav(self.info),
                transitions={'desideNextState': 'DesideNextState'})

            smach.StateMachine.add(
                'WestMarkerNav',
                WestMarkerNav(self.info),
                transitions={'desideNextState': 'DesideNextState'})

        # Start the state machine introspection server
        sis = smach_ros.IntrospectionServer('state_machine', sm, '/SM_ROOT')
        sis.start()

        while not rospy.is_shutdown():

            self.enabled = rospy.get_param(
                "/tasks/maneuvering_navigation_tasks")
            if self.enabled == False:
                print("Exiting because this fsm should be inactive.")
                break

            ud = smach.UserData()
            outcome = sm.execute(ud)
            if outcome == 'STOP':
                print("State machine stopped")
                break

        sis.stop()


if __name__ == "__main__":
    rospy.init_node('Bouys_tasks_fsm')
    fsm_node = ManeuveringNavigationTasks()
    fsm_node.spin()
