#!/usr/bin/python3
#Written by Sigurd von Brandis, Student

import rospy
import smach
import smach_ros
import math
from bouys_tasks import Idle, Search, DesideNextState, OneRedBouyNav, OneGreenBouyNav, GreenAndReadBouyNav, NorthMarkerNav, SouthMarkerNav, EastMarkerNav, WestMarkerNav
from nav_msgs.msg import Odometry
from update_objects_data import DetectedObjectsData  # custom message type for the combined data.
from vortex_msgs import DetectedObjectArray, DetectedObject


class ManeuveringNavigationTasks:

    def __init__(self):
        #self.enabled = rospy.get_param("/tasks/maneuvering_navigation_tasks")
        self.info = DetectedObjectsData
        self.info_sub = rospy.Subscriber('detected_objects',
                                         DetectedObjectArray, self.info_cb)
        self.vessel_pos_sub = rospy.Subscriber('/odometry/filtered', Odometry,
                                               self.odom_cb)

    def info_cb(self, msg):
        detected_objects = msg.detected_objects
        self.info.current_red_bouy = (detected_objects[0].x,
                                      detected_objects[0].y,
                                      detected_objects[0].type)
        self.info.current_green_bouy = (detected_objects[1].x,
                                        detected_objects[1].y,
                                        detected_objects[1].type)
        self.info.current_north_marker = (detected_objects[2].x,
                                          detected_objects[2].y,
                                          detected_objects[2].type)
        self.info.current_south_marker = (detected_objects[3].x,
                                          detected_objects[3].y,
                                          detected_objects[3].type)
        self.info.current_east_marker = (detected_objects[4].x,
                                         detected_objects[4].y,
                                         detected_objects[4].type)
        self.info.current_west_marker = (detected_objects[5].x,
                                         detected_objects[5].y,
                                         detected_objects[5].type)

    def odom_cb(self, msg):
        self.info.vessel_position = (msg.pose.pose.position.x,
                                     msg.pose.pose.position.y)

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
