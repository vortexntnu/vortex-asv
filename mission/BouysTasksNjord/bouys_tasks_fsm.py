#!/usr/bin/python3
#Written by Sigurd von Brandis, Student

import rospy
import smach
import smach_ros
import math
from bouys_tasks import Idle, Search, DesideNextState, OneRedBouyNav, OneGreenBouyNav, GreenAndReadBouyNav, NorthMarkerNav, SouthMarkerNav, EastMarkerNav, WestMarkerNav
from nav_msgs.msg import Odometry
from update_objects_data import DetectedObjectsData  # custom message type for the combined data.
from vortex_msgs.msg import DetectedObjectArray, DetectedObject
from update_objects_data import UpdateDataNode


class ManeuveringNavigationTasks:

    def __init__(self):
        #self.enabled = rospy.get_param("/tasks/maneuvering_navigation_tasks")
        self.data = DetectedObjectsData
        self.data.closest_object = (math.inf, '')
        self.data.second_closest_object = (math.inf, '')
        self.data.DistanceRadius = 3  #Meters. Used to define curve ASV can follow when it only knows one bouy.
        self.data.DirectionWithLeia = True  #Used to descide which side the ASV should be regarding Green and Read "Staker".
        self.data.ObjectSearchAttempts = 0

        self.data_sub = rospy.Subscriber('detected_objects',
                                         DetectedObjectArray, self.data_cb)

        self.vessel_pos_sub = rospy.Subscriber('/odometry/filtered', Odometry,
                                               self.odom_cb)

    def data_cb(self, msg):
        detected_objects = msg.detected_objects
        self.data.current_red_bouy = (detected_objects[0].x,
                                      detected_objects[0].y,
                                      detected_objects[0].type)
        self.data.current_green_bouy = (detected_objects[1].x,
                                        detected_objects[1].y,
                                        detected_objects[1].type)
        # self.data.current_north_marker = (detected_objects[2].x,
        #                                   detected_objects[2].y,
        #                                   detected_objects[2].type)
        # self.data.current_south_marker = (detected_objects[3].x,
        #                                   detected_objects[3].y,
        #                                   detected_objects[3].type)
        # self.data.current_east_marker = (detected_objects[4].x,
        #                                  detected_objects[4].y,
        #                                  detected_objects[4].type)
        # self.data.current_west_marker = (detected_objects[5].x,
        #                                  detected_objects[5].y,
        #                                  detected_objects[5].type)

    def odom_cb(self, msg):
        self.data.vessel_position = (msg.pose.pose.position.x,
                                     msg.pose.pose.position.y)

    def find_closest_objects(self):
        for name, new_object in vars(self.data).items():
            if name.startswith('current_') or (name.endswith('bouy')
                                               or name.endswith('marker')):
                new_obj_type = new_object[2]
                new_obj_pos = (new_object[0], new_object[1])
                dist_to_new_obj = UpdateDataNode.distance(
                    self.data.vessel_position, new_obj_pos)
                old_closest_obj_type = self.data.closest_object[2]
                old_closest_obj_pos = (self.data.closest_object[0],
                                       self.data.closest_object[1])
                dist_to_old_closest_obj = UpdateDataNode.distance(
                    self.data.vessel_position, old_closest_obj_pos)
                old_second_closest_obj_type = self.data.second_closest_object[
                    2]
                old_second_closest_obj_pos = (
                    self.data.second_closest_object[0],
                    self.data.second_closest_object[1])
                dist_to_old_second_closest_obj = UpdateDataNode.distance(
                    self.data.vessel_position, old_second_closest_obj_pos)
            if dist_to_new_obj < dist_to_old_closest_obj:
                self.data.second_closest_object = (dist_to_old_closest_obj,
                                                   old_closest_obj_type)
                self.data.closest_object = (dist_to_new_obj, new_obj_type)
            elif dist_to_new_obj < dist_to_old_second_closest_obj:
                self.data.second_closest_object = (dist_to_new_obj,
                                                   new_obj_type)
                self.data.closest_object = (dist_to_old_closest_obj,
                                            old_closest_obj_type)
            else:  #No new closest objects, but updating distance to the old closest objects again because our position may have changed
                self.data.second_closest_object = (
                    dist_to_old_second_closest_obj,
                    old_second_closest_obj_type)
                self.data.closest_object = (dist_to_old_closest_obj,
                                            old_closest_obj_type)

    def spin(self):
        # Create the state machine
        sm = smach.StateMachine(outcomes=['idle',
                                          'greenAndRedBouyNav',
                                          'red',
                                          'green',
                                          'north',
                                          'south',
                                          'east',
                                          'west']) 
                                         #'STOP',

        # Add states to the state machine
        with sm:
            smach.StateMachine.add('Idle',
                                   Idle(self.data),
                                   transitions={
                                       'search': 'Search',
                                       'desideNextState': 'DesideNextState',
                                    #   'stop': 'STOP'
                                   })

            smach.StateMachine.add('Search',
                                   Search(self.data),
                                   transitions={
                                       'idle': 'Idle',
                                    #   'stop': 'STOP'
                                   })

            smach.StateMachine.add('DesideNextState',
                                   DesideNextState(self.data),
                                   transitions={
                                       'greenAndReadBouyNav': 'GreenAndReadBouyNav',
                                       'red': 'OneRedBouyNav',
                                       'green': 'OneGreenBouyNav',
                                       'north': 'NorthMarkerNav',
                                       'south': 'SouthMarkerNav',
                                       'east': 'EastMarkerNav',
                                       'west': 'WestMarkerNav',
                                       'idle': 'Idle'
                                   })

            smach.StateMachine.add(
                'OneRedBouyNav',
                OneRedBouyNav(self.data),
                transitions={'desideNextState': 'DesideNextState'})

            smach.StateMachine.add(
                'OneGreenBouyNav',
                OneGreenBouyNav(self.data),
                transitions={'desideNextState': 'DesideNextState'})

            smach.StateMachine.add(
                'GreenAndReadBouyNav',
                GreenAndReadBouyNav(self.data),
                transitions={'desideNextState': 'DesideNextState'})

            smach.StateMachine.add(
                'NorthMarkerNav',
                NorthMarkerNav(self.data),
                transitions={'desideNextState': 'DesideNextState'})

            smach.StateMachine.add(
                'SouthMarkerNav',
                SouthMarkerNav(self.data),
                transitions={'desideNextState': 'DesideNextState'})

            smach.StateMachine.add(
                'EastMarkerNav',
                EastMarkerNav(self.data),
                transitions={'desideNextState': 'DesideNextState'})

            smach.StateMachine.add(
                'WestMarkerNav',
                WestMarkerNav(self.data),
                transitions={'desideNextState': 'DesideNextState'})

        # Start the state machine introspection server
        sis = smach_ros.IntrospectionServer('state_machine', sm, '/SM_ROOT')
        sis.start()

        while not rospy.is_shutdown():
            # To be updated
            # self.enabled = rospy.get_param(
            #     "/tasks/maneuvering_navigation_tasks")
            self.enabled = True  #Remove this line when not testing
            if not self.enabled:
                print("Exiting because this fsm should be inactive.")
                break

            ud = smach.UserData()
            outcome = sm.execute(ud)
            if outcome == 'STOP':
                print("State machine stopped")
                break

            ManeuveringNavigationTasks.find_closest_objects()

        sis.stop()


if __name__ == "__main__":
    try:
        rospy.init_node('Bouys_tasks_fsm')
        fsm_node = ManeuveringNavigationTasks()
        fsm_node.spin()
    except rospy.ROSInterruptException:
        pass
