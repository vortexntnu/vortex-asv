#!/usr/bin/python3
#Written by Sigurd von Brandis, Student

import rospy
import smach
import smach_ros
import math
from bouys_tasks import Idle, Search, DetectedObjectsNavigation



class ManeuveringNavigationTasks:

    def __init__(self):
        rospy.init_node('Bouys_tasks_fsm')

    def spin(self):
        sm = smach.StateMachine(outcomes=['Stop'])

        with sm:
            sm.userdata.closest_object = (math.inf, math.inf, '')
            sm.userdata.object_search_attempts = 0
            smach.StateMachine.add('Idle',
                                   Idle(),
                                   transitions={
                                       'detectedObjectsNavigation':
                                       'DetectedObjectsNavigation',
                                       'search': 'Search',
                                       'stop': 'Stop',
                                   },
                                   remapping={
                                       'closest_object':
                                       'closest_object',
                                       'object_search_attempts':
                                       'object_search_attempts',
                                   })

            smach.StateMachine.add('Search',
                                   Search(),
                                   transitions={
                                       'detectedObjectsNavigation':
                                       'DetectedObjectsNavigation'
                                   },
                                   remapping={
                                       'object_search_attempts':
                                        'object_search_attempts',
                                   })

            smach.StateMachine.add('DetectedObjectsNavigation',
                                   DetectedObjectsNavigation(),
                                   transitions={'idle': 'Idle'},
                                   remapping={
                                       'closest_object': 'closest_object',
                                   })

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

        sis.stop()


if __name__ == "__main__":
    try:
        fsm_node = ManeuveringNavigationTasks()
        fsm_node.spin()
    except rospy.ROSInterruptException:
        pass
