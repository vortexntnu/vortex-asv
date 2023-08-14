#!/usr/bin/python3

import rospy
import smach
import smach_ros
import math
from sea_marker_task_states import Maneuvering1, Maneuvering2, Maneuvering3


class sea_marker_task:

    def __init__(self):
        rospy.init_node('sea_markers_task_fsm')

    def spin(self):
        sm = smach.StateMachine(outcomes=['Stop'])

        with sm:
            smach.StateMachine.add('Maneuvering1',
                                   Maneuvering1(),
                                   transitions={'maneuvering1': 'Maneuvering1',
                                                'maneuvering2': 'Maneuvering2',
                                                'maneuvering3': 'Maneuvering3',
                                                'stop':'Stop'})
            smach.StateMachine.add('Maneuvering2',
                                   Maneuvering2(),
                                   transitions={'maneuvering1': 'Maneuvering1',
                                                'maneuvering2': 'Maneuvering2',
                                                'maneuvering3': 'Maneuvering3',
                                                'stop':'Stop'})
            smach.StateMachine.add('Maneuvering3',
                                   Maneuvering3(),
                                   transitions={'maneuvering1': 'Maneuvering1',
                                                'maneuvering2': 'Maneuvering2',
                                                'maneuvering3': 'Maneuvering3',
                                                'stop':'Stop'})


        # Start the state machine introspection server
        sis = smach_ros.IntrospectionServer('state_machine', sm, '/SM_ROOT')
        sis.start()

        while not rospy.is_shutdown():

            if not (rospy.get_param("/tasks/sea_marker_task1") or rospy.get_param("/tasks/sea_marker_task2") or rospy.get_param("/tasks/sea_marker_task3")):
                print("Exiting because this fsm should be inactive.")
                break

            ud = smach.UserData()
            outcome = sm.execute(ud)
            if outcome == 'Stop':
                print("State machine stopped")
                break

        sis.stop()


if __name__ == "__main__":
    try:
        fsm_node = sea_marker_task()
        fsm_node.spin()
    except rospy.ROSInterruptException:
        pass