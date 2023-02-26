#!/usr/bin/python3
import rospy
import smach
import smach_ros
import fsm_states


class ManeuveringNavigationTasks:
    def __init__(self):
        self.enabled = rospy.get_param("/tasks/maneuvering_navigation_tasks")
        # Your code here

    def spin(self):
        # Create the state machine
        sm = smach.StateMachine(outcomes=['STOP'])
        # Add states to the state machine
        with sm:
            smach.StateMachine.add('STATE1', fsm_states.FirstState(),
                                    transitions={'outcome1': 'STATE2', 'outcomex': 'STOP'})
            smach.StateMachine.add('STATE2', fsm_states.SecondState(),
                                    transitions={'outcome2': 'STATE3', 'outcomex': 'STOP'})
            smach.StateMachine.add('STATE3', fsm_states.ThirdState(),
                                    transitions={'outcome3': 'STATE1', 'outcomex': 'STOP'})
        # Start the state machine introspection server
        sis = smach_ros.IntrospectionServer('state_machine', sm, '/SM_ROOT')
        sis.start()
        
        while not rospy.is_shutdown():
            self.enabled = rospy.get_param("/tasks/maneuvering_navigation_tasks")
            ud = smach.UserData()
            outcome = sm.execute(ud)

            if self.enabled == False:
                print("Exiting because this fsm should be inactive.")
                break

            if outcome == 'STOP':
                print("State machine stopped")
                break

        # Stop the state machine introspection server (because while loop is finished)
        sis.stop()



if __name__ == "__main__":
    rospy.init_node("fsm_njord")
    fsm_node = ManeuveringNavigationTasks
    fsm_node.spin()
