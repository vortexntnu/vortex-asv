#!/usr/bin/python3
import rospy
import smach
import smach_ros
import mission.BouysTasksNjord.bouys_tasks as bouys_tasks


class ManeuveringNavigationTasks:
    def __init__(self):
        self.enabled = rospy.get_param("/tasks/maneuvering_navigation_tasks")
        # Your code here

    def spin(self):
        # Create the state machine
        sm = smach.StateMachine(outcomes=['STOP'])
        # Add states to the state machine
        with sm:
            smach.StateMachine.add('Idle', bouys_tasks.Idle(),
                                    transitions={'search': 'Search',
                                                 'stop'  : 'STOP',})
            
            
            smach.StateMachine.add('Search', bouys_tasks.Search(),
                                    transitions={'greenAndReadBouyNav' : 'GreenAndReadBouyNav',
                                                 'red'   : 'OneRedBouyNav',
                                                 'green' : 'OneGreenBouyNav',
                                                 'north' : 'CardinalMarkerNav',
                                                 'south' : 'SouthMarkerNav',
                                                 'east'  : 'EastMarkerNav',
                                                 'west'  : 'WestMarkerNav',
                                                 'idle'  : 'Idle',
                                                 })
            
            smach.StateMachine.add('OneRedBouyNav', bouys_tasks.OneRedBouyNav(),
                                    transitions={'search': 'Search'})
            
            smach.StateMachine.add('OneGreenBouyNav', bouys_tasks.OneGreenBouyNav(), 
                                    transitions={'search': 'Search'})
            
            smach.StateMachine.add('GreenAndReadBouyNav', bouys_tasks.GreenAndReadBouyNav(),
                                    transitions={'search': 'Search'})
            
            smach.StateMachine.add('NorthMarkerNav', bouys_tasks.NorthMarkerNav(),
                                    transitions={'search': 'Search'})
            
            smach.StateMachine.add('SouthMarkerNav', bouys_tasks.SouthMarkerNav(),
                                    transitions={'search': 'Search'})
            
            smach.StateMachine.add('EastMarkerNav', bouys_tasks.EastMarkerNav(),
                                    transitions={'search': 'Search'})
            
            smach.StateMachine.add('WestMarkerNav', bouys_tasks.WestMarkerNav(),
                                    transitions={'search': 'Search'})
            
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
    fsm_node = ManeuveringNavigationTasks()
    fsm_node.spin()
