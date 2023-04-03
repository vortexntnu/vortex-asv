#!/usr/bin/python3
import rospy
import smach
import smach_ros
import mission.BouysTasksNjord.bouys_tasks as bouys_tasks
from smach_ros import UserData

def userdata_callback(userdata):
    # update the value of the 'objectInfo' key in userdata
    objectInfo = [('Tuple %d' % i, i) for i in range(1, 7)] #Change out
    userdata.objectInfo = objectInfo

class ManeuveringNavigationTasks:
    def __init__(self):
        self.enabled = rospy.get_param("/tasks/maneuvering_navigation_tasks")
        # Your code here

    def spin(self):
        # Create the state machine
        sm = smach.StateMachine(outcomes=['STOP'])

        #Define data variables;
        userdata = UserData()
        #                              Position Name
        userdata.ASV                 = (0,0, '')
        userdata.CurrentRedBouy      = (0,0,'red')
        userdata.CurrentGreenBouy    = (0,0,'green')
        userdata.CurrentNorthMarker  = (0,0,'north')
        userdata.CurrentSouthMarker  = (0,0,'south')
        userdata.CurrentEastMarker   = (0,0,'east')
        userdata.CurrentWestMarker   = (0,0,'west')
        #                              Distance Name
        userdata.ClosestObject       = (0, '')
        userdata.SecondClosestObject = (0, '')

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

             # create a ROS node to update the userdata values
            rospy.init_node('userdata_updater')

            # Updating the objectInfo key in the userdata
            userdata_callback(userdata)
        
            # publish the updated userdata on a topic
            pub = rospy.Publisher('userdata_topic', UserData, queue_size=1)
            pub.publish(userdata)

        # Stop the state machine introspection server (because while loop is finished)
        sis.stop()



if __name__ == "__main__":
    rospy.init_node("fsm_njord")
    fsm_node = ManeuveringNavigationTasks()
    fsm_node.spin()

