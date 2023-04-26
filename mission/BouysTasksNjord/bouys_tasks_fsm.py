#!/usr/bin/python3
#Written by Sigurd von Brandis, Student

import rospy
import smach
import smach_ros
import math
import mission.BouysTasksNjord.bouys_tasks as bouys_tasks
from nav_msgs.msg import Odometry
from update_objects_data import DetectedObjectsData # custom message type for the combined data.


rospy.init_node('Bouys_tasks_fsm')

class ManeuveringNavigationTasks:
    def __init__(self):
        self.enabled = rospy.get_param("/tasks/maneuvering_navigation_tasks")
        self.sub_bouy_info = rospy.Subscriber('object_data_Njord', DetectedObjectsData, self.bouy_data_callback)

        self = DetectedObjectsData()
        # self.NoGoSircleRadius = 2 #Meters. Used to define area around bouy that ASV must absolutely NOT enter. 
        self.DistanceRadius = 3 #Meters. Used to define curve ASV can follow when it only knows one bouy.
        self.DirectionWithLeia = True #Used to descide which side the ASV should be regarding Green and Read "Staker".
        self.ObjectSearchAttempts = 0
        
    
    def bouy_data_callback(self, data):
        # Parse the received message and set class attributes
        self.current_red_bouy = data.objects.CurrentRedBouy
        self.current_green_bouy = data.objects.CurrentGreenBouy
        self.current_north_marker = data.objects.CurrentNorthMarker
        self.current_south_marker = data.objects.CurrentSouthMarker
        self.current_east_marker = data.objects.CurrentEastMarker
        self.current_west_marker = data.objects.CurrentWestMarker

    def spin(self):
        # Create the state machine
        sm = smach.StateMachine(outcomes=['STOP'])

        # Add states to the state machine
        with sm:
            smach.StateMachine.add('Idle', bouys_tasks.Idle(self),
                                    transitions={'search'          : 'Search',
                                                 'desideNextState' : 'DesideNextState',
                                                 'stop'            : 'STOP'})
            
            smach.StateMachine.add('Search', bouys_tasks.Search(self),
                                    transitions={'idle' : 'Idle',
                                                 'stop' : 'STOP'})
            
            smach.StateMachine.add('DisideNextState', bouys_tasks.Search(self),
                                    transitions={'greenAndReadBouyNav' : 'GreenAndReadBouyNav',
                                                 'red'   : 'OneRedBouyNav',
                                                 'green' : 'OneGreenBouyNav',
                                                 'north' : 'CardinalMarkerNav',
                                                 'south' : 'SouthMarkerNav',
                                                 'east'  : 'EastMarkerNav',
                                                 'west'  : 'WestMarkerNav',
                                                 'idle'  : 'Idle'})
            
            smach.StateMachine.add('OneRedBouyNav', bouys_tasks.OneRedBouyNav(self),
                                    transitions={'desideNextState': 'DesideNextState'})
            
            smach.StateMachine.add('OneGreenBouyNav', bouys_tasks.OneGreenBouyNav(self), 
                                    transitions={'desideNextState': 'DesideNextState'})
            
            smach.StateMachine.add('GreenAndReadBouyNav', bouys_tasks.GreenAndReadBouyNav(self),
                                    transitions={'desideNextState': 'DesideNextState'})
            
            smach.StateMachine.add('NorthMarkerNav', bouys_tasks.NorthMarkerNav(self),
                                    transitions={'desideNextState': 'DesideNextState'})
            
            smach.StateMachine.add('SouthMarkerNav', bouys_tasks.SouthMarkerNav(self),
                                    transitions={'desideNextState': 'DesideNextState'})
            
            smach.StateMachine.add('EastMarkerNav', bouys_tasks.EastMarkerNav(self),
                                    transitions={'desideNextState': 'DesideNextState'})
            
            smach.StateMachine.add('WestMarkerNav', bouys_tasks.WestMarkerNav(self),
                                    transitions={'desideNextState': 'DesideNextState'})
            
        # Start the state machine introspection server
        sis = smach_ros.IntrospectionServer('state_machine', sm, '/SM_ROOT')
        sis.start()    
        
        while not rospy.is_shutdown():

            self.enabled = rospy.get_param("/tasks/maneuvering_navigation_tasks")
            if self.enabled == False:
                print("Exiting because this fsm should be inactive.")
                break

            ud = smach.UserData()
            outcome = sm.execute(ud)
            if outcome == 'STOP':
                print("State machine stopped")
                break

        # Stop the state machine introspection server (because while loop is finished)
        sis.stop()



if __name__ == "__main__":
    rospy.init_node("fsm_njord")
    fsm_node = ManeuveringNavigationTasks()
    fsm_node.spin()

