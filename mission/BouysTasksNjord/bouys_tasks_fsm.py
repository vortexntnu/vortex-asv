#!/usr/bin/python3
import rospy
import smach
import smach_ros
import math
import mission.BouysTasksNjord.bouys_tasks as bouys_tasks
from nav_msgs.msg import Odometry
#from my_msgs.msg import MyData # custom message type for the combined data. Talk with hannah


rospy.init_node('Bouys_tasks_fsm')

class ManeuveringNavigationTasks:
    def __init__(self):
        self.enabled = rospy.get_param("/tasks/maneuvering_navigation_tasks")
        self.sub_bouy_info = rospy.Subscriber('updated_bouy_data_Njord', MyData, self.bouy_data_callback)

        # Initialize class attributes
        #                          Position, type
        self.current_red_bouy      = (0, 0, 'red')
        self.current_green_bouy    = (0, 0, 'green')
        self.current_north_marker  = (0, 0, 'north')
        self.current_south_marker  = (0, 0, 'south')
        self.current_east_marker   = (0, 0, 'east')
        self.current_west_marker   = (0, 0, 'west')
        #                          Distance, type
        self.closest_object        = (0, '')
        self.second_closest_object = (0, '')
        #                          Position
        self.vessel_position       = (0, 0) #Odometry()
        ##
        # self.treshhold = 1 #meter
        # self.Distance = 100 #meter. Just some lagre init value.
        # self.NoGoSircleRadius = 2 #Meters. Used to define area around bouy that ASV must absolutely NOT enter. 
        self.DistanceRadius = 3 #Meters. Used to define curve ASV can follow when it only knows one bouy.
        self.DirectionWithLeia = True #Used to descide which side the ASV should be regarding Green and Read "Staker".
        # self.DistanceToClosest = bouys_tasks.Distance(... , ...)
        self.ObjectSearchAttempts = 0
        ##
    
    def bouy_data_callback(self, data):
        # Parse the received message and set class attributes
        self.current_red_bouy = data.objects.CurrentRedBouy
        self.current_green_bouy = data.objects.CurrentGreenBouy
        self.current_north_marker = data.objects.CurrentNorthMarker
        self.current_south_marker = data.objects.CurrentSouthMarker
        self.current_east_marker = data.objects.CurrentEastMarker
        self.current_west_marker = data.objects.CurrentWestMarker


    def distance(ObjectOnePosition, ObjectTwoPosition):
        x1, y1 = ObjectOnePosition
        x2, y2 = ObjectTwoPosition
        distanceX = x1 - x2
        distanceY = y1 - y2
        hypDistance = math.sqrt(distanceX**2 + distanceY**2)
        return hypDistance

    def find_closest_objects(self):

        for name, new_object in vars(self).items():
            if name.startswith('current_') or (name.endswith('bouy') or name.endswith('marker')):

                new_obj_type = new_object[2]
                new_obj_pos = (new_object[0], new_object[1])
                dist_to_new_obj = ManeuveringNavigationTasks.distance(self.vessel_position, new_obj_pos)

                old_closest_obj_type = self.closest_object[2]
                old_closest_obj_pos = (self.closest_object[0], self.closest_object[1])
                dist_to_old_closest_obj = ManeuveringNavigationTasks.distance(self.vessel_position, old_closest_obj_pos)
                
                old_second_closest_obj_type = self.second_closest_object[2]
                old_second_closest_obj_pos = (self.second_closest_object[0], self.second_closest_object[1])
                dist_to_old_second_closest_obj = ManeuveringNavigationTasks.distance(self.vessel_position, old_second_closest_obj_pos)
            
            if dist_to_new_obj < dist_to_old_closest_obj:
                self.second_closest_object = (dist_to_old_closest_obj, old_closest_obj_type)
                self.closest_object = (dist_to_new_obj, new_obj_type)
            elif dist_to_new_obj < dist_to_old_second_closest_obj:
                self.second_closest_object = (dist_to_new_obj, new_obj_type)
                self.closest_object = (dist_to_old_closest_obj, old_closest_obj_type)
            else: #No new closest objects, but calculating distance to closest objects again
                self.second_closest_object = (dist_to_old_second_closest_obj, old_second_closest_obj_type)
                self.closest_object = (dist_to_old_closest_obj, old_closest_obj_type)

            # return self.closest_object, self.second_closest_object unnessesary?


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

            ManeuveringNavigationTasks.find_closest_objects()

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

