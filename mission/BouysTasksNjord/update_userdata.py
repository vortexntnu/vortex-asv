#!/usr/bin/env python

import rospy
from std_msgs.msg import String
#from my_msgs.msg import MyData # custom message type for the combined data.
from nav_msgs.msg import Odometry
import math

class DetectedObjectsData:
    def __init__(self):
        self.current_red_bouy      = (0, 0, 'red')
        self.current_green_bouy    = (0, 0, 'green')
        self.current_north_marker  = (0, 0, 'north')
        self.current_south_marker  = (0, 0, 'south')
        self.current_east_marker   = (0, 0, 'east')
        self.current_west_marker   = (0, 0, 'west')
        self.closest_object        = (0, '')
        self.second_closest_object = (0, '')
        self.vessel_position       = (0, 0)

class UpdateDataNode:
    def __init__(self):
        rospy.init_node('update_data_node')     
        
        # Initialize subscribers to topic1 and topic2
        self.sub1 = rospy.Subscriber('detected_objects', DataFromPerception, self.callback1) #This line is bullshit
        self.sub2 = rospy.Subscriber('vessel_position', String, self.callback2) 
        
        # Initialize publisher to data topic
        self.pub = rospy.Publisher('updated_data_Nav_tasks_Njord', DetectedObjectsData, queue_size=1)

    def callback1(self, msg):
        # Extract data from topic1 and combine with topic2 data
        combined_data = DetectedObjectsData()
        combined_data.objects.CurrentRedBouy = msg.objects.CurrentRedBouy
        combined_data.objects.CurrentGreenBouy = msg.objects.CurrentGreenBouy
        combined_data.objects.CurrentNorthMarker = msg.objects.CurrentNorthMarker
        combined_data.objects.CurrentSouthMarker = msg.objects.CurrentSouthMarker
        combined_data.objects.CurrentEastMarker = msg.objects.CurrentEastMarker
        combined_data.objects.CurrentWestMarker = msg.objects.CurrentWestMarker
        
        UpdateDataNode.find_closest_objects()

        if hasattr(self, 'vessel_position'):
            combined_data.vessel_position = self.vessel_position
        
        # Publish combined data on data topic
        self.pub.publish(combined_data)
    
    def callback2(self, msg):
        # Extract data from vessel_position and store for later use
        data_parts = msg.split(',')
        self.vessel_position = (float(data_parts[0]), float(data_parts[1]))

        #Should perhaps be in a different node that updates the topic 'updated_bouy_data_Njord'
    def distance(ObjectOnePosition, ObjectTwoPosition):
        x1, y1 = ObjectOnePosition
        x2, y2 = ObjectTwoPosition
        distanceX = x1 - x2
        distanceY = y1 - y2
        hypDistance = math.sqrt(distanceX**2 + distanceY**2)
        return hypDistance

    #Should perhaps be in a different node that updates the topic 'updated_bouy_data_Njord'
    def find_closest_objects(self):

        for name, new_object in vars(self).items():
            if name.startswith('current_') or (name.endswith('bouy') or name.endswith('marker')):

                new_obj_type = new_object[2]
                new_obj_pos = (new_object[0], new_object[1])
                dist_to_new_obj = UpdateDataNode.distance(self.vessel_position, new_obj_pos)

                old_closest_obj_type = self.closest_object[2]
                old_closest_obj_pos = (self.closest_object[0], self.closest_object[1])
                dist_to_old_closest_obj = UpdateDataNode.distance(self.vessel_position, old_closest_obj_pos)
                
                old_second_closest_obj_type = self.second_closest_object[2]
                old_second_closest_obj_pos = (self.second_closest_object[0], self.second_closest_object[1])
                dist_to_old_second_closest_obj = UpdateDataNode.distance(self.vessel_position, old_second_closest_obj_pos)
            
            if dist_to_new_obj < dist_to_old_closest_obj:
                self.second_closest_object = (dist_to_old_closest_obj, old_closest_obj_type)
                self.closest_object = (dist_to_new_obj, new_obj_type)
            elif dist_to_new_obj < dist_to_old_second_closest_obj:
                self.second_closest_object = (dist_to_new_obj, new_obj_type)
                self.closest_object = (dist_to_old_closest_obj, old_closest_obj_type)
            else: #No new closest objects, but calculating distance to the old closest objects again
                self.second_closest_object = (dist_to_old_second_closest_obj, old_second_closest_obj_type)
                self.closest_object = (dist_to_old_closest_obj, old_closest_obj_type)

            # return self.closest_object, self.second_closest_object unnessesary?

    
if __name__ == '__main__':
    try:
        UpdateDataNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

