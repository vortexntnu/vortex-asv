#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from landmark.srv import request_position
import numpy as np
import math

class DetectedObjectsData:
    #Latitude; x, Longitude; y
    def __init__(self):            #  x, y, type          
        self.current_red_bouy      = (0, 0, 'red')
        self.current_green_bouy    = (0, 0, 'green')
        self.current_north_marker  = (0, 0, 'north')
        self.current_south_marker  = (0, 0, 'south')
        self.current_east_marker   = (0, 0, 'east')
        self.current_west_marker   = (0, 0, 'west')
        self.closest_object        = (math.inf, '')
        self.second_closest_object = (math.inf, '')
        self.vessel_position       = (0, 0)

class UpdateDataNode:
    def __init__(self):
        rospy.init_node('objects_data_Njord_node')
        # Initialize object data to be published
        self.object_data = DetectedObjectsData()

        self.red_bouy_array = []
        self.green_bouy_array = []
        self.north_marker_array = []
        self.south_marker_array = []
        self.east_marker_array = []
        self.west_marker_array = []
        
        # Initialize subscriber and Service to get all the necessary information
        self.Obj_pos_sub = rospy.Subscriber('bouys_and_markers', self.object_pos_cb)
        self.Position_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_cb)
        
        # Initialize publisher to data topic
        self.pub = rospy.Publisher('object_data_Njord', DetectedObjectsData, queue_size=1)

    def spin(self):

        self.object_data.current_red_bouy = UpdateDataNode.find_closest_position_from_array()
        self.object_data.current_green_bouy = UpdateDataNode.find_closest_position_from_array()
        self.object_data.current_north_marker = UpdateDataNode.find_closest_position_from_array()
        self.object_data.current_south_marker = UpdateDataNode.find_closest_position_from_array()
        self.object_data.current_east_marker = UpdateDataNode.find_closest_position_from_array()
        self.object_data.current_west_marker = UpdateDataNode.find_closest_position_from_array()

        UpdateDataNode.find_closest_objects()

        self.pub.publish(self.object_data)
    
    def odom_cb(self, msg): 
        self.object_data.vessel_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def obj_pos_cb(self, msg):
        self.red_bouy_array = msg.red_bouy_array
        self.green_bouy_array = msg.green_bouy_array
        self.north_marker_array = msg.north_marker_array
        self.south_marker_array = msg.south_marker_array
        self.east_marker_array = msg.east_marker_array
        self.west_marker_array = msg.west_marker_array


    def update_array(array, new_point):
        updated = False
        for i, existing_point in enumerate(array):
            distance = UpdateDataNode.distance(new_point, existing_point)
            if distance <= 1:
                array[i] = new_point
                updated = True
                break
        if not updated:
            array = np.vstack([array, new_point])

        return array

    def find_closest_position_from_array(position, positions_array):
        closest_position = None
        closest_distance = math.inf
    
        for pos in positions_array:
            distance = UpdateDataNode.distance(position, pos)
            if distance < closest_distance:
                closest_distance = distance
                closest_position = pos
    
        return closest_position


    def distance(ObjectOnePosition, ObjectTwoPosition):
        x1, y1 = ObjectOnePosition
        x2, y2 = ObjectTwoPosition
        distanceX = x1 - x2
        distanceY = y1 - y2
        hypDistance = math.sqrt(distanceX**2 + distanceY**2)
        return hypDistance

    def find_closest_objects(self):

        for name, new_object in vars(self.object_data).items():
            if name.startswith('current_') or (name.endswith('bouy') or name.endswith('marker')):

                new_obj_type = new_object[2]
                new_obj_pos = (new_object[0], new_object[1])
                dist_to_new_obj = UpdateDataNode.distance(self.object_data.vessel_position, new_obj_pos)

                old_closest_obj_type = self.object_data.closest_object[2]
                old_closest_obj_pos = (self.object_data.closest_object[0], self.object_data.closest_object[1])
                dist_to_old_closest_obj = UpdateDataNode.distance(self.object_data.vessel_position, old_closest_obj_pos)
                
                old_second_closest_obj_type = self.object_data.second_closest_object[2]
                old_second_closest_obj_pos = (self.object_data.second_closest_object[0], self.object_data.second_closest_object[1])
                dist_to_old_second_closest_obj = UpdateDataNode.distance(self.object_data.vessel_position, old_second_closest_obj_pos)
            
            if dist_to_new_obj < dist_to_old_closest_obj:
                self.object_data.second_closest_object = (dist_to_old_closest_obj, old_closest_obj_type)
                self.object_data.closest_object = (dist_to_new_obj, new_obj_type)
            elif dist_to_new_obj < dist_to_old_second_closest_obj:
                self.object_data.second_closest_object = (dist_to_new_obj, new_obj_type)
                self.object_data.closest_object = (dist_to_old_closest_obj, old_closest_obj_type)
            else: #No new closest objects, but calculating distance to the old closest objects again
                self.object_data.second_closest_object = (dist_to_old_second_closest_obj, old_second_closest_obj_type)
                self.object_data.closest_object = (dist_to_old_closest_obj, old_closest_obj_type)


if __name__ == '__main__':
    try:
        UpdateDataNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

