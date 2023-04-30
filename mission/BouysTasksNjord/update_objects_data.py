#!/usr/bin/python3
#Written by Sigurd von Brandis, Student

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import numpy as np
import math
from vortex_msgs.msg import DetectedObjectArray, DetectedObject
from std_msgs.msg import Header


class DetectedObjectsData:
    #Latitude; x, Longitude; y
    def __init__(self):  #  x, y, type
        self.current_red_bouy = (0, 0, 'red')
        self.current_green_bouy = (0, 0, 'green')
        self.current_north_marker = (0, 0, 'north')
        self.current_south_marker = (0, 0, 'south')
        self.current_east_marker = (0, 0, 'east')
        self.current_west_marker = (0, 0, 'west')
        self.vessel_position = (0, 0, 'vessel')


class UpdateDataNode:

    def __init__(self):
        # Initialize object data to be published
        self.object_data = DetectedObjectsData()

        self.red_bouy_array = []
        self.green_bouy_array = []
        self.north_marker_array = []
        self.south_marker_array = []
        self.east_marker_array = []
        self.west_marker_array = []

        # Initialize subscriber and Service to get all the necessary information
        self.Obj_pos_sub = rospy.Subscriber('bouys_and_markers',
                                            DetectedObjectArray,
                                            self.obj_pos_cb)
        self.Position_sub = rospy.Subscriber('/odometry/filtered', Odometry,
                                             self.odom_cb)

        # Initialize publisher to data topic
        self.pub = rospy.Publisher('detected_objects',
                                   DetectedObjectArray,
                                   queue_size=1)

    def spin(self):

        VesselPos = self.object_data.vessel_position

        self.object_data.current_red_bouy = UpdateDataNode.find_closest_object_in_array(
            VesselPos, self.red_bouy_array)
        self.object_data.current_green_bouy = UpdateDataNode.find_closest_object_in_array(
            VesselPos, self.green_bouy_array)
        # find closest north, south, east and west marker.

        UpdateDataNode.find_closest_objects()

        msg = DetectedObjectArray()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        red_bouy = DetectedObject()
        red_bouy.x = self.object_data.current_red_bouy[0]
        red_bouy.y = self.object_data.current_red_bouy[1]
        red_bouy.type = self.object_data.current_red_bouy[2]
        msg.detected_objects.append(red_bouy)

        green_bouy = DetectedObject()
        green_bouy.x = self.object_data.current_green_bouy[0]
        green_bouy.y = self.object_data.current_green_bouy[1]
        green_bouy.type = self.object_data.current_green_bouy[2]
        msg.detected_objects.append(green_bouy)

        # Update msg with current north, south, east and  west marker.

        self.pub.publish(msg)

    def odom_cb(self, msg):
        self.object_data.vessel_position = (msg.pose.pose.position.x,
                                            msg.pose.pose.position.y, 'vessel')

    #Must probably be switched out when message is defined
    def obj_pos_cb(self, msg):
        
        self.red_bouy_array = []
        self.green_bouy_array = []
        self.north_marker_array = []
        self.south_marker_array = []
        self.east_marker_array = []
        self.west_marker_array = []

        for detected_object in len(msg.array):
            if detected_object[2] == 'red':
                self.red_bouy_array.append(detected_object)
            elif detected_object[2] == 'green':
                self.green_bouy_array.append(detected_object)
            elif detected_object[2] == 'north':
                self.north_marker_array.append(detected_object)
            elif detected_object[2] == 'south':
                self.south_marker_array.append(detected_object)
            elif detected_object[2] == 'east':
                self.east_marker_array.append(detected_object)
            elif detected_object[2] == 'west':
                self.west_marker_array.append(detected_object)
            else:
                rospy.loginfo("Received unsupported object type")

    # def update_array(array, new_point):
    #     updated = False
    #     for i, existing_point in enumerate(array):
    #         distance = UpdateDataNode.distance(new_point, existing_point)
    #         if distance <= 1:
    #             array[i] = new_point
    #             updated = True
    #             break
    #     if not updated:
    #         array = np.vstack([array, new_point])

    #     return array

    def find_closest_object_in_array(position, positions_array):
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


if __name__ == '__main__':
    try:
        rospy.init_node("update_Njord_data")
        UpdateDataNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
