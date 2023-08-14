#!/usr/bin/python3

import rospy
import smach
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion
from vortex_msgs.msg import DetectedObjectArray, DetectedObject
from ../finite_state_machine/scripts/lqr_interface import LQRInterface


class Maneuvering1(smach.State):
    def __init__(self, userdata):
        smach.State.__init__(self,
                             outcomes=['maneuvering1', 'maneuvering2', 'maneuvering3' 'stop'],
                             input_keys=['object_search_attempts'],
                             output_keys=['object_search_attempts'])

        self.task = "Buoy"
        self.odom = Odometry()
        self.rate = rospy.Rate(10)

        self.wp_goal2 = (0,0)

        self.sea_marker_list = []


        userdata.closest_object = (math.inf, math.inf, '')

        rospy.Subscriber('sea_markers', DetectedObjectArray, self.data_cb)
        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)


    def odom_cb(self, msg):
        self.odom = msg


    def data_cb(self, msg):
        self.sea_marker_list = msg.DetectedObjectArray


    def execute(self, userdata):

        if (rospy.get_param("/tasks/sea_marker_task1") == True):
            #Code for task 1 here
            LQRInterface.add_point(self.wp_goal2)
            self.set_wp_to_avoid_objects()
            return 'maneuvering1'

        elif (rospy.get_param("/tasks/sea_marker_task2") == True):
            return 'maneuvering1'

        elif (rospy.get_param("/tasks/sea_marker_task3") == True):
            return 'maneuvering3'

        return 'stop'


    def set_wp_to_avoid_objects(self, userdata):

        #Find path to goal from boat position
        position = self.odom.pose.pose.position
        vector = calculate_vector(position, (self.wp_goal2[0], self.wp_goal2[1]))
        Intermediate_coordiante = (position[0]+vector[0]/2, position[1]+vector[1]/2)
        

        #Find necessary waypoints between the boat and the wp_goal,
        #such that the vessel do not get to close to a sea marker.
        for sea_marker in len(self.sea_marker_list):
            if (sea_marker[2].endswith('bouy')):
                radius1 = 1.2
                radius2 = 1.5
                distance_tolerance = 0.2

                #Check if intermediate coordiante is inside small safety circle.
                sea_marker_coordinate = (sea_marker[0], sea_marker[1])
                distance1 = calculate_distance(Intermediate_coordiante, sea_marker_coordinate)
                if (distance1 < radius1):
                    
                    tangent_points = calculate_tangent_points(sea_marker[0], sea_marker[1], radius2, position[0], position[1])
                    distance_pos_to_tangent_pos = calculate_distance(position, tangent_points[0])
                    if distance_pos_to_tangent_pos < distance_tolerance:
                        tangent_points = calculate_tangent_points(sea_marker[0], sea_marker[1], radius1, position[0], position[1])

                    for i, point in enumerate(tangent_points):
                        
                        if sea_marker[2].startswith('red') and i == 0:
                            LQRInterface.clear_all_waypoints()
                            LQRInterface.add_point(point)

                        elif sea_marker[2].startswith('green') and i == 1:
                            LQRInterface.clear_all_waypoints()
                            LQRInterface.add_point(point)

                
                #if so, reset wp list and make intermediete waypoint using big safety circle 


                #Check if path goes through small safty circle around bouy

                #if so, reset wp list and make another intermediete waypoint using small safety circle
                pass


class Maneuvering2(smach.State):
    def __init__(self, userdata):
        smach.State.__init__(self,
                             outcomes=['maneuvering1', 'maneuvering2', 'maneuvering3' 'stop'],
                             input_keys=['object_search_attempts'],
                             output_keys=['object_search_attempts'])

        self.task = "Buoy"
        self.odom = Odometry()
        self.rate = rospy.Rate(10)

        self.wp_goal2 = (0,0)

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)


    def odom_cb(self, msg):
        self.odom = msg


    def data_cb(self, msg):
        self.sea_marker_list = msg.DetectedObjectArray


    def execute(self, userdata):

        if (rospy.get_param("/tasks/sea_marker_task1") == True):
            return 'maneuvering1'

        elif (rospy.get_param("/tasks/sea_marker_task2") == True):
            #Code for task 2 here
            return 'maneuvering2'

        elif (rospy.get_param("/tasks/sea_marker_task3") == True):
            return 'maneuvering3'

        return 'stop'


class Maneuvering3(smach.State):
    def __init__(self, userdata):
        smach.State.__init__(self,
                             outcomes=['maneuvering', 'stop'],
                             input_keys=['object_search_attempts'],
                             output_keys=['object_search_attempts'])

        self.task = "Buoy"
        self.odom = Odometry()
        self.rate = rospy.Rate(10)

        self.wp_goal2 = (0,0)

        self.sea_marker_list = []

        userdata.closest_object = (math.inf, math.inf, '')

        rospy.Subscriber('sea_markers', DetectedObjectArray, self.data_cb)
        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)


    def odom_cb(self, msg):
        self.odom = msg


    def data_cb(self, msg):
        self.sea_marker_list = msg.DetectedObjectArray


    def execute(self, userdata):

        if (rospy.get_param("/tasks/sea_marker_task1") == True):
            return 'maneuvering'

        elif (rospy.get_param("/tasks/sea_marker_task2") == True):
            return 'maneuvering'

        elif (rospy.get_param("/tasks/sea_marker_task3") == True):
            #Code for task 3 here
            return 'maneuvering'

        return 'stop'


    def set_wp_to_avoid_objects(self, userdata):

        #Find path between vessel and given GPS-point 

        #Find necessary waypoints between the boat and the wp_goal,
        #such that the vessel do not get to close to a sea marker.
        for name, new_object in vars(self.data).items():
            if name.startswith('current') or (name.endswith('bouy') or name.endswith('marker')):

                pass


def calculate_vector(coord1, coord2):
    x_diff = coord2[0] - coord1[0]
    y_diff = coord2[1] - coord1[1]
    vector = (x_diff, y_diff)
    return vector


def calculate_distance(coord1, coord2):
    x_diff = coord2[0] - coord1[0]
    y_diff = coord2[1] - coord1[1]
    distance = math.sqrt(x_diff ** 2 + y_diff ** 2)
    return distance

def calculate_tangent_points(Cx, Cy, r, Px, Py):
    dx, dy = Px - Cx, Py - Cy
    dxr, dyr = -dy, dx
    d = math.sqrt(dx ** 2 + dy ** 2)
    
    tangent_points = []

    if d >= r:
        rho = r / d
        ad = rho ** 2
        bd = rho * math.sqrt(1 - rho ** 2)
        T1x = Cx + ad * dx + bd * dxr
        T1y = Cy + ad * dy + bd * dyr
        T2x = Cx + ad * dx - bd * dxr
        T2y = Cy + ad * dy - bd * dyr

        tangent_points.append((T1x, T1y))
        tangent_points.append((T2x, T2y))

    return tangent_points