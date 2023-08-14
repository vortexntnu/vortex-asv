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
            self.set_wp_to_avoid_objects()
            return 'maneuvering1'

        elif (rospy.get_param("/tasks/sea_marker_task2") == True):
            return 'maneuvering1'

        elif (rospy.get_param("/tasks/sea_marker_task3") == True):
            return 'maneuvering3'

        return 'stop'


    def set_wp_to_avoid_objects(self, userdata):

        #Find path between vessel and given GPS-point 

        #Find necessary waypoints between the boat and the wp_goal,
        #such that the vessel do not get to close to a sea marker.
        for name, new_object in vars(self.data).items():
            if name.startswith('current') or (name.endswith('bouy') or name.endswith('marker')):

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