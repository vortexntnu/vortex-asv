#!/usr/bin/python3
#Written by Sigurd von Brandis, Student

import rospy
import smach
import math
from vortex_msgs.srv import Waypoint, WaypointRequest, WaypointResponse
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion
from vortex_msgs.msg import DetectedObjectArray, DetectedObject
from update_objects_data import DetectedObjectsData, UpdateDataNode


class Idle(smach.State):
    # TODO: Idle must stop the vessel before entering search state.
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['detectedObjectsNavigation', 'search', 'stop'],  #, 'stop'
            input_keys=['closest_object', 'object_search_attempts'],
            output_keys=['object_search_attempts'])

    def execute(self, userdata):
        rospy.loginfo('Executing Idle')

        if userdata.closest_object[1] == '':
            if userdata.object_search_attempts >= math.inf: #Change to 5 later
                return 'stop'
            else:
                return 'search'
        else:
            userdata.object_search_attempts = 0
            return 'detectedObjectsNavigation'


class Search(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['detectedObjectsNavigation'],
                             input_keys=['object_search_attempts'],
                             output_keys=['object_search_attempts'])

        self.task = "Buoy"
        self.odom = Odometry()
        self.rate = rospy.Rate(10)

        # Publisher to heading controller
        self.heading_pub = rospy.Publisher(
            "/guidance_interface/desired_heading", Float64, queue_size=1)

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)

    def odom_cb(self, msg):
        self.odom = msg

    # TODO: should be generalized and added to shared package
    def within_acceptance_margins(setpoint, current):
        error = abs(setpoint - current)
        if error < 0.1:
            return True
        return False

    # Turns ASV by specified angle
    def yaw_to_angle(self, angle):
        # Find heading corresponding to the change in angle
        orientation = self.odom.pose.pose.orientation
        orientation_list = [
            orientation.x, orientation.y, orientation.z, orientation.w
        ]
        yaw = euler_from_quaternion(orientation_list)[2]
        heading_goal = yaw + angle

        # publishes heading to heading controller, and waits until the new heading is reached
        self.heading_pub.Publish(heading_goal)
        print(f"Searching for {self.task}, angle: ({angle}) ...")
        while not self.within_acceptance_margins(heading_goal, yaw):
            self.rate.sleep()
            orientation = self.odom.pose.pose.orientation
            orientation_list = [
                orientation.x, orientation.y, orientation.z, orientation.w
            ]
            yaw = euler_from_quaternion(orientation_list)[2]

    def execute(self, userdata):
        rospy.loginfo('Executing Search')

        #Turns side to side
        self.yaw_to_angle(45)
        self.yaw_to_angle(-90)
        self.yaw_to_angle(45)

        userdata.object_search_attempts += 1

        return 'detectedObjectsNavigation'


class DetectedObjectsNavigation(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['idle'],
                             output_keys=['closest_object'])
        #self.enabled = rospy.get_param("/tasks/maneuvering_navigation_tasks")
        self.data = DetectedObjectsData()
        self.closest_object = (math.inf, math.inf, '')
        self.second_closest_object = (math.inf, math.inf, '')
        self.third_closest_object = (math.inf, math.inf, '')
        self.fourth_closest_object = (math.inf, math.inf, '')

        self.DistanceRadius = 3
        self.DirectionWithLeia = True
        self.ObjectSearchAttempts = 0

        self.data_sub = rospy.Subscriber('detected_objects',
                                         DetectedObjectArray, self.data_cb)
        self.vessel_pos_sub = rospy.Subscriber('/odometry/filtered', Odometry,
                                               self.odom_cb)

    def execute(self, userdata):

        while self.closest_object != '':  # and self.enabled == True:

            self.find_closest_objects()

            #Make new path based on the two closest bouys beeing green and red.
            if self.closest_object[2] == 'red' and self.second_closest_object[
                    2] == 'green' or self.closest_object[
                        2] == 'green' and self.second_closest_object[
                            2] == 'red':
                midpoint = (
                    (self.closest_object[0] + self.second_closest_object[0]) /
                    2,
                    (self.closest_object[1] + self.second_closest_object[1]) /
                    2)
                self.send_wp(self.data.vessel_position)
                self.overwrite_with_new_waypoint(midpoint)
                #Add new waypoint to path based on third and fouth closest objects beeing green and red, or if we just have a third closest object.
                if self.third_closest_object[
                        2] == 'red' and self.fourth_closest_object[
                            2] == 'green' or self.third_closest_object[
                                2] == 'green' and self.fourth_closest_object[
                                    2] == 'red':
                    midpoint2 = ((self.third_closest_object[0] +
                                  self.fourth_closest_object[0]) / 2,
                                 (self.third_closest_object[1] +
                                  self.fourth_closest_object[1]) / 2)
                    self.send_wp(midpoint2)
                elif self.third_closest_object != '':
                    next_waypoint = self.NavAroundOneObject(
                        self.data.vessel_position, self.third_closest_object,
                        self.DistanceRadius, self.DirectionWithLeia)
                    self.send_wp(next_waypoint)

            #Make new path based on having a closest object and perheps also a second closest object (which is not green and red)
            elif self.closest_object[2] != '':
                next_waypoint = self.NavAroundOneObject(
                    self.data.vessel_position, self.closest_object,
                    self.DistanceRadius, self.DirectionWithLeia)
                self.send_wp(self.data.vessel_position)
                self.overwrite_with_new_waypoint(next_waypoint)

                if self.second_closest_object[
                        2] == 'red' and self.third_closest_object[
                            2] == 'green' or self.second_closest_object[
                                2] == 'green' and self.third_closest_object[
                                    2] == 'red':
                    midpoint = ((self.second_closest_object[0] +
                                 self.third_closest_object[0]) / 2,
                                (self.second_closest_object[1] +
                                 self.third_closest_object[1]) / 2)
                    self.send_wp(midpoint)
                elif self.second_closest_object != '':
                    next_waypoint = self.NavAroundOneObject(
                        self.data.vessel_position, self.second_closest_object,
                        self.DistanceRadius, self.DirectionWithLeia)
                    self.send_wp(next_waypoint)

        userdata['closest_object'] = self.closest_object
        return 'idle'

    def data_cb(self, msg):
        #TODO:Much duplication that should be switched out.
        detected_objects = msg.detected_objects
        self.data.current_red_bouy = (detected_objects[0].x,
                                      detected_objects[0].y,
                                      detected_objects[0].type)
        self.data.next_red_bouy = (detected_objects[1].x,
                                   detected_objects[1].y,
                                   detected_objects[1].type)
        self.data.current_green_bouy = (detected_objects[2].x,
                                        detected_objects[2].y,
                                        detected_objects[2].type)
        self.data.next_green_bouy = (detected_objects[3].x,
                                     detected_objects[3].y,
                                     detected_objects[3].type)
        self.data.current_north_marker = (detected_objects[4].x,
                                          detected_objects[4].y,
                                          detected_objects[4].type)
        self.data.current_south_marker = (detected_objects[5].x,
                                          detected_objects[5].y,
                                          detected_objects[5].type)
        self.data.current_east_marker = (detected_objects[6].x,
                                         detected_objects[6].y,
                                         detected_objects[6].type)
        self.data.current_west_marker = (detected_objects[7].x,
                                         detected_objects[7].y,
                                         detected_objects[7].type)

    def odom_cb(self, msg):
        self.data.vessel_position = (msg.pose.pose.position.x,
                                     msg.pose.pose.position.y)

    def find_closest_objects(self):
        self.closest_object = (math.inf, math.inf, '')
        self.second_closest_object = (math.inf, math.inf, '')
        self.third_closest_object = (math.inf, math.inf, '')
        self.fourth_closest_object = (math.inf, math.inf, '')

        for name, new_object in vars(self.data).items():
            if name.startswith('current') or (name.endswith('bouy')
                                              or name.endswith('marker')):
                #New object
                new_obj_pos = (new_object[0], new_object[1])
                dist_to_new_obj = UpdateDataNode.distance(
                    self.data.vessel_position, new_obj_pos)
                #Old closest object
                old_closest_obj_pos = (self.closest_object[0],
                                       self.closest_object[1])
                dist_to_old_closest_obj = UpdateDataNode.distance(
                    self.data.vessel_position, old_closest_obj_pos)
                #Old second closest object
                old_second_closest_obj_pos = (self.second_closest_object[0],
                                              self.second_closest_object[1])
                dist_to_old_second_closest_obj = UpdateDataNode.distance(
                    self.data.vessel_position, old_second_closest_obj_pos)
                #Old third closest object
                old_third_closest_obj_pos = (self.third_closest_object[0],
                                             self.third_closest_object[1])
                dist_to_old_third_closest_obj = UpdateDataNode.distance(
                    self.data.vessel_position, old_third_closest_obj_pos)
                #Old fourth closest object
                old_fourth_closest_obj_pos = (self.fourth_closest_object[0],
                                              self.fourth_closest_object[1])
                dist_to_old_fourth_closest_obj = UpdateDataNode.distance(
                    self.data.vessel_position, old_fourth_closest_obj_pos)
            #Update closest objects each iteration using new_object
            if dist_to_new_obj < dist_to_old_closest_obj:
                self.fourth_closest_object = self.third_closest_object
                self.third_closest_object = self.second_closest_object
                self.second_closest_object = self.closest_object
                self.closest_object = new_object
            elif dist_to_new_obj < dist_to_old_second_closest_obj:
                self.fourth_closest_object = self.third_closest_object
                self.third_closest_object = self.second_closest_object
                self.second_closest_object = new_object
            elif dist_to_new_obj < dist_to_old_third_closest_obj:
                self.fourth_closest_object = self.third_closest_object
                self.third_closest_object = new_object
            elif dist_to_new_obj < dist_to_old_fourth_closest_obj:
                self.fourth_closest_object = new_object

    def NavAroundOneObject(ASVPos, object, radius, directionWithLeia):
        """
            object (B)
                   /|
                  / |
                 /  | radius
                /   |
               /    |
              /     |
        ASV (A)----(C) WP
    
        """
        objectType = object[2]
        xWP = 0
        yWP = 0

        if directionWithLeia == True:
            # Calculate the coordinates of next waypoint.
            xAC = ASVPos[0] - object[0]  #xB - xA
            yAC = ASVPos[1] - object[1]  #yB - yA
            AC_length = math.sqrt(xAC**2 + yAC**2)
            xAC_normalized = xAC / AC_length
            yAC_normalized = yAC / AC_length
            if objectType == 'red' or objectType == 'west' or objectType == 'south':
                # Adjust the sign of the vector AC for a reflex angle
                if xAC_normalized > 0:
                    xAC_normalized *= -1
                if yAC_normalized > 0:
                    yAC_normalized *= -1
            xWP = object[0] + radius * xAC_normalized
            yWP = object[1] + radius * yAC_normalized
        else:
            # Calculate the coordinates of next waypoint.
            xAC = ASVPos[0] - object[0]  #xB - xA
            yAC = ASVPos[1] - object[1]  #yB - yA
            AC_length = math.sqrt(xAC**2 + yAC**2)
            xAC_normalized = xAC / AC_length
            yAC_normalized = yAC / AC_length
            if objectType == 'green' or objectType == 'east' or objectType == 'north':
                # Adjust the sign of the vector AC for a reflex angle
                if xAC_normalized > 0:
                    xAC_normalized *= -1
                if yAC_normalized > 0:
                    yAC_normalized *= -1
            xWP = object[0] + radius * xAC_normalized
            yWP = object[1] + radius * yAC_normalized

        return (xWP, yWP)

    #Add single waypoint to current waypoint list
    def send_wp(waypoint_in):
        wp = WaypointRequest()
        wp.waypoint = waypoint_in
        response = WaypointResponse()
        response.success = False
        try:
            rospy.wait_for_service("/navigation/add_waypoint")
            waypoint_client = rospy.ServiceProxy("/navigation/add_waypoint",
                                                 Waypoint)
            response = waypoint_client(wp)
        except rospy.ServiceException as e:
            print("Service call failed: {}".format(e))
        if response.success:
            rospy.loginfo(f"Waypoint {wp.waypoint} sent successfully!")
        else:
            rospy.logwarn(f"Waypoint {wp.waypoint} could not be set!")

    #Remove all waypoints, but not the last one, in waypointlist, and add a new waypoint.
    def overwrite_with_new_waypoint(waypoint_in):
        wp = WaypointRequest()
        wp.waypoint = waypoint_in
        response = WaypointResponse()
        response.success = False
        try:
            rospy.wait_for_service(
                "/navigation/overwrite_waypoint_list_with_new_waypoint")
            overwrite_waypoint_client = rospy.ServiceProxy(
                "/navigation/overwrite_waypoint_list_with_new_waypoint",
                Waypoint)
            response = overwrite_waypoint_client(wp)
        except rospy.ServiceException as e:
            print("Service call failed: {}".format(e))

        if response.success:
            rospy.loginfo(f"Waypoint {wp.waypoint} sent successfully!")
        else:
            rospy.logwarn(f"Waypoint {wp.waypoint} could not be set!")
