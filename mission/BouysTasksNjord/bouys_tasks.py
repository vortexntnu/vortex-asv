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
    # Need to be added that idle make the vessel stop moving, before entering search state.
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['decideNextState', 'search'],  #, 'stop'
            input_keys=['closest_object', 'object_search_attempts'],
            output_keys=['object_search_attempts'])

    def execute(self, userdata):
        rospy.loginfo('Executing Idle')

        if userdata.closest_object[1] == '':
            if userdata.object_search_attempts >= 5:
                return 'stop'
            else:
                return 'search'
        else:
            userdata.object_search_attempts = 0
            return 'decideNextState'


class Search(smach.State):
    # Must be changed so it returns object search attempts. Should be done here, not in idle state.
    def __init__(self):
        smach.State.__init__(
            self, 
            outcomes=['idle'], 
            input_keys=['object_search_attempts'], 
            output_keys=['object_search_attempts'])
        
        self.task = "Buoy"
        self.odom = Odometry()
        self.rate = rospy.Rate(10)

        self.heading_pub = rospy.Publisher(
            "/guidance_interface/desired_heading", Float64, queue_size=1)

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)

    def odom_cb(self, msg):
        self.odom = msg

    def within_acceptance_margins(setpoint, current):
        error = abs(setpoint - current)
        if error < 0.1:
            return True
        return False

    def yaw_to_angle(self, angle):
        orientation = self.odom.pose.pose.orientation
        orientation_list = [
            orientation.x, orientation.y, orientation.z, orientation.w
        ]
        yaw = euler_from_quaternion(orientation_list)[2]
        heading_goal = yaw + angle

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

        self.yaw_to_angle(45)
        self.yaw_to_angle(-90)
        self.yaw_to_angle(45)
        
        userdata.object_search_attempts += 1

        return 'idle'


class DetectedObjectsNavigation():
    #In future data = closest_data, and we sould also find second_closest_data, to be able to set 2 insted of one waypoint.
    def __init__(self, outcomes=['idle'], input_keys=[], output_keys=['closest_object']):
        #self.enabled = rospy.get_param("/tasks/maneuvering_navigation_tasks")
        self.data = DetectedObjectsData()
        self.closest_object = (math.inf, '')
        self.second_closest_object = (math.inf, '')
        self.third_closest_object = (math.inf, '')
        self.fourth_closest_object = (math.inf, '')

        self.DistanceRadius = 3
        self.DirectionWithLeia = True
        self.ObjectSearchAttempts = 0

        self.data_sub = rospy.Subscriber('detected_objects',
                                         DetectedObjectArray, self.data_cb)
        self.vessel_pos_sub = rospy.Subscriber('/odometry/filtered', Odometry,
                                               self.odom_cb)

    #Contains lots of information that is duplicated. Should be switched out.
    def execute(self, userdata):

        self.find_closest_objects()

        while self.closest_object != '':  # and self.enabled == True:

            if self.closest_object[1] == 'red' and self.second_closest_object[
                    1] == 'green' or self.closest_object[
                        1] == 'green' and self.second_closest_object[
                            1] == 'red':
                GreenBouy = self.data.current_green_bouy
                RedBouy = self.data.current_red_bouy
                midpoint = ((GreenBouy[0] + RedBouy[0]) / 2,
                            (GreenBouy[1] + RedBouy[1]) / 2)
                self.send_wp(self.data.vessel_position)
                self.overwrite_with_new_waypoint(midpoint)

                if self.third_object[1] == 'red' and self.fourth_closest_object[
                        1] == 'green' or self.third_object[
                            1] == 'green' and self.fourth_closest_object[
                                1] == 'red':
                    GreenBouy = self.data.next_green_bouy
                    RedBouy = self.data.next_red_bouy
                    midpoint = ((GreenBouy[0] + RedBouy[0]) / 2,
                                (GreenBouy[1] + RedBouy[1]) / 2)
                    self.send_wp(midpoint)

                if self.third_object[1] == 'red':
                    next_waypoint = self.NavAroundOneObject(
                        self.data.vessel_position, self.data.next_red_bouy,
                        self.DistanceRadius, self.DirectionWithLeia)
                    self.send_wp(next_waypoint)

                if self.third_object[1] == 'green':
                    next_waypoint = self.NavAroundOneObject(
                        self.data.vessel_position, self.data.next_green_bouy,
                        self.DistanceRadius, self.DirectionWithLeia)
                    self.send_wp(next_waypoint)

            elif self.closest_object[1] == 'red':
                next_waypoint = self.NavAroundOneObject(
                    self.data.vessel_position, self.data.current_red_bouy,
                    self.DistanceRadius, self.DirectionWithLeia)
                self.send_wp(self.data.vessel_position)
                self.overwrite_with_new_waypoint(next_waypoint)

                if self.second_closest_object == 'red':
                    next_waypoint = self.NavAroundOneObject(
                        self.data.vessel_position, self.data.next_red_bouy,
                        self.DistanceRadius, self.DirectionWithLeia)
                    self.send_wp(next_waypoint)

            elif self.closest_object[1] == 'green':
                next_waypoint = self.NavAroundOneObject(
                    self.data.vessel_position, self.data.current_green_bouy,
                    self.DistanceRadius, self.DirectionWithLeia)
                self.send_wp(userdata['vessel_position'])
                self.overwrite_with_new_waypoint(next_waypoint)

                if self.second_closest_object == 'green':
                    next_waypoint = self.NavAroundOneObject(
                        self.data.vessel_position, self.data.next_green_bouy,
                        self.DistanceRadius, self.DirectionWithLeia)
                    self.send_wp(next_waypoint)

            elif self.closest_object[1] == 'north':
                pass
            elif self.closest_object[1] == 'south':
                pass
            elif self.closest_object[1] == 'east':
                pass
            elif self.closest_object[1] == 'west':
                pass

        return 'idle'

    def data_cb(self, msg):
        #Much duplication that should be switched out.
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

    #Must be updated to also find third and fouth closes objects
    def find_closest_objects(self):
        for name, new_object in vars(self.data).items():
            if name.startswith('current_') or (name.endswith('bouy')
                                               or name.endswith('marker')):

                new_obj_type = new_object[2]
                new_obj_pos = (new_object[0], new_object[1])
                dist_to_new_obj = UpdateDataNode.distance(
                    self.data.vessel_position, new_obj_pos)

                old_closest_obj_type = self.closest_object[2]
                old_closest_obj_pos = (self.closest_object[0],
                                       self.closest_object[1])
                dist_to_old_closest_obj = UpdateDataNode.distance(
                    self.data.vessel_position, old_closest_obj_pos)

                old_second_closest_obj_type = self.second_closest_object[2]
                old_second_closest_obj_pos = (self.second_closest_object[0],
                                              self.second_closest_object[1])
                dist_to_old_second_closest_obj = UpdateDataNode.distance(
                    self.data.vessel_position, old_second_closest_obj_pos)

            if dist_to_new_obj < dist_to_old_closest_obj:
                self.second_closest_object = (dist_to_old_closest_obj,
                                              old_closest_obj_type)
                self.closest_object = (dist_to_new_obj, new_obj_type)

            elif dist_to_new_obj < dist_to_old_second_closest_obj:
                self.second_closest_object = (dist_to_new_obj, new_obj_type)
                self.closest_object = (dist_to_old_closest_obj,
                                       old_closest_obj_type)
            else:  #No new closest objects, but updating distance to the old closest objects again because our position may have changed
                self.second_closest_object = (dist_to_old_second_closest_obj,
                                              old_second_closest_obj_type)
                self.closest_object = (dist_to_old_closest_obj,
                                       old_closest_obj_type)

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
