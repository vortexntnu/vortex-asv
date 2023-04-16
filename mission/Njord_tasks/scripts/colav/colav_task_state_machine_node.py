#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from vortex_msgs.srv import Waypoint, WaypointRequest, WaypointResponse
from math import sqrt
from std_srvs.srv import SetBool, SetBoolRequest, Trigger, TriggerRequest


class ColavTask:
    """
    
    Collision avoidance node for the collison avoidance Njord tasks.
    The colav is implemented as a simple state machine, that continuously switches between 
    LOS guidance and collision avoidance. The node listens to the position of an obstacle,
    and switches mode if the obstacle comes too clos. Los guidance will then be paused, and
    collision avoidance is called through a service. The colav system will check for possible 
    collisions, and take evasive action. When the collision avoidance no longer detects 
    a possible collision, it will stop controlling the UAV. LOS then is turned on again. 
    This implementation only allows for avoidance of a single object. 
    
    """

    def __init__(self):
        """
        To start the task, an start_task_sub subscriber is implemented.
        Posting an odometry of the goal to the "goaltopic" topic, will
        cause the task to start.
        """
        rospy.init_node("colav_fsm", anonymous=True)
        self.vessel = Odometry()
        self.obstacle = Odometry()

        #Subscribers
        #placeholder topic
        self.start_task_sub = rospy.Subscriber("goaltopic",
                                               Odometry,
                                               self.run_task,
                                               queue_size=1)

        self.vessel_sub = rospy.Subscriber("/pose_gt",
                                           Odometry,
                                           self.vessel_callback,
                                           queue_size=1)  # 20hz

        self.obstacle_sub = rospy.Subscriber("/obstacle_gt",
                                             Odometry,
                                             self.vessel_callback,
                                             queue_size=1)  # 20hz

    def vessel_callback(self, data):
        """
        Update the postion of the UAV
        Args:
            data: An odometry of the UAV position
        """
        self.vessel = data

    def obstacle_callback(self, data):
        """
        Update the postion of the obstacle
        Args:
            data: An odometry of the obstacle position
        """
        self.obstacle = data

    def run_task(self, data):
        """
        
        Callback funtion for the start_task subscriber.
        Assumes that the UAV is reasonably placed before the goal is
        posted.
        Args:
            data: An odometry of the goal
        
        """

        wp = WaypointRequest()
        wp.waypoint = [data.pose.pose.position.x, data.pose.pose.position.y]
        vessel_wp = WaypointRequest()
        vessel_wp.waypoint = [
            self.vessel.pose.pose.position.x, self.vessel.pose.pose.position.y
        ]
        response = WaypointResponse()
        response.success = False
        try:
            rospy.loginfo("Sending waypoints")
            rospy.wait_for_service("/navigation/add_waypoint")
            waypoint_client = rospy.ServiceProxy("/navigation/add_waypoint",
                                                 Waypoint)
            first_response = waypoint_client(vessel_wp)
            second_response = waypoint_client(wp)
            response.success = first_response.success and second_response.success
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: {}".format(e))

        if response.success:
            rospy.loginfo(f"Waypoints sent successfully!")
        else:
            rospy.logwarn(f"Waypoints could not be set, exiting task!")

        pause_los = SetBoolRequest()
        colav_trigger = TriggerRequest()
        #should make check VO cone here
        while response is True:
            if self.vessel_in_danger_zone():
                try:
                    rospy.loginfo("Pausing LOS")
                    rospy.wait_for_service("/pause_los")
                    pause_los_client = rospy.ServiceProxy(
                        "/pause_los", SetBool)
                    pause_los.data = True
                    pause_los_client(pause_los)
                    rospy.loginfo("LOS paused!\n calling colav")
                    rospy.wait_for_service("/colav")
                    call_colav_client = rospy.ServiceProxy("/colav", Trigger)
                    call_colav_client(colav_trigger)
                    rospy.loginfo("colav exited, resuming LOS")
                    pause_los.data = False
                    pause_los_client(pause_los)
                    rospy.wait_for_service("/pause_los")
                    rospy.loginfo("LOS resumed!")

                except rospy.ServiceException as e:
                    rospy.loginfo("Service call failed: {}".format(e))

    def vessel_in_danger_zone(self):
        """
        Checks if the UAV is closer to the obstacle than some predefined radius
        """

        danger_zone_r = 666  #placeholder
        return sqrt((self.vessel.pose.pose.position.x -
                     self.obstacle.pose.pose.position.x)**2 +
                    (self.vessel.pose.pose.position.y -
                     self.obstacle.pose.pose.position.y)**2) < danger_zone_r


if __name__ == "__main__":
    try:
        node = ColavTask()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
