#!/usr/bin/python3

#This whole file is malplaced, as its correct position is yet to be determined
import rospy
from nav_msgs.msg import Odometry
from vortex_msgs.srv import Waypoint, WaypointRequest
from math import sqrt
from std_srvs import SetBool, SetBoolRequest,Trigger,TriggerRequest



class Colav_fsm:

    def __init__(self):
        rospy.init_node("colav_fsm",anonymous=True)
        self.vessel = Odometry()
        self.obstacle = Odometry()


        #Subscribers
        #placeholder topic
        self.goal_sub = rospy.Subscriber(
            "goaltopic",Odometry,self.run_fsm,queue_size=1
        )
        
        self.vessel_sub = rospy.Subscriber(
            "/pose_gt", Odometry, self.vessel_callback, queue_size=1
        )  # 20hz



    def vessel_callback(self,data):
        self.vessel = data

    def obstacle_callback(self,data):
        self.obstacle = data


    #could use som failsafing
    def run_fsm(self,data):
        wp = WaypointRequest()
        wp.waypoint = [data.pose.pose.position.x, data.pose.pose.position.y]
        vessel_wp = WaypointRequest()
        vessel_wp.waypoint = [self.vessel.pose.pose.position.x,self.vessel.pose.pose.position.y]
        waypoint_client = rospy.ServiceProxy("/navigation/add_waypoint", Waypoint)
        waypoint_client(vessel_wp)
        waypoint_client(wp)
        pause_los_client = rospy.ServiceProxy("pause",SetBool)
        pause_los = SetBoolRequest()
        call_colav_client = rospy.ServiceProxy("colav",Trigger)
        colav_trigger = TriggerRequest()

        #placeholder
        danger_zone_r = 666
        while True:
            if sqrt((self.vessel.pose.pose.position.x-self.obstacle.pose.pose.position.x)**2+(self.vessel.pose.pose.position.y-self.obstacle.pose.pose.position.y)**2) > danger_zone_r:
               pause_los.data  = True
               pause_los_client(pause_los)
               call_colav_client(colav_trigger)
               pause_los.data = False
               pause_los_client(pause_los)
        






if __name__ == "__main__":
    try:
        node = Colav_fsm()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
