#!/usr/bin/python3
#written by toskar and slasken

import rospy
from std_msgs.msg import Float64
from vortex_msgs.msg import GuidanceData
import time


class GuidanceInterface():
    """
    Input: guidance data (heading, speed,...) from nodes LOS, COLAV
    Output: desired speed and desired heading to nodes speed_controller_node and heading_controller_node
    Functionality: decides which input should be given as output 
    Subscribes to guidance data from topics:
        "/guidance/los_data"
        "/guiadance/colav_data"
    Publishes torque and force input to topics:
        "/guidance/desired_speed"  
        "/guidance/desired_heading"
        
    """ 

    def __init__(self) -> None:
        rospy.init_node("guidance_interface")

        # Subscribers
        self.los_data_sub = rospy.Subscriber(
            rospy.get_param("/guidance_interface/los_data"),
            GuidanceData,
            self.los_data_callback,
            queue_size = 1,
        )
        self.colav_data_sub = rospy.Subscriber(
            rospy.get_param("/guidance_interface/colav_data"),
            GuidanceData,
            self.colav_data_callback,
            queue_size = 1,
        )

        # Publishers
        self.speed_pub = rospy.Publisher(
            rospy.get_param("/guidance_interface/desired_speed"), Float64, queue_size = 1
        )
        self.heading_pub = rospy.Publisher(
            rospy.get_param("/guidance_interface/desired_heading"), Float64, queue_size = 1
        )
        self.pub_colav = False
        self.colav_pub_time = 5 # placeholder


    def los_data_callback(self, guidance_data) -> None:
        if self.pub_colav:
            return
        # Received desired speed and heading
        speed_msg = guidance_data.u_d
        heading_msg = guidance_data.psi_d

        self.speed_pub.publish(speed_msg)
        self.heading_pub.publish(heading_msg)
    def colav_data_callback(self,guidance_data) -> None:
        self.pub_colav = True
        speed_msg = guidance_data.u_d
        heading_msg = guidance_data.psi_d

        toc = time.perf_counter()
        tic = time.perf_counter()
        while tic-toc < self.colav_pub_time :
            tic = time.perf_counter()
            self.speed_pub.publish(speed_msg)
            self.heading_pub.publish(heading_msg)
        self.pub_colav = False

        

if __name__ == "__main__":
    guidance_interface = GuidanceInterface()
    rospy.spin()