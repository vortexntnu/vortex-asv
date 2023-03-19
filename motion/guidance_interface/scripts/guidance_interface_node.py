#!/usr/bin/python3

import rospy
from std_msgs.msg import Float64
from vortex_msgs.msg import GuidanceData


class GuidanceInterface():
    """
    Input: guidance data (heading, speed,...) from nodes LOS, COLAV
    Output: desired speed and desired heading to nodes speed_controller_node and heading_controller_node
    Functionality: decides which input should be given as output 


    Subscribes to guidance data from topics:
        "/guidance/los_data"

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
            self.guidance_data_callback,
            queue_size = 1,
        )

        # Publishers
        self.speed_pub = rospy.Publisher(
            rospy.get_param("/guidance_interface/desired_speed"), Float64, queue_size = 1
        )
        self.heading_pub = rospy.Publisher(
            rospy.get_param("/guidance_interface/desired_heading"), Float64, queue_size = 1
        )

    def guidance_data_callback(self, guidance_data) -> None:
        # Received desired speed and heading
        speed_msg = guidance_data.u_d
        heading_msg = guidance_data.psi_d

        self.speed_pub.publish(speed_msg)
        self.heading_pub.publish(heading_msg)
        

if __name__ == "__main__":
    guidance_interface = GuidanceInterface()
    rospy.spin()