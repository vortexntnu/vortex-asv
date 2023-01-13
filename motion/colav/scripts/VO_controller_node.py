#!/usr/bin/python3

from velocity_obstacle_node import Velocity_Obstacle
import rospy
from nav_msgs.msg import Odometry
from std_srvs import Trigger,TriggerResponse

class VO_controller_node:

    """
    The velocity object controller node.
    Changes the ASV velocity such that a collision cannot occur.

    """
    

    def __init__(self):
        rospy.init_node("colav")

        self.vessel = Odometry()
        self.obstacle = Odometry()


        # Subscribers
        self.vessel_sub = rospy.Subscriber(
            "/pose_gt", Odometry, self.vessel_callback, queue_size=1
        )  # 20hz

        self.obstacle_sub = rospy.Subscriber(
            "/obstacle_gt", Odometry, self.obstacle_callback, queue_size=1
        )  # 20hz

        # Service
        self.colav_srv = rospy.Service('colav',Trigger,self.avoid_collision)



    def avoid_collision(self,trigger):

        VO = Velocity_Obstacle(None,self.obstacle,self.vessel)
        while VO.check_if_collision():
            ref_velocity = VO.choose_velocity()
            ###### Give ref_velocity to velocity controller ######
        

    def vessel_callback(self,data):
        self.vessel = data


    def obstacle_callback(self,data):
        self.obstacle = data


if __name__ == "__main__":
    try:
        node = VO_controller_node()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
