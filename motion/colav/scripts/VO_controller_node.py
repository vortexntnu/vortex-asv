#!/usr/bin/python3

from velocity_obstacle_node import Velocity_Obstacle
import rospy
from nav_msgs.msg import Odometry

class VO_controller_node:

    """
    The velocity object controller node.
    Changes the ASV velocity such that a collision cannot occur.

    """

    def __init__(self,vessel,obstacle):
        self.avoid_collision()
        self.vessel = vessel
        self.obstacle = obstacle


    def avoid_collision(self):        
        VO = Velocity_Obstacle(None,self.obstacle,self.vessel)
        while VO.check_if_collision():
            ref_velocity = VO.choose_velocity()
            ###### Give ref_velocity to velocity controller ######
            ###### update vessel and obstacle odometry somehow ###
        

if __name__ == "__main__":
    try:
        node = VO_controller_node()
    except rospy.ROSInterruptException:
        pass
