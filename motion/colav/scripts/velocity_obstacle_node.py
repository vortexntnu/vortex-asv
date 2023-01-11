#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Point,Vector3
from nav_msgs.msg import Odometry
import math

class Velocity_Obstacle:
    """
    The Velocity Obstacle class

        obstacle: An odometry of the object to avoid
        radius_o: The radius of obstacle
        vessel: An odometry of the UAV vessel 
    """

    def __init__(self, radius_o, obstacle,vessel):
    
        self.vessel = Odometry()
        self.vessel = vessel
        self.radius_r = None #placeholder
        
        self.radius_o = radius_o # maybe not an input
        self.obstacle = Odometry()
        self.obstacle = obstacle
        
        self.left_angle = 0.0
        self.right_angle = 0.0
        

    def set_cone_angles(self):
        """
        Calculates the largest and smallest heading-angle where a collision can occur 
        """
        point = self.obstacle.pose.pose.position
        theta_ro = math.atan2(point.y/point.x)
        theta_ray = math.asin((self.radius_o+self.radius_r)/(math.sqrt(point.x**2+point.y**2)))
        self.right_angle = theta_ro-theta_ray
        self.left_angle = theta_ro + theta_ray 

    def check_if_collision(self):
        """
        Returns true if the current velocity results in a collision.
        """
        velocity_r = self.vessel.twist.twist.linear
        velocity_o = self.obstacle.twist.twist.linear
        translated_vel = Vector3(velocity_r.x-velocity_o.x,velocity_r.y-velocity_o.y,0)
        angle = math.atan2(translated_vel.y/translated_vel.x)
        return angle > self.right_angle and angle < self.left_angle 

    #Elias sin tentative løsning, ja riktig tentativ    

    def choose_velocity(self):

        """
        
        Determines a reference velocity for the velocity controller
 

        """


        buffer_angle = None #placeholder
        velocity_r = self.vessel.twist.twist.linear
        abs_vel = math.sqrt(velocity_r.x**2+velocity_r.y**2)
        point = self.obstacle.pose.pose.position
        theta_ro = math.atan2(point.y/point.x)

        #Determine the direction of the obstacle, and which cone angle to follow

        if theta_ro> 0 and math.atan2(velocity_r.y/velocity_r.x) < math.pi/2 and math.atan2(velocity_r.y/velocity_r.x) > -math.pi/2 :
            new_angle = self.left_angle + buffer_angle 
        else:
            new_angle = self.right_angle - buffer_angle
        return Vector3(math.cos(new_angle)*abs_vel,math.sin(new_angle)*abs_vel)
        


        



        