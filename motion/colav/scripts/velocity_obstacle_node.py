#!/usr/bin/python3

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
        self.radius_r = 1 #placeholder
        
        self.radius_o = radius_o # may not an input
        self.obstacle = Odometry()
        self.obstacle = obstacle
        
        self.left_angle = 0.0
        self.right_angle = 0.0

        self.truncated_time = 5 #placeholder
        

    def set_cone_angles(self):
        """
        Calculates the largest and smallest heading-angle where a collision can occur 
        """
        point = self.obstacle.pose.pose.position
        theta_ro = math.atan2(point.y,point.x)
        theta_ray = math.asin((self.radius_o+self.radius_r)/(math.sqrt(point.x**2+point.y**2)))
        self.right_angle = theta_ro-theta_ray
        self.left_angle = theta_ro + theta_ray 

    def check_if_collision(self):
        """
        Returns true if the current velocity results in a collision. Uses a truncated VO collision cone
        """
        point = self.obstacle.pose.pose.position
        velocity_r = self.vessel.twist.twist.linear
        velocity_o = self.obstacle.twist.twist.linear
        translated_vel = Vector3(velocity_r.x-velocity_o.x,velocity_r.y-velocity_o.y,0)
        angle = math.atan2(translated_vel.y,translated_vel.x)

        acceptance_radius = (self.radius_o + self.radius_r)/self.truncated_time
        max_truncated_veloctiy = math.sqrt(point.x**2+point.y**2)/self.truncated_time - acceptance_radius

        return angle > self.right_angle and angle < self.left_angle and math.sqrt(velocity_r.x**2+velocity_r.y**2) > max_truncated_veloctiy



    #Elias sin tentative løsning, ja riktig tentativ    

    def choose_velocity(self):

        """
        
        Determines a reference velocity for the velocity controller
 

        """


        buffer_angle = math.pi/6 #placeholder
        velocity_o = self.obstacle.twist.twist.linear
        velocity_r = self.vessel.twist.twist.linear
        abs_vel = math.sqrt((velocity_r.x-velocity_o.x)**2+(velocity_r.y-velocity_o.y)**2)
        print(abs_vel)

        point = self.obstacle.pose.pose.position
        theta_ro = math.atan2(point.y,point.x)

        #Determine the direction of the obstacle, and which cone angle to follow

        if self.choose_left_cone():
            new_angle = self.left_angle + buffer_angle 
        
        else:
            new_angle = self.right_angle - buffer_angle
        abs_vel = math.sqrt((velocity_r.x)**2+(velocity_r.y)**2)
        new_velocity = Vector3(math.cos(new_angle)*abs_vel + velocity_o.x  ,math.sin(new_angle)*abs_vel + velocity_o.y,0)
        new_velocity.x = new_velocity.x/(math.sqrt(new_velocity.x**2+new_velocity.y**2))*abs_vel
        new_velocity.y = new_velocity.y/(math.sqrt(new_velocity.x**2+new_velocity.y**2))*abs_vel
        return new_velocity


    def choose_left_cone(self):
        point = self.obstacle.pose.pose.position
        theta_ro = math.atan2(point.y,point.x)
        velocity_o = self.obstacle.twist.twist.linear
        return (theta_ro>= 0 and math.atan2(velocity_o.y,velocity_o.x) < math.pi/2 and math.atan2(velocity_o.y,velocity_o.x) >= -math.pi/2) or not (theta_ro>= 0 and math.atan2(velocity_o.y,velocity_o.x) <= math.pi/2 or math.atan2(velocity_o.y,velocity_o.x) >= -math.pi/2)

        


        

if __name__ == "__main__":

    #Test with no velocity translation
    obstacle = Odometry()
    obstacle.pose.pose.position = Point(5,0,0)
    obstacle.twist.twist.linear = Vector3(0,0,0)
    vessel = Odometry()
    vessel.pose.pose.position = Point(0,0,0)
    
    VO = Velocity_Obstacle(1,obstacle,vessel)
    VO.set_cone_angles()
    print(VO.left_angle*180/math.pi)
    print(VO.right_angle*180/math.pi)

    #Test with velocity translation
    obstacle = Odometry()
    obstacle.pose.pose.position = Point(5,0,0)
    obstacle.twist.twist.linear = Vector3(-100,0,0)
    vessel = Odometry()
    vessel.pose.pose.position = Point(0,0,0)
    vessel.twist.twist.linear = Vector3(-1000,0,0)

    
    VO = Velocity_Obstacle(1,obstacle,vessel)
    VO.set_cone_angles()
    print(VO.check_if_collision())

    #Choose velocity test
    obstacle = Odometry()
    obstacle.pose.pose.position = Point(5,0,0)
    obstacle.twist.twist.linear = Vector3(0,1,0)
    vessel = Odometry()
    vessel.pose.pose.position = Point(0,0,0)
    vessel.twist.twist.linear = Vector3(1,0,0)
    
    VO = Velocity_Obstacle(1,obstacle,vessel)
    VO.set_cone_angles()
    print(VO.choose_velocity())
    #VO.vessel.twist.twist.linear = VO.choose_velocity()
    print(VO.check_if_collision())
    

