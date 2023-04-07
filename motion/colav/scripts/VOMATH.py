#!/usr/bin/python3


#NEEDS TO BE REWRITTEN WITH CORRECT DATATYPE!!!!!

from geometry_msgs.msg import Point,Vector3
from nav_msgs.msg import Odometry
import math
import rospy
from enum import Enum
import numpy as np


class Zones(Enum):
    NOCOL = 1
    COLIMM = 2
    STOPNOW = 3


class Approaches(Enum):
    FRONT = 1
    RIGHT = 2
    LEFT = 3
    BEHIND = 4


class Obstacle:
    def __init__(self) -> None:
        self.vx = 0
        self.vy = 0
        self.r = 0
        self.x = 0
        self.y = 0
        self.heading = 0
        self.speed = 0



class VelocityObstacle:
    """
    The Velocity Obstacle class

        A computational class used by the collision avoidance system 
        to determine if a collison can occur, and how the UAV should respond

        obstacle: An odometry of the object to avoid
        radius_o: The radius of obstacle
        vessel: An odometry of the UAV vessel 
    """

    def __init__(self,vessel :Obstacle  ,obstacle:Obstacle)->None:
    
        self.vessel = vessel
        self.obstacle = obstacle
        
        self.left_angle = 0.0
        self.right_angle = 0.0

        self.truncated_time = 5 #placeholder
        

    def set_cone_angles(self)-> None:
        """
        Calculates the largest and smallest heading-angle where a collision can occur 
        """
        theta_ro = math.atan2(self.obstacle.y-self.vessel.y,self.obstacle.x-self.vessel.x)
        print("ob",self.vessel.r,self.obstacle.r)
        theta_ray = math.asin((self.vessel.r+self.obstacle.r)/(math.sqrt((self.obstacle.x-self.vessel.x)**2+(self.obstacle.y-self.vessel.y)**2)))
        self.right_angle = theta_ro-theta_ray
        self.left_angle = theta_ro + theta_ray 

    def check_if_collision(self)->bool:
        """
        Returns true if the current velocity results in a collision.
        Uses a truncated VO collision cone

        """

        # point = self.obstacle.pose.pose.position
        # velocity_r = self.vessel.twist.twist.linear
        # velocity_o = self.obstacle.twist.twist.linear
        # translated_vel = Vector3(velocity_r.x-velocity_o.x,velocity_r.y-velocity_o.y,0)
        # angle = math.atan2(translated_vel.y,translated_vel.x)

        # acceptance_radius = (self.radius_o + self.radius_r)/self.truncated_time
        # max_truncated_veloctiy = math.sqrt(point.x**2+point.y**2)/self.truncated_time - acceptance_radius

        # return angle > self.right_angle and angle < self.left_angle and math.sqrt(velocity_r.x**2+velocity_r.y**2) > max_truncated_veloctiy

        bouffer = 0
        dvx  = self.obstacle.vx - self.vessel.vx 
        dvy = self.obstacle.vy - self.vessel.vy
        angle = math.atan2(-dvy,-dvx)
        print("vels",dvx,dvy)

        return angle > self.right_angle-bouffer and angle < self.left_angle+bouffer


     

if __name__ == "__main__":

    node = VelocityObstacle()
    node.spin()
    # #Test with no velocity translation
    # obstacle = Odometry()
    # obstacle.pose.pose.position = Point(5,0,0)
    # obstacle.twist.twist.linear = Vector3(0,0,0)
    # vessel = Odometry()
    # vessel.pose.pose.position = Point(0,0,0)
    
    # VO = VelocityObstacle(1,obstacle,vessel)
    # VO.set_cone_angles()
    # print(VO.left_angle*180/math.pi)
    # print(VO.right_angle*180/math.pi)

    # #Test with velocity translation
    # obstacle = Odometry()
    # obstacle.pose.pose.position = Point(5,0,0)
    # obstacle.twist.twist.linear = Vector3(-100,0,0)
    # vessel = Odometry()
    # vessel.pose.pose.position = Point(0,0,0)
    # vessel.twist.twist.linear = Vector3(-1000,0,0)

    
    # VO = VelocityObstacle(1,obstacle,vessel)
    # VO.set_cone_angles()
    # print(VO.check_if_collision())

    # #Choose velocity test
    # obstacle = Odometry()
    # obstacle.pose.pose.position = Point(5,0,0)
    # obstacle.twist.twist.linear = Vector3(0,1,0)
    # vessel = Odometry()
    # vessel.pose.pose.position = Point(0,0,0)
    # vessel.twist.twist.linear = Vector3(1,0,0)
    
    # VO = VelocityObstacle(1,obstacle,vessel)
    # VO.set_cone_angles()
    # print(VO.choose_velocity())
    # #VO.vessel.twist.twist.linear = VO.choose_velocity()
    # print(VO.check_if_collision())
    

    # boat = Vessel(0,0,5,4,2)
    # obs = Vessel(0,12,5,0,2)
    # obs_vector = [obs]
    # new_boat = vo_collision_avoidance(boat,obs_vector)
    # speedx = new_boat.vx
    # speedy = new_boat.vy
    # print(speedx)
    # print(speedy)

    # Sample parameter values
    # current_velocity = np.array([1, 0])
    # current_speed = 2
    # goal = np.array([10, 10])
    # obstacle_positions = [np.array([3, 3]), np.array([7, 7])]
    # obstacle_radii = [1, 2]
    # theta_max = np.pi/6  # 30 degrees in radians

    # # Find new velocity
    # new_velocity = find_new_velocity(current_velocity, current_speed, goal, obstacle_positions, obstacle_radii, theta_max)

    # # Print new velocity
    # print("New velocity: ", new_velocity)

