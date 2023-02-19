#!/usr/bin/python3

from geometry_msgs.msg import Point,Vector3
from nav_msgs.msg import Odometry
import math

import numpy as np

class VelocityObstacle:
    """
    The Velocity Obstacle class

        A computational class used by the collision avoidance system 
        to determine if a collison can occur, and how the UAV should respond

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
        Returns true if the current velocity results in a collision.
        Uses a truncated VO collision cone

        """

        point = self.obstacle.pose.pose.position
        velocity_r = self.vessel.twist.twist.linear
        velocity_o = self.obstacle.twist.twist.linear
        translated_vel = Vector3(velocity_r.x-velocity_o.x,velocity_r.y-velocity_o.y,0)
        angle = math.atan2(translated_vel.y,translated_vel.x)

        acceptance_radius = (self.radius_o + self.radius_r)/self.truncated_time
        max_truncated_veloctiy = math.sqrt(point.x**2+point.y**2)/self.truncated_time - acceptance_radius

        return angle > self.right_angle and angle < self.left_angle and math.sqrt(velocity_r.x**2+velocity_r.y**2) > max_truncated_veloctiy


    #Needs correction, according to boating rules   

    def choose_velocity(self):

        """
        
        Determines a reference velocity for the velocity controller.
        The velocity is chosen such that the UAV's speed remains constant, but the 
        desired heading is placed outside the truncated VO collison cone.
        The desired heading will be placed behind the obstacle, by setting it to
        a buffered version of the collision cone angle.
        Returns:
            Tuple containing:
                abs_vel: The current speed of the UAV
                new_angle: The desired heading of the UAV

        """

        #Current implementation need some work. Doesnt really follow COLREG, and doesnt really achieve constant speed

        buffer_angle = math.pi/6 #placeholder
        velocity_o = self.obstacle.twist.twist.linear
        velocity_r = self.vessel.twist.twist.linear
        abs_vel = math.sqrt((velocity_r.x-velocity_o.x)**2+(velocity_r.y-velocity_o.y)**2)

        point = self.obstacle.pose.pose.position
        theta_ro = math.atan2(point.y,point.x)

        #Determine the direction of the obstacle, and which cone angle to follow

        if self.choose_left_cone():
            new_angle = self.left_angle + buffer_angle 
        
        else:
            new_angle = self.right_angle - buffer_angle
        #abs_vel = math.sqrt((velocity_r.x)**2+(velocity_r.y)**2)
        #new_velocity = Vector3(math.cos(new_angle)*abs_vel + velocity_o.x  ,math.sin(new_angle)*abs_vel + velocity_o.y,0)
        #new_velocity.x = new_velocity.x/(math.sqrt(new_velocity.x**2+new_velocity.y**2))*abs_vel
        #new_velocity.y = new_velocity.y/(math.sqrt(new_velocity.x**2+new_velocity.y**2))*abs_vel
        return abs_vel,new_angle


    def choose_left_cone(self):

        """

        Return true if choosing the left side of the VO collision cone
        results in passing behind the obstacle.

        """

        point = self.obstacle.pose.pose.position
        theta_ro = math.atan2(point.y,point.x)
        velocity_o = self.obstacle.twist.twist.linear
        return (theta_ro>= 0 and math.atan2(velocity_o.y,velocity_o.x) < math.pi/2 and math.atan2(velocity_o.y,velocity_o.x) >= -math.pi/2) or not (theta_ro>= 0 and math.atan2(velocity_o.y,velocity_o.x) <= math.pi/2 or math.atan2(velocity_o.y,velocity_o.x) >= -math.pi/2)

        

MAX_SPEED = 4.0  # maximum speed of the vessel
MAX_ACCELERATION = 1.0  # maximum acceleration of the vessel
MAX_TURN_RATE = 0.5  # maximum turning rate of the vessel
VO_MARGIN = 0.1  # margin for the velocity obstacle

class Vessel:
    def __init__(self, x, y, vx, vy, radius):
        self.x = x        
        self.y = y        
        self.vx = vx        
        self.vy = vy        
        self.radius = radius    
    
    def update(self, x, y, vx, vy):
        self.x = x        
        self.y = y        
        self.vx = vx        
        self.vy = vy

def vo_collision_avoidance(ownship, intruders):
    # Compute velocity obstacles for each intruder    
    v_obs = []
    for intruder in intruders:
        r = ownship.radius + intruder.radius + VO_MARGIN        
        u = intruder.vx - ownship.vx        
        v = intruder.vy - ownship.vy        
        u1 = u + r * np.sign(u) * np.sqrt(u**2 + v**2 - MAX_SPEED**2)
        u2 = u - r * np.sign(u) * np.sqrt(u**2 + v**2 - MAX_SPEED**2)
        v1 = v + r * np.sign(v) * np.sqrt(u**2 + v**2 - MAX_SPEED**2)
        v2 = v - r * np.sign(v) * np.sqrt(u**2 + v**2 - MAX_SPEED**2)
        v_obs.append((u1, u2, v1, v2))

    # Compute the reachable set of velocities for the ownship    
    vx_reach = np.arange(-MAX_SPEED, MAX_SPEED + 0.1, 0.1)
    vy_reach = np.arange(-MAX_SPEED, MAX_SPEED + 0.1, 0.1)
    Vx, Vy = np.meshgrid(vx_reach, vy_reach)
    Vx = Vx.flatten()
    Vy = Vy.flatten()
    V = np.array([Vx, Vy]).T    
    V_reach = []
    
    for v in V:
        if np.linalg.norm(v) < MAX_SPEED:
            a = np.array([MAX_ACCELERATION, 0])
            b = np.array([v[0], v[1]])
            if np.arccos(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))) < MAX_TURN_RATE:
                V_reach.append(v)
    V_reach = np.array(V_reach)
    # Find the velocity that maximizes the distance to the velocity obstacles    
    best_v = np.array([0, 0])
    best_dist = -np.inf    
    for v in V_reach:
        dists = []
        for obs in v_obs:
            if v[0] > obs[0] or v[0] < obs[1] or v[1] > obs[2] or v[1] < obs[3]:
                dists.append(0)
            else:
                d = np.min([obs[0] - v[0], v[0] - obs[1], obs[2] - v[1], v[1] - obs[3]])
                dists.append(d)
        dist = np.min(dists)
        if dist > best_dist:
                    # Check if the chosen velocity is valid          
            a = best_v - np.array([ownship.vx, ownship.vy])
            b = v - np.array([ownship.vx, ownship.vy])
            if np.arccos(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))) > MAX_TURN_RATE:
                continue          
            ownship.update(ownship.x + v[0], ownship.y + v[1], v[0], v[1])
            return ownship      
    return None



        

if __name__ == "__main__":

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
    

    boat = Vessel(0,0,5,4,2)
    obs = Vessel(0,12,5,0,2)
    obs_vector = [obs]
    new_boat = vo_collision_avoidance(boat,obs_vector)
    speedx = new_boat.vx
    speedy = new_boat.vy
    print(speedx)
    print(speedy)

