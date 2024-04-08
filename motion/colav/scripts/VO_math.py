#!/usr/bin/python3

from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
import math
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

    def __init__(self, vessel: Obstacle, obstacle: Obstacle) -> None:

        self.vessel = vessel
        self.obstacle = obstacle

        self.left_angle = 0.0
        self.right_angle = 0.0

        self.truncated_time = 5  #placeholder

    def set_cone_angles(self) -> None:
        """
        Calculates the largest and smallest heading-angle where a collision can occur 
        """
        theta_ro = math.atan2(self.obstacle.y - self.vessel.y,
                              self.obstacle.x - self.vessel.x)
        print("ob", self.vessel.r, self.obstacle.r)
        distance = math.sqrt((self.obstacle.x - self.vessel.x) ** 2 + (self.obstacle.y - self.vessel.y) ** 2)
        if distance == 0:
            # Handle the case where distance is zero
            # This could be setting theta_ray to some default value or skipping calculations
            return
        else:
            theta_ray = math.asin((self.vessel.r + self.obstacle.r) / distance)

        self.right_angle = theta_ro - theta_ray
        self.left_angle = theta_ro + theta_ray

    def check_if_collision(self) -> bool:
        """
        Returns true if the current velocity results in a collision.
        Uses a truncated VO collision cone

        """

        bouffer = 0
        dvx = self.obstacle.vx - self.vessel.vx
        dvy = self.obstacle.vy - self.vessel.vy
        angle = math.atan2(-dvy, -dvx)
        print("vels", dvx, dvy)

        return angle > self.right_angle - bouffer and angle < self.left_angle + bouffer