#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from vortex_msgs.msg import GuidanceData, OdometryArray
import math
from VOMATH import VelocityObstacle, Obstacle, Zones, Approaches
from tf.transformations import euler_from_quaternion

#----------------------------------------------------
# Still dont know the datatype given from perception,
# so just assuming odometry or now
#----------------------------------------------------

# class Zones(Enum):
#     NOCOL = 1
#     COLIMM = 2
#     STOPNOW = 3

# class Approaches(Enum):
#     FRONT = 1
#     RIGHT = 2
#     LEFT = 3
#     BEHIND = 4

# class Obstacle:
#     def __init__(self) -> None:
#         self.vx = 0
#         self.vy = 0
#         self.r = 0
#         self.x = 0
#         self.y = 0
#         self.heading = 0
#         self.speed = 0


class ColavController:

    def __init__(self) -> None:
        rospy.init_node('colav_controller')
        self.obstacle_sub = rospy.Subscriber(
            "/tracking/mul_tracked_cv_objects",
            OdometryArray,
            self.obst_callback,
        )

        self.vessel_sub = rospy.Subscriber(
            "/pose_gt",
            Odometry,
            self.vessel_callback,
        )

        self.colav_pub = rospy.Publisher(
            # rospy.get_param("/guidance_interface/colav_data"),
            "/guidance/colav_data",
            # rospy.get_param("/guidance_interface/colav_data"),
            GuidanceData,
            queue_size=1)

        self.obstacles = []
        self.vessel_odom = Odometry()
        self.vessel = Obstacle()
        self.vessel.r = 2

        #placeholder values, use getparam?
        self.stop_zone_r = 0
        self.colimm_max_r = math.inf

        self.t = 0

    def vessel_callback(self, msg):
        self.vessel_odom = msg
        self.vessel = self.odometry_to_obstacle(msg)
        self.vessel.r = 2
        self.t = msg.header.stamp.to_sec()

    def obst_callback(self, msg):
        self.obstacles = msg.odometry_array
        # print("rec msg:",msg)
        if len(self.obstacles) == 0:
            print("empty!")
            return
        colav_data = self.gen_colav_data()
        if colav_data is None:
            return
        self.colav_pub.publish(colav_data)
        print("publishing data")

    def get_closest_obst(self, obstacles: dict) -> Odometry:
        shortest_dist = math.inf
        closest_obst = None
        for obstacle in obstacles:
            obstacle = self.odometry_to_obstacle(obstacle)
            obstacle.r = 2
            new_distance = self.get_distance(obstacle, self.vessel)
            if new_distance < shortest_dist:
                closest_obst = obstacle
                shortest_dist = new_distance
        return closest_obst

    def get_distance(self, obstacle: Obstacle, vessel: Obstacle) -> float:
        dx = obstacle.x - vessel.x
        dy = obstacle.y - vessel.y

        distance = math.sqrt(dx**2 + dy**2)
        return distance

    def odometry_to_obstacle(self, obstacle: Odometry) -> Obstacle:
        converted = Obstacle()
        converted.x = obstacle.pose.pose.position.x
        converted.y = obstacle.pose.pose.position.y
        converted.vx = obstacle.twist.twist.linear.x
        converted.vy = obstacle.twist.twist.linear.y
        converted.heading = math.atan2(converted.vy, converted.vx)
        converted.speed = math.sqrt(converted.vx**2 + converted.vy**2)
        return converted

    def gen_colav_data(self):
        closest_obst = self.get_closest_obst(self.obstacles)
        VO = VelocityObstacle(self.vessel, closest_obst)
        VO.set_cone_angles()
        print(closest_obst.x)
        print(VO.left_angle, VO.right_angle)
        print(self.vessel.x)

        zone = self.get_zone(closest_obst, self.vessel)
        print(zone)
        if zone == Zones.NOCOL:
            return None
        elif zone == Zones.STOPNOW:
            data = GuidanceData()
            data.psi_d = self.vessel.heading
            data.u_d = 0
            data.u = 0
            data.t = self.t
            orientation_list = [
                self.vessel_odom.pose.pose.orientation.x,
                self.vessel_odom.pose.pose.orientation.y,
                self.vessel_odom.pose.pose.orientation.z,
                self.vessel_odom.pose.pose.orientation.w,
            ]
            data.psi = euler_from_quaternion(orientation_list)[2]
            ##Set velocity to zero and heading to same as before:))
            return data
        elif zone == Zones.COLIMM and not VO.check_if_collision():
            return None

        #Now in collision imminent state

        approach = self.gen_approach(closest_obst, self.vessel)

        if approach == Approaches.FRONT or approach == Approaches.RIGHT:
            print("doodo")
            buffer = math.pi / 6
            new_heading = VO.right_angle - buffer

            data = GuidanceData()
            data.psi_d = new_heading
            data.u_d = self.vessel.speed
            data.u = self.vessel.speed
            data.t = self.t
            orientation_list = [
                self.vessel_odom.pose.pose.orientation.x,
                self.vessel_odom.pose.pose.orientation.y,
                self.vessel_odom.pose.pose.orientation.z,
                self.vessel_odom.pose.pose.orientation.w,
            ]
            data.psi = euler_from_quaternion(orientation_list)[2]
            # choose right cone
            return data
        elif approach == Approaches.BEHIND or approach == Approaches.LEFT:
            return None
        return None

    def gen_approach(self, obstacle: Obstacle, vessel: Obstacle) -> Approaches:
        dx = obstacle.x - vessel.x
        dy = obstacle.y - vessel.y
        buffer = 10 * math.pi / 180
        phi = math.atan2(dy, dx)

        if vessel.heading + buffer > phi and vessel.heading - buffer < phi:
            return Approaches.FRONT
        elif vessel.heading - buffer > phi:
            return Approaches.RIGHT
        elif vessel.heading + buffer < phi:
            return Approaches.LEFT
        return Approaches.BEHIND

    def get_zone(self, obstacle: Obstacle, vessel: Obstacle) -> Zones:
        distance = self.get_distance(obstacle, vessel)
        if distance > self.colimm_max_r:
            return Zones.NOCOL
        elif distance < self.colimm_max_r and distance > self.stop_zone_r:
            return Zones.COLIMM
        return Zones.STOPNOW

    # def run(self):
    #     while True:
    #         if rospy.get_param("/tasks/task_1"):
    #             colav_data = self.gen_colav_data()
    #             if colav_data is None:
    #                 continue
    #             self.colav_pub.publish(colav_data)


if __name__ == "__main__":
    controller = ColavController()
    rospy.spin()
