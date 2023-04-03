
import rospy
from nav_msgs.msg import Odometry
from vortex_msgs.msg import GuidanceData
from enum import Enum
import math
from VOMATH import VelocityObstacle


#----------------------------------------------------
# Still dont know the datatype given from perception,
# so just assuming odometry or now
#----------------------------------------------------

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


class ColavController:
    
    def __init__(self) -> None:
        rospy.init_node('colav_controller')
        self.obstacle_sub = rospy.Subscriber(
            "colav_obst",
            Odometry,
            self.obst_callback,
        )

        self.vessel_sub = rospy.Subscriber(
            "/pose_gt",
            Odometry,
            self.vessel_callback,
        )
        
        self.colav_pub = rospy.Publisher(
            rospy.get_param("/guidance_interface/colav_data"),
            GuidanceData,
            queue_size=1
        )


        self.obstacles = {}

        self.vessel = Obstacle()

        #placeholder values, use getparam?
        self.stop_zone_r = 4
        self.colimm_max_r = 20
    def vessel_callback(self,msg):
        self.vessel = self.odometry_to_obstacle(msg)
        
    def obst_callback(self,msg):
        self.obstacles[msg.header.seq]  = self.odometry_to_obstacle(msg)






    def get_closest_obst(self,obstacles:dict) -> Odometry:
        obst_list = []
        shortest_dist = math.inf
        closest_obst = None
        for obstacle in obstacles.items:
            new_distance = self.get_distance(obstacle,self.vessel)
            if new_distance < shortest_dist:
                closest_obst = obstacle
                shortest_dist = new_distance
        return closest_obst
    



    def get_distance(obstacle:Obstacle,vessel:Obstacle) -> float:
        dx = obstacle.x - vessel.x
        dy = obstacle.y -vessel.y

        distance = math.sqrt(dx**2+dy**2)
        return distance



    def odometry_to_obstacle(obstacle:Odometry) -> Obstacle:
        converted = Obstacle()
        converted.x = obstacle.pose.pose.position.x
        converted.y = obstacle.pose.pose.position.y
        converted.vx = obstacle.twist.twist.linear.x
        converted.vy = obstacle.twist.twist.linear.y
        return converted

    def gen_colav_data(self):
        closest_obst = self.get_closest_obst(self.obstacles)

        VO = VelocityObstacle(self.vessel,closest_obst)
        
        




        


    def run(self):
        while True:
            if rospy.get_param("/tasks/task_1"):
                colav_data = self.gen_colav_data()
                if colav_data is None:
                    continue
                self.colav_pub.publish(colav_data)



if __name__ == "__main__":
    controller = ColavController()
    controller.run()



