#!/usr/bin/python3

#--------------------------------------------
# this is placeholder code
# topics and datatypes are not correct
#--------------------------------------------

import rospy
from nav_msgs.msg import Odometry

class ObstacleServer:
    def __init__(self) -> None:
                # Initialize the ROS node and create a subscriber
        rospy.init_node('obstacle_subscriber')
        # String is just a placeholder for now
        self.obstacle_sub = rospy.Subscriber('obstacle', Odometry, self.obstacle_callback)
        self.obstacle_pub = rospy.Publisher('colav_obst',Odometry,queue_size=1)
        
        # Create an empty dictionary to store the obstacles
        self.obstacles = {}


        self.get_obstacle_service = rospy.Service('get_obstacle', Odometry, self.get_obstacle_callback)

    def obstacle_callback(self, msg):
        
        # Store the dictionary in the obstacles dictionary
        # placeholder code 
        self.obstacles[msg.header.seq] = msg
        self.obstacle_pub.publish(msg)

    def get_obstacle_callback(self,req):
        if req.header.seq in self.obstacles :
            return self.obstacles[req.header.seq]
        else:
            return None
    def run(self):
        rospy.spin()

if __name__ == "__main__":
    obstacle_server = ObstacleServer()
    obstacle_server.run()


