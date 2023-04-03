#!/usr/bin/python3


#------------------------------------------------------------
# will prob use something similar to this node,
# i.e just a node that enables colav and maybe posts wp to los
#-------------------------------------------------------------

import rospy
from nav_msgs.msg import Odometry
import actionlib
from vortex_msgs.srv import Waypoint, WaypointRequest, WaypointResponse



class ColavTask:
    def __init__(self):

        """

        """
        rospy.init_node("colav_fsm",anonymous=True)
        self.vessel = Odometry()
        self.goal = WaypointRequest()
        self.server = actionlib.SimpleActionServer('StartColavTask',Odometry,self.goal_cb, auto_start=True)

        #Subscribers
        #placeholder topic

        # self.start_task_sub = rospy.Subscriber(
        #     "goaltopic",Odometry,self.goal_callback,queue_size=1
        # )

        self.vessel_sub = rospy.Subscriber(
            "/pose_gt", Odometry, self.vessel_callback, queue_size=1
        )  # 20hz

        self.enabled = rospy.get_param("/tasks/task_1")


    def vessel_callback(self,data):
        """
        Update the postion of the UAV
        Args:
            data: An odometry of the UAV position
        """
        self.vessel = data

    def goal_cb(self,data):
        if self.enabled :
            goal_wp = WaypointRequest()
            goal_wp.waypoint = [data.pose.pose.position.s,data.pose.pose.position.y]

            vessel_wp = WaypointRequest()
            vessel_wp.waypoint = [self.vessel.pose.pose.position.x,self.vessel.pose.pose.position.y]
            response = WaypointResponse()
            response.success = False
            try:
                rospy.loginfo("Sending waypoints")
                rospy.wait_for_service("/navigation/add_waypoint")
                waypoint_client = rospy.ServiceProxy("/navigation/add_waypoint", Waypoint)
                first_response = waypoint_client(vessel_wp)
                second_response = waypoint_client(goal_wp)
                response.success = first_response.success and second_response.success
            except rospy.ServiceException as e:
                rospy.loginfo("Service call failed: {}".format(e))
        else:
            rospy.loginfo("FAILURE!: Task is not enabled yet!")

    def spin(self):
            self.enabled = rospy.get_param("/tasks/task_1")

            while not self.enabled:
                rospy.sleep(0.1)




if __name__ == "__main__":
    my_node_1 = ColavTask()
    my_node_1.spin()
    rospy.spin()




# import numpy as np
# MAX_SPEED = 5.0  # maximum speed of the vessel
# MAX_ACCELERATION = 1.0  # maximum acceleration of the vessel
# MAX_TURN_RATE = 0.5  # maximum turning rate of the vessel
# VO_MARGIN = 0.1  # margin for the velocity obstacle

# class Vessel:
#     def __init__(self, x, y, vx, vy, radius):
#         self.x = x
#         self.y = y
#         self.vx = vx
#         self.vy = vy
#         self.radius = radius

#     def update(self, x, y, vx, vy):
#         self.x = x
#         self.y = y
#         self.vx = vx
#         self.vy = vy

# def vo_collision_avoidance(ownship, intruders):
#     # Compute velocity obstacles for each intruder
#     v_obs = []
#     for intruder in intruders:
#         r = ownship.radius + intruder.radius + VO_MARGIN
#         u = intruder.vx - ownship.vx
#         v = intruder.vy - ownship.vy
#         u1 = u + r * np.sign(u) * np.sqrt(u**2 + v**2 - MAX_SPEED**2)
#         u2 = u - r * np.sign(u) * np.sqrt(u**2 + v**2 - MAX_SPEED**2)
#         v1 = v + r * np.sign(v) * np.sqrt(u**2 + v**2 - MAX_SPEED**2)
#         v2 = v - r * np.sign(v) * np.sqrt(u**2 + v**2 - MAX_SPEED**2)
#         v_obs.append((u1, u2, v1, v2))

#     # Compute the reachable set of velocities for the ownship
#     vx_reach = np.arange(-MAX_SPEED, MAX_SPEED + 0.1, 0.1)
#     vy_reach = np.arange(-MAX_SPEED, MAX_SPEED + 0.1, 0.1)
#     Vx, Vy = np.meshgrid(vx_reach, vy_reach)
#     Vx = Vx.flatten()
#     Vy = Vy.flatten()
#     V = np.array([Vx, Vy]).T
#     V_reach = []

#     for v in V:
#         if np.linalg.norm(v) < MAX_SPEED:
#             a = np.array([MAX_ACCELERATION, 0])
#             b = np.array([v[0], v[1]])
#             if np.arccos(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))) < MAX_TURN_RATE:
#                 V_reach.append(v)
#     V_reach = np.array(V_reach)
#     # Find the velocity that maximizes the distance to the velocity obstacles
#     best_v = np.array([0, 0])
#     best_dist = -np.inf
#     for v in V_reach:
#         dists = []
#         for obs in v_obs:
#             if v[0] > obs[0] or v[0] < obs[1] or v[1] > obs[2] or v[1] < obs[3]:
#                 dists.append(0)
#             else:
#                 d = np.min([obs[0] - v[0], v[0] - obs[1], obs[2] - v[1], v[1] - obs[3]])
#                 dists.append(d)
#         dist = np.min(dists)
#         if dist > best_dist:
#                     # Check if the chosen velocity is valid
#             a = best_v - np.array([ownship.vx, ownship.vy])
#             b = v - np.array([ownship.vx, ownship.vy])
#             if np.arccos(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))) > MAX_TURN_RATE:
#                 continue
#             ownship.update(ownship.x + v[0], ownship.y + v[1], v[0], v[1])
#             return ownship
#     return None