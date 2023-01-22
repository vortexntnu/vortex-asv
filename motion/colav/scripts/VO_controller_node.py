#!/usr/bin/python3

from velocity_obstacle_node import Velocity_Obstacle
import rospy
from nav_msgs.msg import Odometry
from std_srvs import Trigger,TriggerResponse
from vortex_msgs.msg import GuidanceData
from tf.transformations import euler_from_quaternion



class VO_controller_node:

    """
    The velocity object controller node.
    Changes the ASV velocity such that a collision cannot occur.

    """
    

    def __init__(self):
        rospy.init_node("colav",anonymous=True)

        self.vessel = Odometry()
        self.obstacle = Odometry()


        # Subscribers
        self.vessel_sub = rospy.Subscriber(
            "/pose_gt", Odometry, self.vessel_callback, queue_size=1
        )  # 20hz

        self.obstacle_sub = rospy.Subscriber(
            "/obstacle_gt", Odometry, self.obstacle_callback, queue_size=1
        )  # 20hz

        # Service
        self.colav_srv = rospy.Service("/colav",Trigger,self.avoid_collision)
        
        self.velocity_pub = rospy.Publisher("/guidance/los_data",GuidanceData,queue_size=1)
    



    def avoid_collision(self,trigger):


        """

        Avoids collison by continuously updating the desired velociy,
        and sending it to the AUV's velocity controller. This is implemented as a service that 
        will run as long a collision is possible.

        """


        VO = Velocity_Obstacle(None,self.obstacle,self.vessel) #Placeholder None
        while VO.check_if_collision():
            ref_speed, ref_heading = VO.choose_velocity()
            data = GuidanceData()
            data.psi_d = ref_heading
            data.u_d = ref_speed
            data.u = ref_speed
            data.t = rospy.Time.now()
            orientation_list = [
                self.vessel.pose.pose.orientation.x,
                self.vessel.pose.pose.orientation.y,
                self.vessel.pose.pose.orientation.z,
                self.vessel.pose.pose.orientation.w,
            ]
            data.psi = euler_from_quaternion(orientation_list)[2]
            self.velocity_pub.publish(data)
        return TriggerResponse (
            success = True,
            message = "success!"
        )
        
        

    def vessel_callback(self,data):
        self.vessel = data


    def obstacle_callback(self,data):
        self.obstacle = data


if __name__ == "__main__":
    try:
        node = VO_controller_node()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
