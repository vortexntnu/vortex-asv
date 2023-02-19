#!/usr/bin/python3
import rospy


class Colav:
    def __init__(self):
        self.enabled = rospy.get_param("/tasks/collision_avoidance")
        #Your code here

    def execute(self):
        while not rospy.is_shutdown():
            self.enabled = rospy.get_param("/tasks/collision_avoidance")

            if self.enabled:
                rospy.loginfo("collision_avoidance; Inside callback")
                #Your code here

            rospy.sleep(0.1)


if __name__ == "__main__":
    rospy.init_node("collision_avoidance")
    colav_node = Colav()
    colav_node.spin()