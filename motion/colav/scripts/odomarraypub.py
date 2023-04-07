#!/usr/bin/python3

from vortex_msgs.msg import OdometryArray
from nav_msgs.msg import Odometry

import rospy

rospy.init_node('asdasdaqw')
pub = rospy.Publisher(
    "/tracking/mul_tracked_cv_objects",
    OdometryArray,
    queue_size=1
)

msg = OdometryArray()
submsg = Odometry()
submsg.pose.pose.position.x = 10
msg.odometry_array = [submsg , submsg]

pub.publish(msg)
print("message sent!")

while not rospy.is_shutdown():
    pub.publish(msg)
    rate = rospy.Rate(10)
    rate.sleep()


