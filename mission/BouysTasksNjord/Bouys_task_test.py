#!/usr/bin/python3
#Written by Sigurd von Brandis, Student

import rospy
from std_msgs.msg import String
from Bouys_task_test import BouysAndMarkers


class BouysAndMarkersPublisher:

    def __init__(self):
        rospy.init_node('bouys_and_markers_publisher', anonymous=True)
        self.pub = rospy.Publisher('bouys_and_markers',
                                   BouysAndMarkers,
                                   queue_size=10)
        self.rate = rospy.Rate(1)  # Publishing rate in Hz

    def publish_bouys_and_markers(self):
        while not rospy.is_shutdown():
            bouys_and_markers_msg = BouysAndMarkers()
            bouys_and_markers_msg.red_bouy_array = [(1, 4), (5, 15)]
            bouys_and_markers_msg.green_bouy_array = [(3, 4)]
            bouys_and_markers_msg.north_marker_array = []
            bouys_and_markers_msg.south_marker_array = []
            bouys_and_markers_msg.east_marker_array = []
            bouys_and_markers_msg.west_marker_array = []
            self.pub.publish(bouys_and_markers_msg)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        bouys_and_markers_publisher = BouysAndMarkersPublisher()
        bouys_and_markers_publisher.publish_bouys_and_markers()
    except rospy.ROSInterruptException:
        pass
