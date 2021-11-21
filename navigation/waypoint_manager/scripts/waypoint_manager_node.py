#!/usr/bin/python3
# written by Ronja Kræmer, Student

import rospy
from vortex_msgs.srv import WaypointUpdate


class WaypointManager():
    def __init__(self):
        rospy.init_node('WaypointManager')
        update_service = rospy.Service('/update_waypoint', WaypointUpdate, self.send_next_waypoint)
        
        self.waypoint_list = []

    def send_next_waypoint(self):
        pass
























if __name__ == '__main__':
	try:
		waypoint_manager = WaypointManager()

		rospy.spin()
	except rospy.ROSInterruptException:
		pass