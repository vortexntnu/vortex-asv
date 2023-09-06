#!/usr/bin/python3

import rospy

from lqr_interface import LQRInterface

from geometry_msgs.msg import Point
"""
DOCKING SEARCH

Look for Aruco markers.

Once a marker has been found, extract bearing.

Find the PCL cluster centroid that is within some interval of this bearing.

Convert centroid to world point.

Return world point to DockingConverge.
"""
"""
DOCKING CONVERGE

lqr_move_to_point to the world point.

If closer than some predefined limit (i.e. 3m), move to execute with the current world point,

else once halfway to world point, move back to DockingSearch.
"""
"""
DOCKING EXECUTE

Use lqr_move_to_point, but use a much smaller Q matrix for slower convergence.
"""


class states:
    SEARCH = 0
    CONVERGE = 1
    EXECUTE = 2


class DockingTask:

    def __init__(self):
        rospy.init_node("docking_task_node")

        aruco_pose_sub = rospy.Subscriber("/detector/aruco",
                                          self.aruco_pose_cb,
                                          queue_size=10)
        waypoint_reached_signal = rospy.Subscriber(
            "/controller/lqr/final_waypoint_reached",
            self.waypoint_reached_signal_cb,
            queue_size=10)

        self.lqr_controller_interface = LQRInterface()

        self.aruco_detection = None
        self.waypoint_reached_signal = False

        self.state = states.SEARCH
        self.converge_entry_flag = False

    def closest_world_point_from_bearing(self, bearing: float) -> Point:
        #Find the PCL cluster centroid that is within some interval of this bearing.

        #Convert centroid to world point.

        return Point(3.0, 1.0, 0.0)

    def get_halfway_point(world_point: Point) -> Point:
        # Calculate the point halfway between us and the world_point.
        pass

    def bearing_from_aruco_detection(self):
        # Convert aruco detection to bearing
        pass

    def aruco_pose_cb(self, detection):
        if self.state == states.SEARCH:
            # We only update the detection we keep when we are in search mode
            self.aruco_detection = detection
            self.state = states.CONVERGE
            self.converge_entry_flag = True

    def waypoint_reached_signal_cb(self, msg):
        self.waypoint_reached_signal = msg.data

    def spin_once(self):

        if self.state == states.SEARCH:
            # Simply wait for aruco detections
            # Might need to do exploration if no marker is found from the current position of the ASV
            return

        # Here we assume to have a valid aruco detection
        bearing = self.bearing_from_aruco_detection()

        world_point = self.closest_world_point_from_bearing(bearing)

        if self.state == states.CONVERGE:
            if self.converge_entry_flag:
                # Move to halfway point, only call move_to_point once
                halfway_point = self.get_halfway_point(world_point)
                self.lqr_controller_interface.move_to_point(halfway_point)

            self.converge_entry_flag = False

            if self.waypoint_reached_signal:
                # We have reached halfway point
                self.state = states.SEARCH

            return


if __name__ == "__main__":

    state_machine = DockingTask()

    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        state_machine.spin_once()
        rate.sleep()
