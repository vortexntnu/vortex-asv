#!/usr/bin/python3
from dataclasses import dataclass

from std_msgs.msg import Float64MultiArray, Bool, Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import rospy
import numpy as np


@dataclass
class Waypoint:
    north: float
    east: float
    heading: float


class LQRGuidanceROS:

    def __init__(self):
        rospy.init_node("lqr_guidance")

        self.interpolation_step_size = rospy.get_param(
            "lqr_guidance/interpolation_step_size", 1.0)

        self.convergence_radius = rospy.get_param(
            "lqr_guidance/convergence/radius", 0.25)
        self.convergence_angle = rospy.get_param(
            "lqr_guidance/convergence/angle", np.pi / 4)

        self.use_path_dependent_heading = rospy.get_param(
            "lqr_guidance/use_path_dependent_heading", True)
        self.do_debug_print = rospy.get_param("lqr_guidance/debug/print",
                                              False)

        self.waypoints = []
        self.current_waypoint_index = None
        self.current_pose = np.zeros(3)

        rospy.Subscriber("/odometry/filtered", Odometry,
                         self.odometry_callback)
        rospy.Subscriber("/guidance/lqr/clear_waypoints", Empty,
                         self.clear_waypoints_callback)
        rospy.Subscriber("/guidance/lqr/add_waypoint", Point,
                         self.add_waypoint_callback)
        rospy.Subscriber("/guidance/lqr/toggle_path_dependent_heading", Bool,
                         self.toggle_path_dependent_heading)

        self.setpoint_pub = rospy.Publisher("/controller/lqr/setpoints",
                                            Float64MultiArray,
                                            queue_size=10)
        self.enable_pub = rospy.Publisher("/controller/lqr/enable",
                                          Bool,
                                          queue_size=10)
        self.final_waypoint_reached_pub = rospy.Publisher("/controller/lqr/final_waypoint_reached", Bool, queue_size=10)

    def ssa(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def quaternion_to_yaw(self, q):
        x = q.x
        y = q.y
        z = q.z
        w = q.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return yaw

    def odometry_callback(self, data):
        position = data.pose.pose.position
        yaw = self.quaternion_to_yaw(data.pose.pose.orientation)

        self.current_pose = np.array((position.x, position.y, yaw))

        if self.current_waypoint_index is not None:
            waypoint = self.waypoints[self.current_waypoint_index]
            dx = position.x - waypoint.north
            dy = position.y - waypoint.east
            dyaw = np.abs(self.ssa(waypoint.heading - yaw))
            if np.hypot(
                    dx, dy
            ) < self.convergence_radius and dyaw < self.convergence_angle:
                if self.do_debug_print:
                    rospy.loginfo(
                        f"Reached waypoint {self.current_waypoint_index}")
                self.send_waypoint(self.current_waypoint_index + 1)

    def publish_setpoint(self, waypoint):
        msg = Float64MultiArray()
        msg.data = [waypoint.north, waypoint.east, waypoint.heading, 0, 0, 0]

        self.setpoint_pub.publish(msg)

    def send_waypoint(self, index):
        """ Send the waypoint at the given index to the controller """
        if 0 <= index < len(self.waypoints):
            self.current_waypoint_index = index
            waypoint = self.waypoints[index]
            if self.do_debug_print:
                rospy.loginfo(f"Sending waypoint {index}: {waypoint}")
            self.publish_setpoint(waypoint)

            #self.enable_pub.publish(True)
        else:
            rospy.loginfo("Final waypoint reached!")
            self.final_waypoint_reached_pub.publish(Bool(True))
            #self.enable_pub.publish(False)

    def clear_waypoints_callback(self, data):
        """ Callback for the clear waypoints topic """
        self.waypoints = []
        self.current_waypoint_index = None
        rospy.loginfo("Waypoints cleared!")

    def add_waypoint_callback(self, data):
        self.final_waypoint_reached_pub.publish(Bool(False))
        new_waypoint = Waypoint(data.x, data.y, data.z)

        starting_waypoint = None
        if self.current_waypoint_index is None:  # if this is the first waypoint, interpolate from where we are at:
            starting_waypoint = Waypoint(self.current_pose[0],
                                         self.current_pose[1],
                                         self.current_pose[2])
        else:  # if there are already existing waypoints, interpolate between the last one and the new one
            starting_waypoint = self.waypoints[-1]

        self.interpolate_and_append_waypoints(starting_waypoint, new_waypoint)

        if self.do_debug_print:
            rospy.loginfo(f"Waypoint added: {new_waypoint}")

        # once waypoints have been added, send the first one
        if self.current_waypoint_index is None:
            self.send_waypoint(0)

    def interpolate_and_append_waypoints(self, waypoint1, waypoint2):
        dx = waypoint2.north - waypoint1.north
        dy = waypoint2.east - waypoint1.east
        distance = np.hypot(dx, dy)

        num_intermediate_points = int(
            np.ceil(distance / self.interpolation_step_size))

        if self.use_path_dependent_heading:
            heading = np.arctan2(dy, dx)
        else:
            heading = waypoint1.heading

        for i in range(num_intermediate_points +
                       1):  # +1 to include the endpoint
            ratio = i / num_intermediate_points  # this will be 1 for the endpoint
            intermediate_point = Waypoint(waypoint1.north + ratio * dx,
                                          waypoint1.east + ratio * dy, heading)
            self.waypoints.append(intermediate_point)

    def toggle_path_dependent_heading(self, msg):
        self.use_path_dependent_heading = msg.data


if __name__ == "__main__":

    guidance_scheme = LQRGuidanceROS()

    rospy.spin()
