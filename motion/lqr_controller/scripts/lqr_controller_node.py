#!/usr/bin/python3

import numpy as np

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64MultiArray, Bool
from tf.transformations import euler_from_quaternion

from lqr_controller import LQRController


class LQRControllerNode:

    def __init__(self):
        rospy.init_node("lqr_controller")

        mass = rospy.get_param("lqr_controller/mass", 50.0)
        inertia = rospy.get_param("lqr_controller/inertia", 5.0)

        damping_x = rospy.get_param("lqr_controller/damping/x", 5.0)
        damping_y = rospy.get_param("lqr_controller/damping/y", 20.0)
        damping_psi = rospy.get_param("lqr_controller/damping/psi", 15.0)

        Q = rospy.get_param("lqr_controller/Q", [10.0, 10.0, 1.0, 0.001, 0.001, 0.001, 1.0, 1.0])
        R = rospy.get_param("lqr_controller/R", [0.01, 0.01, 0.01])

        M = np.diag([mass, mass, inertia]) 
        D = np.diag([damping_x, damping_y, damping_psi])

        self.setpoints = np.zeros(6)

        self.do_publish_tau = False

        self.lqr_controller = LQRController(M, D, Q, R, actuator_limits=150.0)
        self.lqr_controller.set_setpoint(self.setpoints)

        # Whenever a state input is received, publish a control force
        rospy.Subscriber("/odometry/filtered", Odometry,
                         self.odometry_callback)

        rospy.Subscriber("/controller/lqr/enable", Bool, self.enable_callback)
        rospy.Subscriber("/controller/lqr/setpoints", Float64MultiArray,
                         self.setpoint_callback)

        self.tau_pub = rospy.Publisher("/thrust/force_input",
                                       Wrench,
                                       queue_size=10)

        self.last_time = rospy.get_time()

    def enable_callback(self, msg):
        rospy.loginfo(f"Turning LQR {'on' if msg.data else 'off'}")
        self.last_time = rospy.get_time()
        self.do_publish_tau = msg.data

    def setpoint_callback(self, msg):
        number_of_setpoints = len(msg.data)
        if number_of_setpoints != 6:
            rospy.logwarn(
                f"LQR setpoints must be an array of 6 elements, received {number_of_setpoints}. Setpoints not updated..."
            )
            return

        rospy.loginfo(f"Setting LQR setpoints to: {msg.data}")
        self.lqr_controller.set_setpoint(msg.data)

    def odometry_callback(self, msg):

        if not self.do_publish_tau:
            return

        # Extract the x and y position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Extract the orientation quaternion
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ]

        # Convert quaternion to Euler angles
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        # Extract the linear velocities
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y

        # Extract the angular velocity (yaw rate)
        vyaw = msg.twist.twist.angular.z

        # Combine all data into numpy array
        state = np.array([x, y, yaw, vx, vy, vyaw])

        current_time = rospy.get_time()
        dt = current_time - self.last_time
        self.last_time = current_time

        tau = self.lqr_controller.control(state, dt)

        wrench = Wrench()
        wrench.force.x = tau[0]
        wrench.force.y = tau[1]
        wrench.force.z = 0.0
        wrench.torque.x = 0.0
        wrench.torque.y = 0.0
        wrench.torque.z = tau[2]

        # You can then publish this Wrench message
        self.tau_pub.publish(wrench)


if __name__ == "__main__":

    lqr_controller = LQRControllerNode()

    rospy.spin()
