#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Wrench


class JoystickInterface:

    def __init__(self):
        """
        Define constants used in the joystick mapping, and any ros
        specifics
        """

        rospy.init_node("joystick_interface")

        self.joystick_surge_scaling = rospy.get_param(
            "/joystick/scaling/surge")
        self.joystick_sway_scaling = rospy.get_param("/joystick/scaling/sway")
        self.joystick_yaw_scaling = rospy.get_param("/joystick/scaling/yaw")

        self.joystick_buttons_map = [
            "A",
            "B",
            "X",
            "Y",
            "LB",
            "RB",
            "back",
            "start",
            "power",
            "stick_button_left",
            "stick_button_right",
        ]

        self.joystick_axes_map = [
            "horizontal_axis_left_stick",
            "vertical_axis_left_stick",
            "LT",
            "horizontal_axis_right_stick",
            "vertical_axis_right_stick",
            "RT",
            "dpad_horizontal",
            "dpad_vertical",
        ]

        self.joystick_sub = rospy.Subscriber("/joystick/joy",
                                             Joy,
                                             self.joystick_cb,
                                             queue_size=1)

        self.force_pub = rospy.Publisher("/thrust/desired_force",
                                         Wrench,
                                         queue_size=1)
        self.torque_pub = rospy.Publisher("/thrust/desired_torque",
                                          Wrench,
                                          queue_size=1)

        rospy.loginfo("Joystick interface is up and running")

    def joystick_cb(self, msg):
        buttons = {}
        axes = {}

        for i in range(len(msg.buttons)):
            buttons[self.joystick_buttons_map[i]] = msg.buttons[i]

        for i in range(len(msg.axes)):
            axes[self.joystick_axes_map[i]] = msg.axes[i]

        surge = axes["vertical_axis_left_stick"] * self.joystick_surge_scaling
        sway = axes["horizontal_axis_left_stick"] * self.joystick_sway_scaling
        yaw = axes["horizontal_axis_right_stick"] * self.joystick_yaw_scaling

        wrench_msg = Wrench()
        wrench_msg.force.x = surge
        wrench_msg.force.y = sway
        wrench_msg.torque.z = yaw

        self.force_pub.publish(wrench_msg)
        self.torque_pub.publish(wrench_msg)


if __name__ == "__main__":

    joystick_interface = JoystickInterface()
    rospy.spin()
