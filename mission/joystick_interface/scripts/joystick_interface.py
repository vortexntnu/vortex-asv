#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Joy
#git remote set-url origin git@github.com:vortexntnu/vortex-asv.git


class JoystickInterface(Node):

    def __init__(self):
        rclpy.init()

        #Mapping copy pasted from the original file
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

        super().__init__('joystick_interface_node')
        self.subscriber = self.create_subscription(Joy, 'joy',
                                                   self.joystick_cb, 1)

        #self.wrench_publisher = self.create_publisher(Wrench, 'wrench_topic', 1)

    def create_2d_wrench_message(self, x, y, yaw):
        wrench_msg = Wrench()
        wrench_msg.force.x = x
        wrench_msg.force.y = y
        wrench_msg.torque.z = yaw
        return wrench_msg

    """"
    def publish_wrench_message(self, wrench):
        self.wrench.pub.publish(wrench)

    """

    def joystick_cb(self, msg):
        #print(f"Received message: {msg.buttons}")
        print("hello")
        a = msg.buttons
        return a


def main(args=None):
    #rclpy.init(args=args)
    print("hello from main")
    joystick_interface = JoystickInterface().subscriber
    rclpy.spin(joystick_interface)
    joystick_interface.destroy_node()

    rclpy.shutdown()
    return
