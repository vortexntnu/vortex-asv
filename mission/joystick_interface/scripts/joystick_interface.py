#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Joy
#git remote set-url origin git@github.com:vortexntnu/vortex-asv.git
import yaml

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
            "horizontal_axis_left_stick", #Translation (Left and Right)
            "vertical_axis_left_stick", #Translation (Forwards and Backwards)
            "LT",
            "horizontal_axis_right_stick", #Rotation
            "vertical_axis_right_stick",
            "RT",
            "dpad_horizontal",
            "dpad_vertical",
        ]

        #Create a Publisher and a suscriber from the ros2 tutorial
        super().__init__('joystick_interface_node')
        self.joy_subscriber = self.create_subscription(Joy, "/joystick/joy", self.joystick_cb, 1)
        
        self.wrench_publisher = self.create_publisher(Wrench, "/thrust/wrench_input", 1)


        #YAML file first need to be translating in ROS2 ; Getting the input from the controller'
        self.declare_parameter('surge', 100.0)
        self.declare_parameter('sway', 100.0)
        self.declare_parameter('yaw', 100.0)

        self.joystick_surge_scaling = self.get_parameter('surge').value #is it getting the parameters from the YAML file ? No --> TO DO
        self.joystick_sway_scaling = self.get_parameter('sway').value
        self.joystick_yaw_scaling = self.get_parameter('yaw').value



    def create_2d_wrench_message(self, x, y, yaw):
        wrench_msg = Wrench()
        wrench_msg.force.x = x
        wrench_msg.force.y = y
        wrench_msg.torque.z = yaw
        return wrench_msg

    
    def publish_wrench_message(self, wrench):
        self.wrench_publisher.publish(wrench)

    
    def joystick_cb(self, msg):
        
        #Input from controller to joystick_interface
        buttons = {} #dictionnary
        axes = {}

        for i in range(len(msg.buttons)):
            buttons[self.joystick_buttons_map[i]] = msg.buttons[i]

        for i in range(len(msg.axes)):
            axes[self.joystick_axes_map[i]] = msg.axes[i]

        surge = axes["vertical_axis_left_stick"] * self.joystick_surge_scaling
        sway = axes["horizontal_axis_left_stick"] * self.joystick_sway_scaling
        yaw = axes["horizontal_axis_right_stick"] * self.joystick_yaw_scaling

        #Msg published from joystick_interface to thrust allocation
        wrench_msg = self.create_2d_wrench_message(surge, sway, yaw)

        self.publish_wrench_message(wrench_msg)


        #a = msg.buttons #to remove
        return wrench_msg
    


def main(args=None):
    #rclpy.init(args=args)
    print("hello from main")
    joystick_interface = JoystickInterface().joy_subscriber
    rclpy.spin(joystick_interface)
    joystick_interface.destroy_node()

    rclpy.shutdown()
    return


#we want to move yaml into joystick_interface foulder
#create launch file for joystick interface
#modify pc launch file by taking out
