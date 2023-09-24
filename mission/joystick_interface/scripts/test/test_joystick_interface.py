from scripts.joystick_interface import JoystickInterface
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Joy


class TestJoystickInterface:

    def start(self):
        rclpy.init()
        j = JoystickInterface()
        rclpy.shutdown()

    def test_2d_wrench_msg(self):
        self.start()
        msg = JoystickInterface().create_2d_wrench_message(2.0, 3.0, 4.0)
        assert msg.force.x == 2.0
        assert msg.force.y == 3.0
        assert msg.torque.z == 4.0

    #def test_publish_wrench_msg(self):
    #msg = JoystickInterface().create_2d_wrench_message(2.0, 3.0, 4.0)
    #publisher = JoystickInterface().publish_wrench_message(msg)
    #assert self.get_logger() == msg

    #def test_joystick_input_output(self):
    #    return
    #
    #
    def test_suscriber(self):
        self.start()
        #subscriber = JoystickInterface().subscriber
        #joystick = JoystickInterface()
        joy_msg = Joy()
        joy_msg.axes = [0.1, 0.2, 0.3]  # Customize axes values as needed
        joy_msg.buttons = [1, 0, 1]
        #JoystickInterface().joystick_cb(joy_msg, subscriber)
        #assert JoystickInterface().node.joy_msg
        assert JoystickInterface().joystick_cb(joy_msg) == joy_msg.buttons
