from scripts.joystick_interface import JoystickInterface
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Joy


class TestJoystickInterface:

    def test_2d_wrench_msg(self):
        msg = JoystickInterface().create_2d_wrench_message(2.0, 3.0, 4.0)
        assert msg.force.x == 2.0
        assert msg.force.y == 3.0
        assert msg.torque.z == 4.0
        rclpy.shutdown()

    def test_input_from_controller_into_wrench_msg(self):
        joy_msg = Joy()
        joy_msg.axes = [-1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]#Should have 8 inputs self.joystick_axes_map
        joy_msg.buttons = [1, -1, -1, 0, 0, 6, 0, 0, 0, 0, 0]#Should have 11 inputs self.joystick_buttons_map 
        wrench_msg = JoystickInterface().joystick_cb(joy_msg)
        assert wrench_msg.force.x == -100.0
        assert wrench_msg.force.y == -100.0
        assert wrench_msg.torque.z == 0.0
        rclpy.shutdown()
    


    
