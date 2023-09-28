from scripts.joystick_interface import JoystickInterface
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Joy


class TestJoystickInterface:

    def test_2d_wrench_msg(self):
        #self.start()
        msg = JoystickInterface().create_2d_wrench_message(2.0, 3.0, 4.0)
        assert msg.force.x == 2.0
        assert msg.force.y == 3.0
        assert msg.torque.z == 4.0
        rclpy.shutdown()

    def test_suscriber(self):
        joy_msg = Joy()
        joy_msg.axes = [0.1, 0.2, 0.3]
        joy_msg.buttons = [1, 0, 1]
        assert JoystickInterface().joystick_cb(joy_msg) == joy_msg.buttons
        rclpy.shutdown()
    
    
