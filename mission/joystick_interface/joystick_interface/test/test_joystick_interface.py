from joystick_interface.joystick_interface import JoystickInterface
from joystick_interface.joystick_interface import states
import rclpy
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Joy


class TestJoystickInterface:

    def test_right_trigger_linear_converter(self):
        rclpy.init()
        joystick = JoystickInterface()
        assert joystick.right_trigger_linear_converter(1) == 1
        assert joystick.right_trigger_linear_converter(0) == 1.5
        assert joystick.right_trigger_linear_converter(-1) == 2
        rclpy.shutdown()
    
    def test_left_trigger_linear_converter(self):
        rclpy.init()
        joystick = JoystickInterface()
        assert joystick.left_trigger_linear_converter(1) == 1
        assert joystick.left_trigger_linear_converter(0) == 0.75
        assert joystick.left_trigger_linear_converter(-1) == 0.5
        rclpy.shutdown()
    
    def test_2d_wrench_msg(self):
        rclpy.init()
        msg = JoystickInterface().create_2d_wrench_message(2.0, 3.0, 4.0)
        assert msg.force.x == 2.0
        assert msg.force.y == 3.0
        assert msg.torque.z == 4.0
        rclpy.shutdown()

    def test_input_from_controller_into_wrench_msg(self):
        rclpy.init()
        joy_msg = Joy()
        joy_msg.axes = [-1.0, -1.0, 1.0, 0.0, 0.0, 1.0, 0.0,
                        0.0]  #Should have 8 inputs self.joystick_axes_map
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0]  #Should have 11 inputs self.joystick_buttons_map
        wrench_msg = JoystickInterface().joystick_cb(joy_msg)
        assert wrench_msg.force.x == -100.0
        assert wrench_msg.force.y == -100.0
        assert wrench_msg.torque.z == 0.0
        rclpy.shutdown()

    def test_killswitch_button(self):
        rclpy.init()
        joystick = JoystickInterface()
        joystick.state = states.XBOX_MODE
        joy_msg = Joy()
        joy_msg.axes = [-1.0, -1.0, 1.0, 0.0, 0.0, 1.0, 0.0,
                        0.0]  #Should have 8 inputs self.joystick_axes_map
        joy_msg.buttons = [0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                           0]  #Should have 11 inputs self.joystick_buttons_map
        wrench_msg = joystick.joystick_cb(joy_msg)
        assert wrench_msg.force.x == 0.0
        assert wrench_msg.force.y == 0.0
        assert wrench_msg.torque.z == 0.0
        rclpy.shutdown()

    def test_moving_in_and_out_of_xbox_mode(self):
        rclpy.init()
        joystick = JoystickInterface()
        joystick.state = states.XBOX_MODE
        joy_msg = Joy()
        joy_msg.axes = [-1.0, -1.0, 1.0, 0.0, 0.0, 1.0, 0.0,
                        0.0]  #Should have 8 inputs self.joystick_axes_map
        joy_msg.buttons = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0]  #Should have 11 inputs self.joystick_buttons_map
        wrench_msg = joystick.joystick_cb(joy_msg)
        assert wrench_msg.force.x == -100.0
        assert wrench_msg.force.y == -100.0
        assert wrench_msg.torque.z == 0.0
        rclpy.shutdown()



