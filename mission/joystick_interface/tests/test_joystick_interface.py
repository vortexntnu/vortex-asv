import rclpy
from joystick_interface.joystick_interface import JoystickInterface, States
from sensor_msgs.msg import Joy


class TestJoystickInterface:
    # test that the linear conversion from (1 to -1) to (1 to 2) is working
    def test_right_trigger_linear_converter(self):
        rclpy.init()
        joystick = JoystickInterface()
        assert joystick.right_trigger_linear_converter(1) == 1
        assert joystick.right_trigger_linear_converter(0) == 1.5
        assert joystick.right_trigger_linear_converter(-1) == 2
        rclpy.shutdown()

    # test that the linear conversion from (1 to -1) to (1 to 0.5) is working
    def test_left_trigger_linear_converter(self):
        rclpy.init()
        joystick = JoystickInterface()
        assert joystick.left_trigger_linear_converter(1) == 1
        assert joystick.left_trigger_linear_converter(0) == 0.75
        assert joystick.left_trigger_linear_converter(-1) == 0.5
        rclpy.shutdown()

    # test that the 2d wrench msg is created successfully
    def test_2d_wrench_msg(self):
        rclpy.init()
        msg = JoystickInterface().create_2d_wrench_message(2.0, 3.0, 4.0)
        assert msg.force.x == 2.0
        assert msg.force.y == 3.0
        assert msg.torque.z == 4.0
        rclpy.shutdown()

    # Test that the callback function will be able to interpret the joy msg
    def test_input_from_controller_into_wrench_msg(self):
        rclpy.init()
        joy_msg = Joy()
        joy_msg.axes = [-1.0, -1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        wrench_msg = JoystickInterface().joystick_cb(joy_msg)
        assert wrench_msg.force.x == -1.0 * JoystickInterface().joystick_surge_scaling_
        assert wrench_msg.force.y == 1.0 * JoystickInterface().joystick_sway_scaling_
        assert wrench_msg.torque.z == 0.0
        rclpy.shutdown()

    # When the killswitch button is activated in the buttons list, it should output a wrench msg with only zeros
    def test_killswitch_button(self):
        rclpy.init()
        joystick = JoystickInterface()
        joystick.state_ = States.XBOX_MODE
        joy_msg = Joy()
        joy_msg.axes = [-1.0, -1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        joy_msg.buttons = [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        wrench_msg = joystick.joystick_cb(joy_msg)
        assert wrench_msg.force.x == 0.0
        assert wrench_msg.force.y == 0.0
        assert wrench_msg.torque.z == 0.0
        rclpy.shutdown()

    # When we move into XBOX mode it should still be able to return this wrench message
    def test_moving_in_of_xbox_mode(self):
        rclpy.init()
        joystick = JoystickInterface()
        joystick.state_ = States.XBOX_MODE
        joy_msg = Joy()
        joy_msg.axes = [-1.0, -1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        joy_msg.buttons = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        wrench_msg = joystick.joystick_cb(joy_msg)
        assert wrench_msg.force.x == -1.0 * JoystickInterface().joystick_surge_scaling_
        assert wrench_msg.force.y == 1.0 * JoystickInterface().joystick_sway_scaling_
        assert wrench_msg.torque.z == 0.0
        rclpy.shutdown()
