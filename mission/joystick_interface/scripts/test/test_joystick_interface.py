from scripts.joystick_interface import JoystickInterface
from geometry_msgs.msg import Wrench


class TestJoystickInterface:

    def test_2d_wrench_msg(self):
        msg = JoystickInterface().create_2d_wrench_message(2.0, 3.0, 4.0)
        assert msg.force.x == 2.0
        assert msg.force.y == 3.0
        assert msg.torque.z == 4.0
