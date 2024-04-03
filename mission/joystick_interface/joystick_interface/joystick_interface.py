#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_msgs.msg import String


class States:
    XBOX_MODE = 1
    AUTONOMOUS_MODE = 2
    NO_GO = 3

class Wired:
    joystick_buttons_map_ = [
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
            "share_button",
        ]

    joystick_axes_map_ = [
            "horizontal_axis_left_stick",  #Sway
            "vertical_axis_left_stick",    #Surge
            "LT",                          #Negative thrust/torque multiplier
            "horizontal_axis_right_stick", #Yaw
            "vertical_axis_right_stick",
            "RT",                          #Positive thrust/torque multiplier
            "dpad_horizontal",
            "dpad_vertical",
        ]

class Wireless:
    joystick_buttons_map_ = [
            "A",
            "B",
            "0",
            "X",
            "Y",
            "0",
            "LB",
            "RB",
            "0",
            "0",
            "back",
            "start",
            "power",
            "stick_button_left",
            "stick_button_right",
            "share_button",
        ]

    joystick_axes_map_ = [
            "horizontal_axis_left_stick",  #Sway
            "vertical_axis_left_stick",    #Surge
            "horizontal_axis_right_stick", #Yaw
            "vertical_axis_right_stick",
            "RT",                          #Positive thrust/torque multiplier
            "LT",                          #Negative thrust/torque multiplier
            "dpad_horizontal",
            "dpad_vertical",
        ]

class JoystickInterface(Node):

    def __init__(self):
        super().__init__('joystick_interface_node')
        self.get_logger().info("Joystick interface is up and running. \n When the XBOX controller is connected, press the killswitch button once to enter XBOX mode.")

        self.last_button_press_time_ = 0
        self.debounce_duration_ = 0.25
        self.state_ = States.NO_GO

        self.joystick_buttons_map_ = []

        self.joystick_axes_map_ = []

        self.joy_subscriber_ = self.create_subscription(Joy, "joystick/joy",
                                                       self.joystick_cb, 1)
        self.wrench_publisher_ = self.create_publisher(Wrench,
                                                      "thrust/wrench_input",
                                                      1)

        self.declare_parameter('surge_scale_factor', 50.0)
        self.declare_parameter('sway_scale_factor', 50.0)
        self.declare_parameter('yaw_scale_factor', 50.0)

        #Gets the scaling factors from the yaml file
        self.joystick_surge_scaling_ = self.get_parameter('surge_scale_factor').value
        self.joystick_sway_scaling_ = self.get_parameter('sway_scale_factor').value
        self.joystick_yaw_scaling_ = self.get_parameter('yaw_scale_factor').value

        #Killswitch publisher
        self.software_killswitch_signal_publisher_ = self.create_publisher(
            Bool, "softWareKillSwitch", 10)
        self.software_killswitch_signal_publisher_.publish(
            Bool(data=True))  #Killswitch is not active

        #Operational mode publisher
        self.operational_mode_signal_publisher_ = self.create_publisher(
            String, "softWareOperationMode", 10)
        
        # Signal that we are not in autonomous mode
        self.operational_mode_signal_publisher_.publish(String(data="XBOX"))

        #Controller publisher
        self.enable_controller_publisher_ = self.create_publisher(
            Bool, "controller/lqr/enable", 10)
        
    def right_trigger_linear_converter(self, rt_input: float) -> float:
        """
        Does a linear conversion from the right trigger input range of (1 to -1) to (1 to 2)

        Args:
            rt_input: The input value of the right trigger, ranging from -1 to 1.

        Returns:
            float: The output value, ranging from 1 to 2.
        """
        output_value = (rt_input + 1) * (-0.5) + 2
        return output_value

    def left_trigger_linear_converter(self, lt_input: float ) -> float:
        """
        Does a linear conversion from the left trigger input range of (1 to -1) to (1 to 0.5)

        Args:
            lt_input: The input value of the left trigger, ranging from -1 to 1.

        Returns:
            float: The output value, ranging from 1 to 2.
        """
        ouput_value = lt_input * 0.25 + 0.75
        return ouput_value

    def create_2d_wrench_message(self, x: float, y: float, yaw: float) -> Wrench:
        """
        Creates a 2D wrench message with the given x, y, and yaw values.

        Args:
            x: The x component of the force vector.
            y: The y component of the force vector.
            yaw: The z component of the torque vector.

        Returns:
            Wrench: A 2D wrench message with the given values.
        """
        wrench_msg = Wrench()
        wrench_msg.force.x = x
        wrench_msg.force.y = y
        wrench_msg.torque.z = yaw
        return wrench_msg

    def transition_to_xbox_mode(self):
        """
        Turns off the controller and signals that the operational mode has switched to Xbox mode.
        """
        self.enable_controller_publisher_.publish(Bool(data=False))
        self.operational_mode_signal_publisher_.publish(String(data="XBOX"))
        self.state_ = States.XBOX_MODE

    def transition_to_autonomous_mode(self):
        """
        Publishes a zero force wrench message and signals that the system is turning on autonomous mode.
        """
        wrench_msg = self.create_2d_wrench_message(0.0, 0.0, 0.0)
        self.wrench_publisher_.publish(wrench_msg)
        self.operational_mode_signal_publisher_.publish(String(data="autonomous mode"))
        self.state_ = States.AUTONOMOUS_MODE

    def joystick_cb(self, msg : Joy) -> Wrench:
        """
        Callback function that receives joy messages and converts them into
        wrench messages to be sent to the thruster allocation node. 
        Handles software killswitch and control mode buttons,
        and transitions between different states of operation.

        Args:
            msg: A ROS message containing the joy input data.

        Returns:
            A ROS message containing the wrench data that was sent to the thruster allocation node.
        """       
        current_time = self.get_clock().now().to_msg()._sec

        buttons = {}
        axes = {}

        if len(msg.buttons) > 12:
            self.joystick_buttons_map_ = Wireless.joystick_buttons_map_
            self.joystick_axes_map_ = Wireless.joystick_axes_map_
        else:
            self.joystick_buttons_map_ = Wired.joystick_buttons_map_
            self.joystick_axes_map_ = Wired.joystick_axes_map_

        for i in range(len(msg.buttons)):
            buttons[self.joystick_buttons_map_[i]] = msg.buttons[i]

        for i in range(len(msg.axes)):
            axes[self.joystick_axes_map_[i]] = msg.axes[i]

        xbox_control_mode_button = buttons["A"]
        software_killswitch_button = buttons["B"]
        software_control_mode_button = buttons["X"]
        left_trigger = axes['LT']
        right_trigger = axes['RT']
        right_trigger = self.right_trigger_linear_converter(right_trigger)
        left_trigger = self.left_trigger_linear_converter(left_trigger)

        surge = axes[
            "vertical_axis_left_stick"] * self.joystick_surge_scaling_ * left_trigger * right_trigger
        sway = -axes[
            "horizontal_axis_left_stick"] * self.joystick_sway_scaling_ * left_trigger * right_trigger
        yaw  = -axes[
            "horizontal_axis_right_stick"] * self.joystick_yaw_scaling_ * left_trigger * right_trigger

        # Debounce for the buttons
        if current_time - self.last_button_press_time_ < self.debounce_duration_:
            software_control_mode_button = False
            xbox_control_mode_button = False
            software_killswitch_button = False

        # If any button is pressed, update the last button press time
        if software_control_mode_button or xbox_control_mode_button or software_killswitch_button:
            self.last_button_press_time_ = current_time

        # Toggle ks on and off
        if software_killswitch_button:
            if self.state_ == States.NO_GO:
                # signal that killswitch is not blocking
                self.software_killswitch_signal_publisher_.publish(
                    Bool(data=False))
                self.transition_to_xbox_mode()
                return

            else:
                self.get_logger().info("SW killswitch", throttle_duration_sec=1)
                # signal that killswitch is blocking
                self.software_killswitch_signal_publisher_.publish(Bool(data=True))
                # Turn off controller in sw killswitch
                self.enable_controller_publisher_.publish(Bool(data=False))
                # Publish a zero wrench message when sw killing
                wrench_msg = self.create_2d_wrench_message(0.0, 0.0, 0.0)
                self.wrench_publisher_.publish(wrench_msg)
                self.state_ = States.NO_GO
                return wrench_msg

        #Msg published from joystick_interface to thrust allocation
        wrench_msg = self.create_2d_wrench_message(surge, sway, yaw)

        if self.state_ == States.XBOX_MODE:
            self.get_logger().info("XBOX mode", throttle_duration_sec=1)
            self.wrench_publisher_.publish(wrench_msg)

            if software_control_mode_button:
                self.transition_to_autonomous_mode()

        if self.state_ == States.AUTONOMOUS_MODE:
            self.get_logger().info("autonomous mode", throttle_duration_sec=1)

            if xbox_control_mode_button:
                self.transition_to_xbox_mode()

        return wrench_msg


def main():
    rclpy.init()
    joystick_interface = JoystickInterface()
    rclpy.spin(joystick_interface)
    joystick_interface.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
