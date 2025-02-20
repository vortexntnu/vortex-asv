#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Wrench
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String

from joystick_interface_asv.joystick_utils import States, Wired, WirelessXboxSeriesX


class JoystickInterface(Node):
    def __init__(self):
        super().__init__('joystick_interface_node')

        self.last_button_press_time_ = 0
        self.mode_ = States.KILLSWITCH
        self.joystick_buttons_map_ = []
        self.joystick_axes_map_ = []
        self.previous_state_ = States.KILLSWITCH
        self.mode_logger_done_ = False

        self.get_params()
        self.set_publishers_and_subscribers()

        self.get_logger().info(
            f"Joystick interface node started. Current mode: {self.mode_}"
        )

    def get_params(self):
        self.surge_scale_ = (
            self.declare_parameter("surge_scale_factor", 1.0)
            .get_parameter_value()
            .double_value
        )
        self.sway_scale_ = (
            self.declare_parameter("sway_scale_factor", 1.0)
            .get_parameter_value()
            .double_value
        )
        self.yaw_scale_ = (
            self.declare_parameter("yaw_scale_factor", 1.0)
            .get_parameter_value()
            .double_value
        )
        self.debounce_duration_ = (
            self.declare_parameter("debounce_duration", 1.0)
            .get_parameter_value()
            .double_value
        )

        topics = [
            "wrench",
            "operation_mode",
            "killswitch",
            "joy",
        ]

        for topic in topics:
            self.declare_parameter(f"topics.{topic}", "_")
            setattr(
                self,
                f"{topic}_topic",
                self.get_parameter(f"topics.{topic}").value,
            )

    def set_publishers_and_subscribers(self):
        self.wrench_publisher_ = self.create_publisher(Wrench, self.wrench_topic, 1)
        self.operational_mode_signal_publisher_ = self.create_publisher(
            String, self.operation_mode_topic, 1
        )
        self.software_killswitch_signal_publisher_ = self.create_publisher(
            Bool, self.killswitch_topic, 1
        )
        self.joystick_subscriber_ = self.create_subscription(
            Joy, "joy", self.joystick_cb, 10
        )

    def right_trigger_linear_converter(self, rt_input: float) -> float:
        """Does a linear conversion from the right trigger input range of (1 to -1) to (1 to 2).

        Args:
            rt_input: The input value of the right trigger, ranging from -1 to 1.

        Returns:
            float: The output value, ranging from 1 to 2.
        """
        output_value = (rt_input + 1) * (-0.5) + 2
        return output_value

    def left_trigger_linear_converter(self, lt_input: float) -> float:
        """Does a linear conversion from the left trigger input range of (1 to -1) to (1 to 0.5).

        Args:
            lt_input: The input value of the left trigger, ranging from -1 to 1.

        Returns:
            float: The output value, ranging from 1 to 0.5.
        """
        ouput_value = lt_input * 0.25 + 0.75
        return ouput_value

    def create_2d_wrench_message(self, x: float, y: float, yaw: float) -> Wrench:
        """Creates a 2D wrench message with the given x, y, and yaw values.

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
        """Turns off the controller and signals that the operational mode has switched to Xbox mode."""
        msg = String()
        msg.data = "XBOX"
        self.operational_mode_signal_publisher_.publish(msg)
        self.mode_ = States.XBOX_MODE
        self.previous_state_ = States.XBOX_MODE
        self.mode_logger_done_ = False

    def transition_to_autonomous_mode(self):
        """Publishes a zero force wrench message and signals that the system is turning on autonomous mode."""
        wrench_msg = self.create_2d_wrench_message(0.0, 0.0, 0.0)
        self.wrench_publisher_.publish(wrench_msg)
        msg = String()
        msg.data = "autonomous mode"
        self.operational_mode_signal_publisher_.publish(msg)
        self.mode_ = States.AUTONOMOUS_MODE
        self.previous_state_ = States.AUTONOMOUS_MODE
        self.mode_logger_done_ = False

    def set_stationkeeping_pose_callback(self, future):
        """Callback function for the set_stationkeeping_pose service."""
        try:
            response = future.result()
            self.get_logger().info(f"Response: {response}")
        except Exception as e:
            self.get_logger().info(f"Service call failed {str(e)}")

    def joystick_cb(self, msg: Joy):
        """Receives joy messages and converts them into wrench messages.

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

        # Check if the controller is wireless (has 16 buttons) or wired
        if len(msg.buttons) == 16:
            self.joystick_buttons_map_ = WirelessXboxSeriesX.joystick_buttons_map_
            self.joystick_axes_map_ = WirelessXboxSeriesX.joystick_axes_map_
        else:
            self.joystick_buttons_map_ = Wired.joystick_buttons_map_
            self.joystick_axes_map_ = Wired.joystick_axes_map_

        # Populate buttons dictionary
        for i, button_name in enumerate(self.joystick_buttons_map_):
            if i < len(msg.buttons):
                buttons[button_name] = msg.buttons[i]
            else:
                # Assuming default value if button is not present
                buttons[button_name] = 0

        # Populate axes dictionary
        for i, axis_name in enumerate(self.joystick_axes_map_):
            if i < len(msg.axes):
                axes[axis_name] = msg.axes[i]
            else:
                # Assuming default value if axis is not present
                axes[axis_name] = 0.0

        xbox_control_mode_button = buttons.get("A", 0)
        software_killswitch_button = buttons.get("B", 0)
        software_control_mode_button = buttons.get("X", 0)
        left_trigger = self.left_trigger_linear_converter(axes.get('LT', 0.0))
        right_trigger = self.right_trigger_linear_converter(axes.get('RT', 0.0))

        surge = (
            axes.get("vertical_axis_left_stick", 0.0)
            * self.surge_scale_
            * left_trigger
            * right_trigger
        )
        sway = (
            -axes.get("horizontal_axis_left_stick", 0.0)
            * self.sway_scale_
            * left_trigger
            * right_trigger
        )
        yaw = (
            -axes.get("horizontal_axis_right_stick", 0.0)
            * self.yaw_scale_
            * left_trigger
            * right_trigger
        )

        # Debounce for the buttons
        if current_time - self.last_button_press_time_ < self.debounce_duration_:
            software_control_mode_button = False
            xbox_control_mode_button = False

        # If any button is pressed, update the last button press time
        if (
            software_control_mode_button
            or xbox_control_mode_button
            or software_killswitch_button
        ):
            self.last_button_press_time_ = current_time

        # Toggle ks on and off
        if software_killswitch_button:
            if self.mode_ == States.KILLSWITCH:
                self.software_killswitch_signal_publisher_.publish(Bool(data=False))

                if self.previous_state_ == States.XBOX_MODE:
                    self.transition_to_xbox_mode()

                else:
                    self.transition_to_autonomous_mode()
                return

            else:
                self.get_logger().info("SW killswitch", throttle_duration_sec=1)
                self.software_killswitch_signal_publisher_.publish(Bool(data=True))
                wrench_msg = self.create_2d_wrench_message(0.0, 0.0, 0.0)
                self.wrench_publisher_.publish(wrench_msg)
                self.mode_ = States.KILLSWITCH
                return wrench_msg

        # Msg published from joystick_interface to thrust allocation
        wrench_msg = self.create_2d_wrench_message(surge, sway, yaw)

        if self.mode_ == States.XBOX_MODE:
            if not self.mode_logger_done_:
                self.get_logger().info("XBOX mode")
                self.mode_logger_done_ = True

            self.wrench_publisher_.publish(wrench_msg)

            if software_control_mode_button:
                self.transition_to_autonomous_mode()

        elif self.mode_ == States.AUTONOMOUS_MODE:
            if not self.mode_logger_done_:
                self.get_logger().info("autonomous mode")
                self.mode_logger_done_ = True

            if xbox_control_mode_button:
                self.transition_to_xbox_mode()

        else:
            if not self.mode_logger_done_:
                self.get_logger().info("Killswitch is active")
                self.mode_logger_done_ = True
            return


def main():
    rclpy.init()
    joystick_interface = JoystickInterface()
    rclpy.spin(joystick_interface)
    joystick_interface.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
