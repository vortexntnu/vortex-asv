#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class states:
    XBOX_MODE = 1
    AUTONOMOUS_MODE = 2
    NO_GO = 3  # Do nothing


class JoystickInterface(Node):

    def __init__(self):
        super().__init__('joystick_interface_node')
        self.get_logger().info("Joystick interface is up and running")

        self.last_button_press_time = 0
        self.debounce_duration = 0.25
        self.state = states.NO_GO

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
            "horizontal_axis_left_stick",  #Translation (Left and Right)
            "vertical_axis_left_stick",    #Translation (Forwards and Backwards)
            "LT",                          #Negative thrust/torque multiplier
            "horizontal_axis_right_stick", #Rotation
            "vertical_axis_right_stick",
            "RT",                          #Positive thrust/torque multiplier
            "dpad_horizontal",
            "dpad_vertical",
        ]

        self.joy_subscriber = self.create_subscription(Joy, "joystick/joy",
                                                       self.joystick_cb, 1)
        self.wrench_publisher = self.create_publisher(Wrench,
                                                      "thrust/wrench_input",
                                                      1)

        self.declare_parameter('surge_scale_factor', 100.0)
        self.declare_parameter('sway_scale_factor', 100.0)
        self.declare_parameter('yaw_scale_factor', 100.0)

        #Gets the scaling factors from the yaml file
        self.joystick_surge_scaling = self.get_parameter('surge_scale_factor').value
        self.joystick_sway_scaling = self.get_parameter('sway_scale_factor').value
        self.joystick_yaw_scaling = self.get_parameter('yaw_scale_factor').value

        #Killswitch publisher
        self.software_killswitch_signal_publisher = self.create_publisher(
            Bool, "softWareKillSwitch", 10)
        self.software_killswitch_signal_publisher.publish(
            Bool(data=False))  #Killswitch is not active

        #Operational mode publisher
        self.operational_mode_signal_publisher = self.create_publisher(
            Bool, "softWareOperationMode", 10)
        # Signal that we are not in autonomous mode
        self.operational_mode_signal_publisher.publish(Bool(data=True))

        #Controller publisher
        self.enable_controller_publisher = self.create_publisher(
            Bool, "controller/lqr/enable", 10)

    #does a linear conversion from trigger inputs (1 to -1) to (1 to 2)
    def right_trigger_linear_converter(self, rt_input):
        output_value = (rt_input + 1) * (-0.5) + 2
        return output_value

    #does a linear conversion from trigger input (1 to -1) to (1 to 0.5)
    def left_trigger_linear_converter(self, lt_input):
        ouput_value = lt_input * 0.25 + 0.75
        return ouput_value

    def create_2d_wrench_message(self, x, y, yaw):
        wrench_msg = Wrench()
        wrench_msg.force.x = x
        wrench_msg.force.y = y
        wrench_msg.torque.z = yaw
        return wrench_msg

    def publish_wrench_message(self, wrench):
        self.wrench_publisher.publish(wrench)

    def transition_to_xbox_mode(self):
        # We want to turn off controller when moving to xbox mode
        self.enable_controller_publisher.publish(Bool(data=False))
        # signal that we enter xbox mode
        self.operational_mode_signal_publisher.publish(Bool(data=True))
        self.state = states.XBOX_MODE

    def transition_to_autonomous_mode(self):
        # We want to publish zero force once when transitioning
        wrench_msg = self.create_2d_wrench_message(0.0, 0.0, 0.0)
        self.publish_wrench_message(wrench_msg)
        # signal that we are turning on autonomous mode
        self.operational_mode_signal_publisher.publish(Bool(data=False))
        self.state = states.AUTONOMOUS_MODE

    def joystick_cb(self, msg):
        current_time = self.get_clock().now().to_msg()._sec

        #Input from controller to joystick_interface
        buttons = {}
        axes = {}

        for i in range(len(msg.buttons)):
            buttons[self.joystick_buttons_map[i]] = msg.buttons[i]

        for i in range(len(msg.axes)):
            axes[self.joystick_axes_map[i]] = msg.axes[i]

        xbox_control_mode_button = buttons["A"]
        software_killswitch_button = buttons["B"]
        software_control_mode_button = buttons["X"]
        left_trigger = axes['LT']
        right_trigger = axes['RT']
        right_trigger = self.right_trigger_linear_converter(right_trigger)
        left_trigger = self.left_trigger_linear_converter(left_trigger)

        surge = axes[
            "vertical_axis_left_stick"] * self.joystick_surge_scaling * left_trigger * right_trigger
        sway = axes[
            "horizontal_axis_left_stick"] * self.joystick_sway_scaling * left_trigger * right_trigger
        yaw = axes[
            "horizontal_axis_right_stick"] * self.joystick_yaw_scaling * left_trigger * right_trigger

        # Debounce for the buttons
        if current_time - self.last_button_press_time < self.debounce_duration:
            software_control_mode_button = False
            xbox_control_mode_button = False
            software_killswitch_button = False

        # If any button is pressed, update the last button press time
        if software_control_mode_button or xbox_control_mode_button or software_killswitch_button:
            self.last_button_press_time = current_time

        # Toggle ks on and off
        if self.state == states.NO_GO and software_killswitch_button:
            # signal that killswitch is not blocking
            self.software_killswitch_signal_publisher.publish(Bool(data=True))
            self.transition_to_xbox_mode()
            return

        if software_killswitch_button:
            self.get_logger().info("SW killswitch", throttle_duration_sec=1)
            # signal that killswitch is blocking
            self.software_killswitch_signal_publisher.publish(Bool(data=False))
            # Turn off controller in sw killswitch
            self.enable_controller_publisher.publish(Bool(data=False))
            # Publish a zero wrench message when sw killing
            wrench_msg = self.create_2d_wrench_message(0.0, 0.0, 0.0)
            self.publish_wrench_message(wrench_msg)
            self.state = states.NO_GO
            return wrench_msg

        #Msg published from joystick_interface to thrust allocation
        wrench_msg = self.create_2d_wrench_message(surge, sway, yaw)

        if self.state == states.XBOX_MODE:
            self.get_logger().info("XBOX mode", throttle_duration_sec=1)
            self.publish_wrench_message(wrench_msg)

            if software_control_mode_button:
                self.transition_to_autonomous_mode()

        if self.state == states.AUTONOMOUS_MODE:
            self.get_logger().info("autonomous mode", throttle_duration_sec=1)

            if xbox_control_mode_button:
                self.transition_to_xbox_mode()

        return wrench_msg


def main():
    rclpy.init()

    joystick_interface = JoystickInterface()
    print(joystick_interface.joystick_surge_scaling)
    rclpy.spin(joystick_interface)

    joystick_interface.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
