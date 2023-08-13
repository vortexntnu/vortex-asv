#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Wrench
from std_msgs.msg import Bool

class states:
    XBOX_MODE = 1
    AUTONOMOUS_MODE = 2
    NO_GO = 3 # Do nothing


class JoystickInterface:

    def __init__(self):
        """
        Define constants used in the joystick mapping, and any ros
        specifics
        """

        rospy.init_node("joystick_interface")

        self.last_button_press_time = 0
        self.debounce_duration = 0.25
        self.state = states.NO_GO

        self.joystick_surge_scaling = rospy.get_param(
            "/joystick/scaling/surge")
        self.joystick_sway_scaling = rospy.get_param("/joystick/scaling/sway")
        self.joystick_yaw_scaling = rospy.get_param("/joystick/scaling/yaw")

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
            "horizontal_axis_left_stick",
            "vertical_axis_left_stick",
            "LT",
            "horizontal_axis_right_stick",
            "vertical_axis_right_stick",
            "RT",
            "dpad_horizontal",
            "dpad_vertical",
        ]

        self.joystick_sub = rospy.Subscriber("/joystick/joy",
                                             Joy,
                                             self.joystick_cb,
                                             queue_size=1)

        self.force_pub = rospy.Publisher("/thrust/force_input",
                                         Wrench,
                                         queue_size=1)
        self.torque_pub = rospy.Publisher("/thrust/torque_input",
                                          Wrench,
                                          queue_size=1)
        
        self.software_killswitch_signal_publisher = rospy.Publisher("/softWareKillSwtich", Bool, queue_size=10)
        self.operational_mode_signal_publisher = rospy.Publisher("/softWareOperationMode", Bool, queue_size=10)
        self.enable_controller_publisher = rospy.Publisher("/controller/lqr/enable", Bool, queue_size=10)

        rospy.loginfo("Joystick interface is up and running")

        self.software_killswitch_signal_publisher.publish(Bool(False)) # initially signal that we are blocking
        self.operational_mode_signal_publisher.publish(Bool(True)) # Signal that we are not in autonomous mode

    def create_2d_wrench_message(self, x, y, yaw):
        wrench_msg = Wrench()
        wrench_msg.force.x = x
        wrench_msg.force.y = y
        wrench_msg.torque.z = yaw
        return wrench_msg
    
    def publish_wrench_message(self, wrench):
        self.force_pub.publish(wrench)
        self.torque_pub.publish(wrench)


    def transition_to_xbox_mode(self):
        # We want to turn off controller when moving to xbox mode
        self.enable_controller_publisher.publish(Bool(False))

        self.operational_mode_signal_publisher.publish(Bool(True)) # signal that we enter xbox mode

        self.state = states.XBOX_MODE

    def transition_to_autonomous_mode(self):
        # We want to publish zero force once when transitioning
        wrench_msg = self.create_2d_wrench_message(0.0, 0.0, 0.0)
        self.publish_wrench_message(wrench_msg)

        self.operational_mode_signal_publisher.publish(Bool(False)) # signal that we are turning on autonomous mode

        self.state = states.AUTONOMOUS_MODE

        

    def joystick_cb(self, msg):
        current_time = rospy.Time.now().to_sec()

        buttons = {}
        axes = {}

        for i in range(len(msg.buttons)):
            buttons[self.joystick_buttons_map[i]] = msg.buttons[i]

        for i in range(len(msg.axes)):
            axes[self.joystick_axes_map[i]] = msg.axes[i]

        surge = axes["vertical_axis_left_stick"] * self.joystick_surge_scaling
        sway = axes["horizontal_axis_left_stick"] * self.joystick_sway_scaling
        yaw = axes["horizontal_axis_right_stick"] * self.joystick_yaw_scaling

        software_control_mode_button = buttons["X"]
        xbox_control_mode_button = buttons["A"]
        software_killswitch_button = buttons["B"]

        # Debounce for the buttons
        if current_time - self.last_button_press_time < self.debounce_duration:
            software_control_mode_button = False
            xbox_control_mode_button = False
            software_killswitch_button = False

        # If any button is pressed, update the last button press time
        if software_control_mode_button or xbox_control_mode_button or software_killswitch_button:
            self.last_button_press_time = current_time

        if self.state == states.NO_GO and software_killswitch_button: # Toggle ks on and off
            self.software_killswitch_signal_publisher.publish(Bool(True)) # signal that killswitch is not blocking
            self.transition_to_xbox_mode()
            return

        if software_killswitch_button:
            rospy.loginfo("SW killswitch")
            self.software_killswitch_signal_publisher.publish(Bool(False)) # signal that killswitch is blocking
            self.enable_controller_publisher.publish(Bool(False)) # Turn off controller in sw killswitch
            # Publish a zero wrench message when sw killing
            wrench_msg = self.create_2d_wrench_message(0.0, 0.0, 0.0)
            self.publish_wrench_message(wrench_msg)
            self.state = states.NO_GO
            return
        
        wrench_msg = self.create_2d_wrench_message(surge, sway, yaw)

        if self.state == states.XBOX_MODE:
            rospy.loginfo("xbox mode")
            self.publish_wrench_message(wrench_msg)

            if software_control_mode_button:
                self.transition_to_autonomous_mode()



        if self.state == states.AUTONOMOUS_MODE:
            rospy.loginfo("autonomous mode")

            if xbox_control_mode_button:
                self.transition_to_xbox_mode()


if __name__ == "__main__":

    joystick_interface = JoystickInterface()
    rospy.spin()
