#!/usr/bin/python3

import rospy

from std_msgs.msg import String

from gpio_functions import *

class FailSafeInterface(object):
    def __init__(
        self,
    ):
        self.gpioSoftWareKillSwitch = rospy.get_param(
            "/failsafe/gpio/gpioSoftWareKillSwitch"
        )
        print(self.gpioSoftWareKillSwitch)
        init_pin(self.gpioSoftWareKillSwitch)

        self.gpioSoftWareOperationMode = rospy.get_param(
            "/failsafe/gpio/gpioSoftWareOperationMode"
        )
        print(self.gpioSoftWareOperationMode)
        init_pin(self.gpioSoftWareOperationMode)

        self.gpioFailSafeStatus = rospy.get_param("/failsafe/gpio/gpioFailSafeStatus")
        init_pin(self.gpioFailSafeStatus)

        self.gpioHardWareOperationMode = rospy.get_param(
            "/failsafe/gpio/gpioHardWareOperationMode"
        )
        print(self.gpioHardWareOperationMode)
        init_pin(self.gpioHardWareOperationMode)


        topicFailSafeStatus = rospy.get_param("/failsafe/publishers/failSafeStatus")
        topicHardWareOperationMode = rospy.get_param(
            "/failsafe/publishers/hardwareOperationMode"
        )
        topicSoftWareKillSwitch = rospy.get_param(
            "/failsafe/subscribers/softWareKillSwitch"
        )
        
        topicSoftWareOperationMode = rospy.get_param(
            "/failsafe/subscribers/softWareOperationMode"
        )

        """
        GPIO.setmode(GPIO.BCM)  # Use BCM numbering

        GPIO.setup(self.gpioFailSafeStatus, GPIO.IN)  # Set pin 4 as input
        GPIO.setup(self.gpioHardWareOperationMode, GPIO.IN)  # Set pin 4 as input
        GPIO.setup(self.gpioSoftWareKillSwitch, GPIO.OUT)  # Set pin 4 as input
        GPIO.setup(self.gpioSoftWareOperationMode, GPIO.OUT)  # Set pin 4 as input

        self.failSafeStatus = 0  # 1 means normal operation, 0 means error
        self.hardWareOperationMode = 0  # 1 means software operation, 0 means RX manual
        self.softWareKillSwitch = 0  # 1 means normal operation, 0 means error
        self.softWareOperationMode = 0  # 1 means software manual, 0 means autonomous
        """
        set_pin_as_output(self.gpioSoftWareKillSwitch)
        set_pin_as_output(self.gpioSoftWareOperationMode)
        set_pin_as_input(self.gpioFailSafeStatus)
        set_pin_as_input(self.gpioHardWareOperationMode)




        self.pubFailSafeStatus = rospy.Publisher(
            topicFailSafeStatus, String, queue_size=10
        )
        self.pubHardWareOperationMode = rospy.Publisher(
            topicHardWareOperationMode, String, queue_size=10
        )

        self.subSoftWareKillSwitch = rospy.Subscriber(
            topicSoftWareKillSwitch, String, self.writeSoftwareKillSwitch
        )
        self.subSoftWareOperationMode = rospy.Subscriber(
            topicSoftWareOperationMode, String, self.writeSoftwareOperationMode
        )

    def readPins(self):
        #self.failSafeStatus = GPIO.input(self.gpioFailSafeStatus)
        self.failSafeStatus = pin_read(self.gpioFailSafeStatus)
        #self.hardWareOperationMode = GPIO.input(self.gpioHardWareOperationMode)
        self.hardWareOperationMode = pin_read(self.gpioHardWareOperationMode)

        self.pubFailSafeStatus.publish(str(self.failSafeStatus))
        self.pubHardWareOperationMode.publish(str(self.hardWareOperationMode))

        rospy.loginfo("Received software kill status %s", self.failSafeStatus)

    def writeSoftwareKillSwitch(self, message):
        self.softWareKillSwitch = message.data
        pin_write(self.gpioSoftWareKillSwitch, self.softWareKillSwitch)
        #GPIO.output(self.gpioSoftWareKillSwitch, self.softWareKillSwitch)

    def writeSoftwareOperationMode(self, message):
        self.softWareOperationMode = message.data
        pin_write(self.gpioSoftWareOperationMode, self.softWareOperationMode)
        #GPIO.output(self.gpioSoftWareOperationMode, self.softWareOperationMode)


def FailSafeNodeSetup():
    rospy.init_node("failsafe_interface_node", anonymous=True)
    print("Set up the node")
    failsafe = FailSafeInterface()

    while not rospy.is_shutdown():
        print("Got into the loop")
        failsafe.readPins()


if __name__ == "__main__":
    try:
        print("Attempting to set up node")
        FailSafeNodeSetup()
    except rospy.ROSInterruptException:
        pass

