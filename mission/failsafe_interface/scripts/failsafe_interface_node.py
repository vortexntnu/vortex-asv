#!/usr/bin/python3

import rospy

from std_msgs.msg import String, Bool
import RPi._GPIO as GPIO


class FailSafeInterface(object):

    def __init__(self, ):

        GPIO.setmode(GPIO.BOARD)

        #Temporary solution because .yaml file is non-cooperative
        self.gpioSoftWareKillSwitch = 13

        # Fetch the pin numbers from the ASV.yaml file
        #self.gpioSoftWareKillSwitch = rospy.get_param(
        #    "/failsafe/gpio/gpioSoftWareKillSwitch"
        #)

        # Initialize the pin
        GPIO.setup(self.gpioSoftWareKillSwitch, GPIO.OUT)

        GPIO.output(self.gpioSoftWareKillSwitch, 1)

        self.gpioSoftWareOperationMode = 18

        #Repeat process for the other pins
        #self.gpioSoftWareOperationMode = rospy.get_param(
        #    "/failsafe/gpio/gpioSoftWareOperationMode"
        #)

        GPIO.setup(self.gpioSoftWareOperationMode, GPIO.OUT)

        self.gpioHardWareOperationMode = 16

        GPIO.output(self.gpioHardWareOperationMode, 0)

        #Repeat process for the other pins

        self.gpioFailSafeStatus = 15

        #self.gpioFailSafeStatus = rospy.get_param("/failsafe/gpio/gpioFailSafeStatus")
        GPIO.setup(self.gpioFailSafeStatus, GPIO.IN)

        #Repeat process for the other pins

        #self.gpioHardWareOperationMode = rospy.get_param(
        #    "/failsafe/gpio/gpioHardWareOperationMode"
        #)

        GPIO.setup(self.gpioHardWareOperationMode, GPIO.IN)
        #init_pin(self.gpioFailSafeStatus)

        # Fetch topics
        topicFailSafeStatus = '/failSafeStatus'
        topicHardWareOperationMode = '/hardWareOperationMode'

        topicSoftWareKillSwitch = '/softWareKillSwtich'
        topicSoftWareOperationMode = '/softWareOperationMode'

        #topicFailSafeStatus = rospy.get_param("/failsafe/publishers/failSafeStatus")
        #topicHardWareOperationMode = rospy.get_param(
        #    "/failsafe/publishers/hardwareOperationMode"
        #)

        #topicSoftWareKillSwitch = rospy.get_param(
        #    "/failsafe/subscribers/softWareKillSwitch"
        #)

        #topicSoftWareOperationMode = rospy.get_param(
        #    "/failsafe/subscribers/softWareOperationMode"
        #)

        #Create publisher for the hardware fail safe
        self.pubFailSafeStatus = rospy.Publisher(topicFailSafeStatus,
                                                 Bool,
                                                 queue_size=10)

        #Create publisher for the hardware operation mode
        self.pubHardWareOperationMode = rospy.Publisher(
            topicHardWareOperationMode, Bool, queue_size=10)

        #Create subscriber for the software kill switch
        self.subSoftWareKillSwitch = rospy.Subscriber(
            topicSoftWareKillSwitch, Bool, self.writeSoftwareKillSwitch)

        #Create subscriber for the software operation mode
        self.subSoftWareOperationMode = rospy.Subscriber(
            topicSoftWareOperationMode, Bool, self.writeSoftwareOperationMode)

    def readPins(self):
        # Read the pins
        self.failSafeStatus = GPIO.input(self.gpioFailSafeStatus)
        self.hardWareOperationMode = GPIO.input(self.gpioHardWareOperationMode)

        # Publish accordingly, need to make it into a string
        self.pubFailSafeStatus.publish(self.failSafeStatus)
        self.pubHardWareOperationMode.publish(self.hardWareOperationMode)

        #rospy.loginfo("Received hardware kill status %s", self.failSafeStatus)

    # Output software kill switch through pin
    def writeSoftwareKillSwitch(self, message):
        rospy.loginfo("softwarekill")
        self.softWareKillSwitch = message.data
        GPIO.output(self.gpioSoftWareKillSwitch, self.softWareKillSwitch)
        rospy.loginfo("Outputting software killswitch %s",
                      self.softWareKillSwitch)

    # Output software operation mode through pin
    def writeSoftwareOperationMode(self, message):
        rospy.loginfo("softwareopmode")
        self.softWareOperationMode = message.data
        GPIO.output(self.gpioSoftWareOperationMode, self.softWareOperationMode)
        rospy.loginfo("Outputting software operation mode %s",
                      self.softWareOperationMode)


def FailSafeNodeSetup():
    rospy.init_node("failsafe_interface_node", anonymous=True)
    print("Set up the node")
    failsafe = FailSafeInterface()

    while not rospy.is_shutdown():
        failsafe.readPins()


if __name__ == "__main__":
    try:
        print("Attempting to set up node")
        FailSafeNodeSetup()
    except rospy.ROSInterruptException:
        pass
