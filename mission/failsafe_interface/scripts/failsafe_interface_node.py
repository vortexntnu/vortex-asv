#!/usr/bin/python3

import rospy

from std_msgs.msg import String
import RPi._GPIO as GPIO


class FailSafeInterface(object):
    def __init__(
        self,
    ):
        GPIO.setmode(GPIO.BOARD)
        # Fetch the pin numbers from the ASV.yaml file
        self.gpioSoftWareKillSwitch = rospy.get_param(
            "/failsafe/gpio/gpioSoftWareKillSwitch"
        )
        # Initialize the pin
        GPIO.setmode(self.gpioSoftWareKillSwitch, GPIO.OUT)

        
        #Repeat process for the other pins
        self.gpioSoftWareOperationMode = rospy.get_param(
            "/failsafe/gpio/gpioSoftWareOperationMode"
        )
        
        GPIO.setmode(self.gpioSoftWareOperationMode, GPIO.OUT)


        
        #Repeat process for the other pins
        self.gpioFailSafeStatus = rospy.get_param("/failsafe/gpio/gpioFailSafeStatus")
        GPIO.setmode(self.gpioFailSafeStatus, GPIO.IN)


        
        #Repeat process for the other pins
        self.gpioHardWareOperationMode = rospy.get_param(
            "/failsafe/gpio/gpioHardWareOperationMode"
        )

        
        GPIO.setmode(self.gpioHardWareOperationMode, GPIO.IN)
        #init_pin(self.gpioFailSafeStatus)


        # Fetch topics
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

        
        #Create publisher for the hardware fail safe
        self.pubFailSafeStatus = rospy.Publisher(
            topicFailSafeStatus, String, queue_size=10
        )
        
        #Create publisher for the hardware operation mode
        self.pubHardWareOperationMode = rospy.Publisher(
            topicHardWareOperationMode, String, queue_size=10
        )
        
        #Create subscriber for the software kill switch
        self.subSoftWareKillSwitch = rospy.Subscriber(
            topicSoftWareKillSwitch, String, self.writeSoftwareKillSwitch
        )
        
        #Create subscriber for the software operation mode
        self.subSoftWareOperationMode = rospy.Subscriber(
            topicSoftWareOperationMode, String, self.writeSoftwareOperationMode
        )

    def readPins(self):
        # Read the pins
        self.failSafeStatus = GPIO.read(self.gpioFailSafeStatus)
        self.hardwareOperationMode = GPIO.read(self.gpioHardWareOperationMode)

        # Publish accordingly, need to make it into a string
        self.pubFailSafeStatus.publish(str(self.failSafeStatus))
        self.pubHardWareOperationMode.publish(str(self.hardWareOperationMode))

        rospy.loginfo("Received software kill status %s", self.failSafeStatus)

    # Output software kill switch through pin
    def writeSoftwareKillSwitch(self, message):
        self.softWareKillSwitch = message.data
        GPIO.out(self.gpioSoftWareKillSwitch, self.softWareKillSwitch)

    # Output software operation mode through pin
    def writeSoftwareOperationMode(self, message):
        self.softWareOperationMode = message.data
        GPIO.out(self.gpioSoftWareOperationMode, self.softWareOperationMode)


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
