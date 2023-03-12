import RPi.GPIO as GPIO
import rospy

from std_msgs.msg import String

class FailSafeInterface(object):
    def __init__(
        self,  

    ):
        self.gpioSoftWareKillSwitch = rospy.get_param("/failsafe/gpio/gpioSoftWareKillSwitch")
        self.gpioSoftWareOperationMode = rospy.get_param("/failsafe/gpio/gpioSoftWareOperationMode")
        self.gpioFailSafeStatus = rospy.get_param("/failsafe/gpio/gpioFailSafeStatus")
        self.gpioHardWareOperationMode = rospy.get_param("/failsafe/gpio/gpioHardWareOperationMode")

        topicFailSafeStatus = rospy.get_param("/failsafe/publishers/failSafeStatus")
        topicHardWareOperationMode = rospy.get_param("/failsafe/publishers/hardwareOperationMode")
        topicSoftWareKillSwitch = rospy.get_param("/failsafe/subscribers/softWareKillSwitch")
        topicSoftWareOperationMode = rospy.get_param("/failsafe/subscribers/softWareOperationMode")

        GPIO.setmode(GPIO.BCM)  # Use BCM numbering

        GPIO.setup(self.gpioFailSafeStatus, GPIO.IN)  # Set pin 4 as input
        GPIO.setup(self.gpioHardWareOperationMode, GPIO.IN)  # Set pin 4 as input
        GPIO.setup(self.gpioSoftWareKillSwitch, GPIO.OUT)  # Set pin 4 as input
        GPIO.setup(self.gpioSoftWareOperationMode, GPIO.OUT)  # Set pin 4 as input

        self.failSafeStatus = 0 #1 means normal operation, 0 means error
        self.hardWareOperationMode = 0 #1 means software operation, 0 means RX manual
        self.softWareKillSwitch = 0 #1 means normal operation, 0 means error
        self.softWareOperationMode = 0 #1 means software manual, 0 means autonomous

        self.pubFailSafeStatus = rospy.Publisher(topicFailSafeStatus, String, queue_size=10)
        self.pubHardWareOperationMode = rospy.Publisher(topicHardWareOperationMode, String, queue_size=10)

        self.subSoftWareKillSwitch = rospy.Subscriber(topicSoftWareKillSwitch, String, self.writeSoftwareKillSwitch)
        self.subSoftWareOperationMode = rospy.Subscriber(topicSoftWareOperationMode, String, self.writeSoftwareOperationMode)

    def readPins(self):
        self.failSafeStatus = GPIO.input(self.gpioFailSafeStatus)
        self.hardWareOperationMode = GPIO.input(self.gpioHardWareOperationMode)

        self.pubFailSafeStatus.publish(self.failSafeStatus)
        self.pubHardWareOperationMode.publish(self.hardWareOperationMode)

        rospy.loginfo("Received software kill status %s", self.failSafeStatus )

    def writeSoftwareKillSwitch(self, message): 
        self.softWareKillSwitch = message.data
        GPIO.output(self.gpioSoftWareKillSwitch, self.softWareKillSwitch)

    def writeSoftwareOperationMode(self, message): 
        self.softWareOperationMode = message.data
        GPIO.output(self.gpioSoftWareOperationMode, self.softWareOperationMode)

def FailSafeNodeSetup():
    rospy.init_node('failsafe_interface_node', anonymous=True)

    failsafe = FailSafeInterface()

    while not rospy.is_shutdown():
        failsafe.readPins()

if __name__ == '__main__':
    try:
        FailSafeNodeSetup()
    except rospy.ROSInterruptException:
        pass
