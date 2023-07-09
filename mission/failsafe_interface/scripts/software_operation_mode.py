import rospy
import RPi.GPIO as GPIO

SF_OP_MOD_PIN = 20


class FailSafeInterface(object):
    def __init__(
        self,
    ):  
        #Repeat process for the other pins
        self.gpioSoftWareOperationMode = rospy.get_param(
            "/failsafe/gpio/gpioSoftWareOperationMode"
        )
        GPIO.setmode(self.gpioSoftWareOperationMode)
        
        topicSoftWareOperationMode = rospy.get_param(
            "/failsafe/subscribers/softWareOperationMode"
        )

        #Create subscriber for the software operation mode
        self.subSoftWareOperationMode = rospy.Subscriber(
            topicSoftWareOperationMode, String, self.writeSoftwareOperationMode
        )

    # Output software operation mode through pin
    def writeSoftwareOperationMode(self, message):
        self.softWareOperationMode = message.data
        GPIO.write(self.gpioSoftWareOperationMode, self.softWareOperationMode)


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
