#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import BatteryState
from freya_bms import execute_jbdtool, parse_bms_data

def main():
    # Initialize the node with the name 'freya_bms'
    rospy.init_node('freya_bms')

    # Create a Publisher object to publish messages to the 'battery_info' topic
    battery_pub = rospy.Publisher('/internal/status/bms', BatteryState, queue_size=10)

    # List of USB devices to loop through
    usb_ports = ["ttyUSB1", "ttyUSB2"] # Add as many as you need

    # Rate object to control the loop rate (in Hz)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        for usb_port in usb_ports:
            response = execute_jbdtool(usb_port)
            if response.stdout != b'': # Check if data is received
                
                voltage, current, temps, cells = parse_bms_data(response)

                battery_msg = BatteryState()
                battery_msg.voltage = float(voltage)   # Assuming voltage is in Volts
                battery_msg.current = float(current)   # Assuming current is in Amperes
                battery_msg.temperature = float(temps.split(',')[0]) # Assuming temperature is in degrees Celsius
                battery_msg.location = usb_port

                # Publish the Battery message to the 'battery_info' topic
                battery_pub.publish(battery_msg)

        # Sleep until the next iteration
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
