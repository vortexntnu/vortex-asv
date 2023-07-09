#!/usr/bin/python3
import time
import smbus

from MCP342x import MCP342x
from std_msgs.msg import Float32

import RPi.GPIO as GPIO


# Pin number to control
FS_PIN = 18

# Threshold value
THRESHOLD = 50

# Duration in seconds to wait before setting the pin LOW
DURATION = 1

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(FS_PIN, GPIO.OUT)




        # to read voltage and current from ADC on PDB through I2C
i2c_adress = 0x69

        # init of I2C bus communication
bus = smbus.SMBus(1)


psm_to_battery_voltage = 11.0  # V/V
psm_to_battery_current_scale_factor = 37.8788  # A/V
psm_to_battery_current_offset = 0.330  # V

upper_current_threshold = 12


# Function to control the pin based on the value
def current_fs(current):
    if current < THRESHOLD:
        GPIO.output(FS_PIN, GPIO.HIGH)
    else:
        GPIO.output(FS_PIN, GPIO.LOW)
        time.sleep(DURATION)
        GPIO.output(FS_PIN, GPIO.HIGH)

while True:
    channel_voltage = MCP342x(bus,
                            i2c_adress,
                            channel=0,
                            resolution=18)  # voltage
    channel_current = MCP342x(bus,
                            i2c_adress,
                            channel=1,
                            resolution=18)  # current
    
    current_fs(channel_current)