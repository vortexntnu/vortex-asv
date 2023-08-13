#!/usr/bin/python3
import time
import smbus

from MCP342x import MCP342x
from std_msgs.msg import Float32

import RPi.GPIO as GPIO


# Pin number to control
FS_PIN = 33

# Threshold value current
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

upper_current_threshold = 100

channel_current = MCP342x(bus,
                        i2c_adress,
                        channel=1,
                        resolution=18)  # current

# Function to control the pin based on the current
def current_fs(value):
    global start_time
    if value < THRESHOLD:
        GPIO.output(FS_PIN, GPIO.HIGH)
        start_time = None
    else:
        if start_time is None:
            start_time = time.time()
        else:
            if time.time() - start_time >= DURATION:
                GPIO.output(FS_PIN, GPIO.LOW)
            else:
                GPIO.output(FS_PIN, GPIO.HIGH)

while True:

    
    current_fs(channel_current)