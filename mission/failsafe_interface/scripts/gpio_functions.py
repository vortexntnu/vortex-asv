#!/usr/bin/env python
# Check this link to understand what's going on https://wiki.radxa.com/Rockpi4/hardware/gpio
# In short, to work with the GPIO pins we need to work with files.
# We go to /sys/class/gpio/ and we get a folder (gpio133 for example) for each initalized pin


import os
import time

import subprocess

#We store the password in a file locally on the rock pi
passwordFile = open("/home/rock/Documents/password.txt", "r")

sudoPassword = passwordFile.readline().strip()

passwordFile.close()


def init_pin(pin):
    try:
        command = f'sudo sh -c "echo {pin} > /sys/class/gpio/export"'
        p = os.system("echo %s|sudo -S %s" % (sudoPassword, command))
    except:
        pass


def set_pin_as_output(pin):
    try:
        # With the /direction file we set either to input or output, in this case output with out
        command = f'sudo sh -c "echo out > /sys/class/gpio/gpio{pin}/direction"'
        p = os.system("echo %s|sudo -S %s" % (sudoPassword, command))
    except:
        pass


def pin_write(pin, value):
    try:
        # With the /value file we set the output
        sudoPassword = ""
        command = f'sudo sh -c "echo {value} > /sys/class/gpio/gpio{pin}/value"'
        p = os.system("echo %s|sudo -S %s" % (sudoPassword, command))
    except:
        pass


def set_pin_as_input(pin):
    try:
        # With the /direction file we set either to input or input, in this case output with in
        command = f'sudo sh -c "echo in > /sys/class/gpio/gpio{pin}/direction"'
        p = os.system("echo %s|sudo -S %s" % (sudoPassword, command))
    except:
        pass


def pin_read(pin):
    try:
        # With the /value file we read the input
        command = f'sudo sh -c "cat /sys/class/gpio/gpio{pin}/value"'
        output_str = os.popen("echo %s|sudo -S %s" % (sudoPassword, command)).read()
        output_int = int(output_str)
        return output_int

    except:
        return -1
