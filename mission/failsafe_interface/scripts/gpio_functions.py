#!/usr/bin/env python
#Check this link to understand what's going on https://wiki.radxa.com/Rockpi4/hardware/gpio

import os
import time

import subprocess

hardware_trigger = 133 # pin 35
software_trigger = 76 # pin 33
software_mode = 73 # pin 31
hardware_mode = 74 # pin 29



sudoPassword = "rock"


def init_pin(pin):
    try:
        command = f'sudo sh -c "echo {pin} > /sys/class/gpio/export"' 
        p = os.system('echo %s|sudo -S %s' % (sudoPassword, command))
        print("successfully set up pin")
    except:
        pass

def set_pin_as_output(pin):
    try:
        command = f'sudo sh -c "echo out > /sys/class/gpio/gpio{pin}/direction"' 
        p = os.system('echo %s|sudo -S %s' % (sudoPassword, command))
    except:
        pass

def pin_write(pin, value):
    try:
        sudoPassword = 'rock' 
        command = f'sudo sh -c "echo {value} > /sys/class/gpio/gpio{pin}/value"' 
        p = os.system('echo %s|sudo -S %s' % (sudoPassword, command))
    except:
        pass

def set_pin_as_input(pin):
    try:
        command = f'sudo sh -c "echo in > /sys/class/gpio/gpio{pin}/direction"' 
        p = os.system('echo %s|sudo -S %s' % (sudoPassword, command))
    except:
        pass

def pin_read(pin):
    try:
        command = f'sudo sh -c "cat /sys/class/gpio/gpio{pin}/value"' 

        output_str = os.popen('echo %s|sudo -S %s' % (sudoPassword, command)).read()
        output_int = int(output_str)
        return output_int

    except:
        return -1
