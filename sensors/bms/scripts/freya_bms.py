#!/usr/bin/env python3
import subprocess

def execute_jbdtool(usb_port):
    """
    This function executes the jbdtool command to query the BMS (Battery Management System)
    connected to the specified serial port. It then parses the output to extract 
    information about Voltage, Current, Temps, and Cells.

    :param usb_port: The USB port number to connect to (e.g., "USB0", "USB1", etc.)
    :type usb_port: str

    Output:
        - Prints the Voltage, Current, Temps, and Cells to the console.
        - If there's an error in executing the command, prints an error message.

    Example Output:
        Voltage: 24.010
        Current: 0.000
        Temps: 23.1,23.1,22.9
        Cells: 4.006,4.005,4.000,4.005,4.000,4.000
    """


    command = ["./jbdtool", "-t", f"serial:/dev/{usb_port}"]
    working_directory = "/home/vortex/bms/jbdtool"

    try:
        response = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True, cwd=working_directory)
        return response
        
    except subprocess.CalledProcessError as e:
        print("An error occurred while executing the command.")
        print("Error:", e.stderr.decode())

def parse_bms_data(response):
    output = response.stdout.decode()

    voltage = output.split("Voltage")[1].split("\n")[0].strip()
    current = output.split("Current")[1].split("\n")[0].strip()
    temps = output.split("Temps")[1].split("\n")[0].strip()
    cells = output.split("Cells")[1].split("\n")[0].strip()

    return voltage, current, temps, cells