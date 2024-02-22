import rclpy
import subprocess
import time
import os
from datetime import datetime, timedelta

class ROSMonitor:
    def __init__(self):
        # timestamp variable is a string that represents the current date and time in the format 'YYYY-MM-DD HH:MM:SS'
        timestamp = time.strftime('%Y-%m-%d_%H:%M:%S')    
        self.bag_file = 'rosdata_' + timestamp + '.bag'
        self.topics = [                                                  #indicate there all the topics we want to record
                    '/turtle1/cmd_vel',
                    '/turtle1/color_sensor',
                    '/turtle1/pose'
                    ]

    def monitor_nodes_and_topics(self):

        # Record bag data  -> then to see the data we just recorded we use the command "ros2 bag play <bag_file>"
        subprocess.run(['ros2', 'bag', 'record', '-o', self.bag_file] + self.topics, text=True)
        #the name of the bagfile is the date and hour in the format 'rosdata_YYYY-MM-DD HH:MM:SS.bag'

if __name__ == '__main__':
    monitor = ROSMonitor()
    monitor.monitor_nodes_and_topics()
