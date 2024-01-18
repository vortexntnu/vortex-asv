import rclpy
import subprocess
import time

class ROSMonitor:
    def __init__(self, bag_file='ros_data.bag'):
        self.bag_file = bag_file

    def monitor_nodes_and_topics(self):
        while True:
            # timestamp variable is a string that represents the current date and time in the format 'YYYY-MM-DD HH:MM:SS'
            timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
            
            # Get the list of nodes and topics
            nodes = subprocess.run(['ros2', 'node', 'list'], stdout=subprocess.PIPE, text=True).stdout.splitlines()
            topics = subprocess.run(['ros2', 'topic', 'list'], stdout=subprocess.PIPE, text=True).stdout.splitlines()

            # Record bag data  -> then to see the data we just recorded we use the command "ros2 bag play <bag_file>"
            subprocess.run(['ros2', 'bag', 'record', '-o', self.bag_file] + topics, stdout=subprocess.PIPE, text=True)
            #how to know the date, i have timestamp but i don't know how to use it with "ros 2 bag record"

            time.sleep(5)  # Adjust the sleep duration as needed

if __name__ == '__main__':
    monitor = ROSMonitor()
    monitor.monitor_nodes_and_topics()