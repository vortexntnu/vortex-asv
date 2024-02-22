import rclpy
import subprocess
import time
import threading

class ROSMonitor:
    def __init__(self):

        # timestamp variable is a string that represents the current date and time in the format 'YYYY-MM-DD HH:MM:SS'
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S')     #I have to do it in the init function? or later?
        self.bag_file = timestamp
        # Get the list of topics
        self.topics = subprocess.run(['ros2', 'topic', 'list'], stdout=subprocess.PIPE, text=True).stdout.splitlines()
        print("Active topics when we start recording: ", self.topics)
        #self.new_topics = False



    def start_recording(self):

        print("thread1 running (record)")

        # Record bag data  -> then to see the data we just recorded we use the command "ros2 bag play <bag_file>"
        subprocess.Popen(['ros2', 'bag', 'record', '-o', self.bag_file] + self.topics, text=True)
        #subprocess.run(['ros2', 'bag', 'record', '-o', self.bag_file] + topics, stdout=subprocess.PIPE, text=True)
        #how to know the date, i have timestamp but i don't know how to use it with "ros 2 bag record"


    
    def check_topics(self):

        print("thread2 running (topic)")

        while True:     
            # Get the list of topics
            current_topics = subprocess.run(['ros2', 'topic', 'list'], stdout=subprocess.PIPE, text=True).stdout.splitlines()
            print(current_topics)

            if current_topics != self.topics:
                #self.new_topics = True
                self.topics = current_topics
                print("Updated topic list: ", self.topics) 
            #else:
                #self.new_topics = False            
    
            time.sleep(5)  # Adjust the sleep duration as needed



    def monitor_nodes_and_topics(self):
        
        # Create two threads to rcord and check if there is new topics at the same time
        record = threading.Thread(target=self.start_recording)
        check_if_new_topics = threading.Thread(target=self.check_topics)
        
        # Start the threads
        record.start()
        check_if_new_topics.start()


if __name__ == '__main__':
    monitor = ROSMonitor()
    monitor.monitor_nodes_and_topics()