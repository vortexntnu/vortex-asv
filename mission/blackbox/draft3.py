import rclpy
import subprocess
import time
import threading

class ROSMonitor:
    def __init__(self):
        # timestamp variable is a string that represents the current date and time in the format 'YYYY-MM-DD HH:MM:SS'
        timestamp = time.strftime('%Y-%m-%d_%H-%M-%S')
        self.bag_file = timestamp
        # Get the initial list of topics
        self.topics = subprocess.run(['ros2', 'topic', 'list'], stdout=subprocess.PIPE, text=True).stdout.splitlines()
        print("Active topics when we start recording: ", self.topics)
        # Variable to track whether recording is in progress
        self.recording_process = None
        self.recording_in_progress = False

    def check_topics(self):
        print("thread2 running (record)")

        while True:
            if self.recording_in_progress:
                # If recording is in progress, periodically check for changes in the list of topics
                current_topics = subprocess.run(['ros2', 'topic', 'list'], stdout=subprocess.PIPE, text=True).stdout.splitlines()

                if current_topics != self.topics:
                    # Stop the current recording process
                    self.stop_recording()

                    # Update the list of topics
                    self.topics = current_topics
                    print("Updated topic list: ", self.topics)

                    # Start a new recording process
                    self.start_recording()

            time.sleep(5)  # Adjust the sleep duration as needed

    def stop_recording(self):
        print("Stopping the current recording process")
        self.recording_in_progress = False  # Set flag to stop recording

        # Check if a recording process is running and terminate it
        if self.recording_process:
            self.recording_process.terminate()

    def start_recording(self):
        print("thread1 running: starting a new recording")

        # Record bag data
        self.recording_process = subprocess.Popen(['ros2', 'bag', 'record', '--all', self.bag_file])
        self.recording_in_progress = True  # Set flag to indicate recording is in progress

    def monitor_nodes_and_topics(self):
        # Create two threads to record and check if there are new topics at the same time
        record = threading.Thread(target=self.start_recording)
        check_if_new_topics = threading.Thread(target=self.check_topics)

        # Start the threads
        record.start()
        check_if_new_topics.start()

        # Wait for both threads to finish before exiting
        record.join()
        check_if_new_topics.join()

if __name__ == '__main__':
    monitor = ROSMonitor()
    monitor.monitor_nodes_and_topics()
