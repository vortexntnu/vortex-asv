#!/usr/bin/env python3

import rospy
import csv
import numpy as np
from pathlib import Path
from datetime import datetime
from threading import Lock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState


class DataLoggerNode:

    def __init__(self):
        rospy.init_node('logger_node', anonymous=True)

        # Create subscribers for the desired topics
        self.odom_sub = rospy.Subscriber('/estimator/pose', Odometry,
                                         self.odom_callback)
        self.battery_sub = rospy.Subscriber('/internal/status/bms',
                                            BatteryState,
                                            self.battery_callback)

        self.log_array = []  # Array to store logged data
        self.log_lock = Lock()  # Lock for thread safety

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_file_name = f"data_log_{timestamp}.csv"
        script_directory = Path(__file__).resolve().parent
        csv_file_path = script_directory.parent / 'logs' / csv_file_name

        self.csv_file = open(csv_file_path, 'w')
        self.csv_writer = csv.writer(self.csv_file, delimiter=',')

        self.csv_writer.writerow([
            'latitude', 'longitude', 'altitude', 'heading', 'speed',
            'power_consumption', 'battery%', 'timestep'
        ])

        self.log_rate = rospy.Rate(1)

        self.current_odom = np.zeros(5)
        self.current_battery = np.zeros(2)

        self.prev_timestamp = rospy.get_time()

        self.max_voltage = 24.6
        self.min_voltage = 19.0

        self.log_threshold = 100  # Number of entries to accumulate before writing to file

    def quaternion_to_yaw(self, q):
        x = q.x
        y = q.y
        z = q.z
        w = q.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return yaw

    def find_battery_percentage_left(self, voltage):
        return round((voltage - self.min_voltage) /
                     (self.max_voltage - self.min_voltage) * 100, 2)

    def odom_callback(self, data):
        self.log_lock.acquire()
        self.current_odom = [
            data.pose.pose.position.x, data.pose.pose.position.y,
            data.pose.pose.position.z,
            self.quaternion_to_yaw(data.pose.pose.orientation),
            np.sqrt(data.twist.twist.linear.x**2 +
                    data.twist.twist.linear.y**2)
        ]
        self.log_lock.release()

    def battery_callback(self, data):
        self.log_lock.acquire()
        self.current_battery = [
            data.voltage * data.current,
            self.find_battery_percentage_left(data.voltage)
        ]
        self.log_lock.release()

    def timer_callback(self):
        self.log_data()

    def log_data(self):
        now = rospy.get_time()
        timestep = now - self.prev_timestamp
        self.prev_timestamp = now

        self.log_lock.acquire()
        data = [self.current_odom, self.current_battery, timestep]
        self.log_lock.release()

        with self.log_lock:
            self.log_array.append(data)

        rospy.loginfo("Logged data from %s")

        if len(self.log_array) >= self.log_threshold:
            self.write_to_file()

    def write_to_file(self):
        with self.log_lock:
            for entry in self.log_array:
                self.csv_writer.writerow(entry)
            self.log_array = []  # Clear the log array

    def run(self):
        while not rospy.is_shutdown():
            self.log_data()
            self.log_rate.sleep()

        self.csv_file.close()

    def shutdown(self):
        self.write_to_file()  # Write any remaining data before shutting down
        self.csv_file.close()


if __name__ == '__main__':
    logger = DataLoggerNode()
    rospy.on_shutdown(logger.shutdown)
    logger.run()
