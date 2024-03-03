# Python Libraries
import os
import time
import csv
from datetime import datetime, timedelta



class BlackBoxLogData:
    def __init__(self,
        ROS2_PACKAGE_DIRECTORY = ""     
    ):
        # Global variables for .csv file manipulation ----------
        # Get the path for the directory where we will store our data
        self.blackbox_data_directory = ROS2_PACKAGE_DIRECTORY + "blackbox_data/"
        
        timestamp = time.strftime('%Y-%m-%d_%H:%M:%S')
        data_file_name = 'blackbox_data_' + timestamp + '.csv'
        self.data_file_location = self.blackbox_data_directory + data_file_name

        self.csv_headers = [
            "Time",
            "Power Sense Module Current",
            "Power Sense Module Voltage",
        ]

        # Manage csv files for blackbox data ----------
        # If there are stale old .csv files => Delete oldes ones
        # If .csv files take up to much space => Delte oldest ones
        self.manage_csv_files()

        # Make new .csv file for loging blackbox data ----------
        with open(self.data_file_location, mode="w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(self.csv_headers)
    
    # Methods for inside use of the class ----------
    def manage_csv_files(self, max_file_age_in_days=1, max_size_kb=3_000):         #adjust the max size before you start deleting old files (1 000 000 kb = 1 000 mb = 1 gb)
        current_time = datetime.now()
        older_than_time = current_time - timedelta(days=max_file_age_in_days)

        # List all .csv files in the blackbox data directory
        csv_files = [f for f in os.listdir(self.blackbox_data_directory) if f.endswith('.csv') and f.startswith('blackbox_data_')]
        
        # Delete .csv files older than 1 day
        for csv_file in csv_files:
            try:
                file_time = datetime.strptime(csv_file[13:-4], '%Y-%m-%d_%H:%M:%S')
            except ValueError:
                print(f'Invalid filename format, skipping file: {csv_file}')
                continue
            
            if file_time < older_than_time:
                file_path = os.path.join(self.blackbox_data_directory, csv_file)
                os.remove(file_path)
                print(f'Deleted old csv file: {file_path}')

        # Calculate the total size of remaining .csv files
        total_size_kb = sum(os.path.getsize(os.path.join(self.blackbox_data_directory, f)) for f in os.listdir(self.blackbox_data_directory) if f.endswith('.csv')) / 1024
        
        # Delete oldest files if total size exceeds max_size_kb
        while total_size_kb > max_size_kb:
            # Sort .csv files by timestamp (oldest first)
            csv_files_sorted = sorted(csv_files, key=lambda x: datetime.strptime(x[13:-4], '%Y-%m-%d_%H:%M:%S'))
            
            if not csv_files_sorted:
                print('No .csv files to delete.')
                break
            
            oldest_file = csv_files_sorted[0]
            oldest_file_path = os.path.join(self.blackbox_data_directory, oldest_file)
            os.remove(oldest_file_path)
            print(f'Deleted the oldest csv file: {oldest_file_path}')
            
            # Recalculate the total size of remaining .csv files
            total_size_kb = sum(os.path.getsize(os.path.join(self.blackbox_data_directory, f)) for f in os.listdir(self.blackbox_data_directory) if f.endswith('.csv')) / 1024
            print(f'Now the total size of .csv files is: {total_size_kb:.2f} KB')

    # Methods for external uses ----------
    def log_data_to_csv_file(self,
        psm_current = 0.0,
        psm_voltage = 0.0,
    ):
        # Get current time in hours, minutes, seconds and miliseconds
        current_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]

        # Write to .csv file
        with open(self.data_file_location, mode="a", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow([
                current_time,
                psm_current,
                psm_voltage,
            ])
