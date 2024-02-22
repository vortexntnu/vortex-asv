import rclpy
import subprocess
import time
import os
from datetime import datetime, timedelta
import shutil

class ROSMonitor:
    def __init__(self):
       print("Initialisation faite!")


    def delete_old_bags(self, days=1, max_size_kb=300):         #adjust the max size before you start deleting old files (1 000 000 kb = 1 000 mb = 1 gb)
        print('Entering the delete_old_bags function')
        current_time = datetime.now()
        older_than_time = current_time - timedelta(days=days)


        print("-------------FIRST WE DELETE ALL THE BAGFILES OLDER THAN 24H-------------------")   


        # List all directories in the current directory
        directories_in_current_directory = [d for d in os.listdir('.') if os.path.isdir(d)]
        print('List of directories in the current directory:')
        print(directories_in_current_directory)
        
        for directory_name in directories_in_current_directory:
            if directory_name.endswith('.bag') and directory_name.startswith('rosdata'):
                print('Entering the if condition')
                # Assuming the directory name is in the format 'rosdata_YYYY-MM-DD_HH:MM:SS.bag'
                try:
                    directory_time = datetime.strptime(directory_name[8:27], '%Y-%m-%d_%H:%M:%S')
                    print('DIRECTORY TIME')
                    print(directory_time)
                except ValueError:
                    print('Invalid format, skipping directory')
                    # Skip directories that do not match the expected format
                    continue
                
                if directory_time < older_than_time:
                    directory_path = os.path.join('.', directory_name)
                    shutil.rmtree(directory_path)
                    print(f"Deleted old directory: {directory_path}")

        
        print("-------------ALL BAGFILES OLDER THAN 24H DELETED------------------")

        print("")

        print("----------------NOW WE DELETE THE OLDER BAGFILES IF THE SIZE EXCEED THE MAX SIZE------------------")


        # Update the list of directories after deleting old directories
        directories_in_current_directory = [d for d in os.listdir('.') if os.path.isdir(d)]

        # Calculate total size of remaining directories ON
        total_size_kb = sum(self.get_directory_size(d) for d in directories_in_current_directory if (d.startswith('rosdata_') and d.endswith('.bag')) ) / 1024
        print(f"Total size of directories: {total_size_kb:.2f} KB")

        # Delete additional directories if total size exceeds the specified limit
        while total_size_kb > max_size_kb:
            # Sort directories by timestamp (oldest first) ONLY BAGFILES
            sorted_directories = sorted(
                [d for d in directories_in_current_directory if (d.startswith('rosdata_') and d.endswith('.bag'))],
                key=lambda d: datetime.strptime(d[8:27], '%Y-%m-%d_%H:%M:%S')
            )
            
            if not sorted_directories:
                print("No directories to delete.")
                break

            oldest_directory = sorted_directories[0]
            oldest_directory_path = os.path.join('.', oldest_directory)

            print(f"Deleting the oldest directory: {oldest_directory_path}")
            shutil.rmtree(oldest_directory_path)

            # Update the list of directories and total size
            directories_in_current_directory = [d for d in os.listdir('.') if os.path.isdir(d)]
            total_size_kb = sum(self.get_directory_size(d) for d in directories_in_current_directory if (d.startswith('rosdata_') and d.endswith('.bag')) ) / 1024
            print(f'Now the total size of directories is: {total_size_kb:.2f} KB')

    def get_directory_size(self, directory):
        directory_path = os.path.join('.', directory)
        total_size = sum(os.path.getsize(os.path.join(directory_path, file)) for file in os.listdir(directory_path) if os.path.isfile(os.path.join(directory_path, file)))
        return total_size
        

if __name__ == '__main__':
    monitor = ROSMonitor()
    monitor.delete_old_bags()










