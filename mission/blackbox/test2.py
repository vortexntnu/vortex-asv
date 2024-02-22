
import rclpy
import subprocess
import time
import os
from datetime import datetime, timedelta
import shutil

class ROSMonitor:
    def __init__(self):
       print("Initialisation faite youhou!!!!!")


    def delete_old_bags(self, days=1):
        print('Entering the delete_old_bags function')
        current_time = datetime.now()
        older_than_time = current_time - timedelta(days=days)

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
        

if __name__ == '__main__':
    monitor = ROSMonitor()
    monitor.delete_old_bags()




















# print('on rentre dans la fonction delete_old_bag')
#         current_time = datetime.now()
#         older_than_time = current_time - timedelta(days=days)

#         # List all files in the current directory
#         files_in_directory = os.listdir('.')
#         print('liste des files_in_directory faite normalement')
#         print(files_in_directory)
        
#         for file in files_in_directory:
#             #if file.endswith('.bag') and file.startswith('rosdata'):
#                 print('on rentre dans le if')
#                 # Assuming bag files are in the format 'rosdata_YYYY-MM-DD_HH:MM:SS.bag'
#                 try:
#                     file_time = datetime.strptime(file[8:27], '%Y-%m-%d_%H:%M:%S')     ###################to change !!!!!!!
#                     #file_time = datetime.strptime(file, '%Y-%m-%d_%H:%M:%S')
#                     print('FILE TIME')
#                     print(file_time)
#                 except ValueError:
#                     print('erreur pas le bon format')
#                     # Skip files that do not match the expected format
#                     continue
                
#                 if file_time < older_than_time:
#                     os.remove(file)
#                     print(f"Deleted old bag file: {file}")