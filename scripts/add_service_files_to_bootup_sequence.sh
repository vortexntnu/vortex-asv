#!/bin/bash

# Variables ----------
# Define variables for launching service files
SERVICE_FILE_NAME_BMS="bms.service"
SERVICE_FILE_PATH_BMS="../sensors/bms/startup_scripts/"
SERVICE_FILE_NAME_TEMPERATURE="temperature.service"
SERVICE_FILE_PATH_TEMPERATURE="../sensors/temperature/startup_scripts/"

SYSTEMD_PATH="/etc/systemd/system/"



# Navigating ----------
echo "Navigated to the correct folder..."
# Get scripts directory and go there
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/"
cd $SCRIPT_DIR



# Setup ----------
echo "Seting up .service files..."
# Copy the .service files to current directory
cp $SERVICE_FILE_PATH_BMS$SERVICE_FILE_NAME_BMS $SERVICE_FILE_NAME_BMS # BMS
cp $SERVICE_FILE_PATH_TEMPERATURE$SERVICE_FILE_NAME_TEMPERATURE $SERVICE_FILE_NAME_TEMPERATURE # Temperature

# Replace placeholders in the .service files
# Note: This assumes <pathToThisFile> is to be replaced with the current directory
sed -i "s|<pathToThisFile>|$SCRIPT_DIR$SERVICE_FILE_PATH_BMS|g" $SERVICE_FILE_NAME_BMS # BMS
sed -i "s|<pathToThisFile>|$SCRIPT_DIR$SERVICE_FILE_PATH_TEMPERATURE|g" $SERVICE_FILE_NAME_TEMPERATURE # Temperature

# Kill the systems service files if it exists
sudo systemctl kill $SERVICE_FILE_NAME_BMS # BMS
sudo systemctl kill $SERVICE_FILE_NAME_TEMPERATURE # Temperature

# Copy the modified .service files to the systemd directory
# Note: Need sudo permission to copy to /etc/systemd/system
sudo cp $SERVICE_FILE_NAME_BMS $SYSTEMD_PATH # BMS
sudo cp $SERVICE_FILE_NAME_TEMPERATURE $SYSTEMD_PATH # Temperature

# Delete the redundant copy
rm $SERVICE_FILE_NAME_BMS # BMS
rm $SERVICE_FILE_NAME_TEMPERATURE # Temperature

# Change permision of the .service files
sudo chmod 777 $SYSTEMD_PATH$SERVICE_FILE_NAME_BMS # BMS
sudo chmod 777 $SYSTEMD_PATH$SERVICE_FILE_NAME_TEMPERATURE # Temperature



# Launching ----------
echo "Launching .service files..."
# Reload systemd to recognize the new services
sudo systemctl daemon-reload

# Enable new services to start on boot
sudo systemctl enable $SERVICE_FILE_NAME_BMS # BMS
sudo systemctl enable $SERVICE_FILE_NAME_TEMPERATURE # Temperature

# Start the services immediately
sudo systemctl start $SERVICE_FILE_NAME_BMS # BMS
sudo systemctl start $SERVICE_FILE_NAME_TEMPERATURE # Temperature



# Debugging ----------
echo "'$SERVICE_FILE_NAME_BMS' has been installed and started successfully :)"
echo "'$SERVICE_FILE_NAME_TEMPERATURE' has been installed and started successfully :)"