#!/bin/bash

# Variables ----------
# Define variables for launching service files
SERVICE_FILE_NAME_BMS="bms.service"
SERVICE_FILE_PATH_BMS="../sensors/bms/startup_scripts/"
SERVICE_FILE_NAME_TEMPERATURE="temperature.service"
SERVICE_FILE_PATH_TEMPERATURE="../sensors/temperature/startup_scripts/"
SERVICE_FILE_NAME_INTERNAL_STATUS="internal_status.service"
SERVICE_FILE_PATH_INTERNAL_STATUS="../mission/internal_status/startup_scripts/"
SERVICE_FILE_NAME_BLACKBOX="blackbox.service"
SERVICE_FILE_PATH_BLACKBOX="../mission/blackbox/startup_scripts/"

SYSTEMD_PATH="/etc/systemd/system/"



# Navigating ----------
echo "Navigated to the correct folder..."
# Get scripts directory and go there
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/"
cd $SCRIPT_DIR



# Setup ----------
echo "Setting up .service files..."
# Copy the .service files to current directory
cp $SERVICE_FILE_PATH_BMS$SERVICE_FILE_NAME_BMS $SERVICE_FILE_NAME_BMS # BMS
cp $SERVICE_FILE_PATH_TEMPERATURE$SERVICE_FILE_NAME_TEMPERATURE $SERVICE_FILE_NAME_TEMPERATURE # Temperature
cp $SERVICE_FILE_PATH_INTERNAL_STATUS$SERVICE_FILE_NAME_INTERNAL_STATUS $SERVICE_FILE_NAME_INTERNAL_STATUS # Internal Status
cp $SERVICE_FILE_PATH_BLACKBOX$SERVICE_FILE_NAME_BLACKBOX $SERVICE_FILE_NAME_BLACKBOX # Blackbox

# Replace placeholders in the .service files
# Note: This assumes <pathToThisFile> is to be replaced with the current directory
sed -i "s|<pathToThisFile>|$SCRIPT_DIR$SERVICE_FILE_PATH_BMS|g" $SERVICE_FILE_NAME_BMS # BMS
sed -i "s|<pathToThisFile>|$SCRIPT_DIR$SERVICE_FILE_PATH_TEMPERATURE|g" $SERVICE_FILE_NAME_TEMPERATURE # Temperature
sed -i "s|<pathToThisFile>|$SCRIPT_DIR$SERVICE_FILE_PATH_INTERNAL_STATUS|g" $SERVICE_FILE_NAME_INTERNAL_STATUS # Internal Status
sed -i "s|<pathToThisFile>|$SCRIPT_DIR$SERVICE_FILE_PATH_BLACKBOX|g" $SERVICE_FILE_NAME_BLACKBOX # Blackbox

# Kill the systems service files if it exists
sudo systemctl kill $SERVICE_FILE_NAME_BMS # BMS
sudo systemctl kill $SERVICE_FILE_NAME_TEMPERATURE # Temperature
sudo systemctl kill $SERVICE_FILE_NAME_INTERNAL_STATUS # Internal Status
sudo systemctl kill $SERVICE_FILE_NAME_BLACKBOX # Blackbox

# Copy the modified .service files to the systemd directory
# Note: Need sudo permission to copy to /etc/systemd/system
sudo cp $SERVICE_FILE_NAME_BMS $SYSTEMD_PATH # BMS
sudo cp $SERVICE_FILE_NAME_TEMPERATURE $SYSTEMD_PATH # Temperature
sudo cp $SERVICE_FILE_NAME_INTERNAL_STATUS $SYSTEMD_PATH # Internal Status
sudo cp $SERVICE_FILE_NAME_BLACKBOX $SYSTEMD_PATH # Blackbox

# Delete the redundant copy
rm $SERVICE_FILE_NAME_BMS # BMS
rm $SERVICE_FILE_NAME_TEMPERATURE # Temperature
rm $SERVICE_FILE_NAME_INTERNAL_STATUS # Internal Status
rm $SERVICE_FILE_NAME_BLACKBOX # Blackbox

# Change permision of the .service files
sudo chmod 777 $SYSTEMD_PATH$SERVICE_FILE_NAME_BMS # BMS
sudo chmod 777 $SYSTEMD_PATH$SERVICE_FILE_NAME_TEMPERATURE # Temperature
sudo chmod 777 $SYSTEMD_PATH$SERVICE_FILE_NAME_INTERNAL_STATUS # Internal Status
sudo chmod 777 $SYSTEMD_PATH$SERVICE_FILE_NAME_BLACKBOX # Blackbox



# Launching ----------
echo "Launching .service files..."
# Reload systemd to recognize the new services
sudo systemctl daemon-reload

# Enable new services to start on boot
sudo systemctl enable $SERVICE_FILE_NAME_BMS # BMS
sudo systemctl enable $SERVICE_FILE_NAME_TEMPERATURE # Temperature
sudo systemctl enable $SERVICE_FILE_NAME_INTERNAL_STATUS # Internal Status
sudo systemctl enable $SERVICE_FILE_NAME_BLACKBOX # Blackbox

# Start the services immediately
sudo systemctl start $SERVICE_FILE_NAME_BMS # BMS
sudo systemctl start $SERVICE_FILE_NAME_TEMPERATURE # Temperature
sudo systemctl start $SERVICE_FILE_NAME_INTERNAL_STATUS # Internal Status
sudo systemctl start $SERVICE_FILE_NAME_BLACKBOX # Blackbox



# Debugging ----------
echo "'$SERVICE_FILE_NAME_BMS' - installed and started successfully :)"
echo "'$SERVICE_FILE_NAME_TEMPERATURE' - installed and started successfully :)"
echo "'$SERVICE_FILE_NAME_INTERNAL_STATUS' - installed and started successfully :)"
echo "'$SERVICE_FILE_NAME_BLACKBOX' - installed and started successfully :)"