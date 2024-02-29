#!/bin/bash
# Determine the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Source ROS 2 setup
source /opt/ros/humble/setup.bash

# Export ROS environment variables
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# Execute the Python script, using the script's directory to find it
exec python3 "$SCRIPT_DIR/record_data.py"