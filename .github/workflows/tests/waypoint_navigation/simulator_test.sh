#!/bin/bash
set -e

# Load ROS 2 environment
echo "Setting up ROS 2 environment..."
. /opt/ros/humble/setup.sh
. ~/ros2_ws/install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

# Get the directory of this script dynamically
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

setsid ros2 launch stonefish_sim simulation_nogpu.launch.py task:=freya_no_gpu &
SIM_PID=$!
echo "Launced simulator with PID: $SIM_PID"

echo "Waiting for simulator to start..."
timeout 30s bash -c 'until ros2 topic list | grep -q "/seapath/odom/ned"; do sleep 1; done'
echo "Simulator started"

echo "Waiting for odom data..."
timeout 10s ros2 topic echo /seapath/odom/ned --once
echo "Got odom data"

setsid ros2 launch thrust_allocator_asv thrust_allocator_asv.launch.py &
THRUST_PID=$!
echo "Launced thrust allocator with PID: $THRUST_PID"

setsid ros2 launch hybridpath_controller hybridpath_controller.launch.py &
CONTROLLER_PID=$!
echo "Launced controller with PID: $CONTROLLER_PID"

setsid ros2 launch hybridpath_guidance hybridpath_guidance.launch.py &
GUIDANCE_PID=$!
echo "Launced guidance with PID: $GUIDANCE_PID"

echo "Turning off killswitch and setting operation mode to autonomous mode"
ros2 topic pub /freya/killswitch std_msgs/msg/Bool "{data: false}" -1
ros2 topic pub /freya/operation_mode std_msgs/msg/String "{data: 'autonomous mode'}" -1

echo "Sending goal"
python3 "$SCRIPT_DIR/freya_send_goal.py"

echo "Checking if goal reached"
python3 "$SCRIPT_DIR/freya_check_goal.py"

kill -TERM -"$SIM_PID" -"$CONTROLLER_PID" -"$GUIDANCE_PID" -"$THRUST_PID"