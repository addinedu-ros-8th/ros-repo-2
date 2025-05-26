#!/bin/bash
# run_aruco_down.sh

export ROS_DOMAIN_ID=23
source /opt/ros/jazzy/setup.bash
source ~/pinky_violet/install/local_setup.bash
source ~/venv/pinky/bin/activate

echo "Running aruco_drive_down.py..."
python3 /home/pinky/dev_ws/sugoi/src/aruco_drive_down.py
