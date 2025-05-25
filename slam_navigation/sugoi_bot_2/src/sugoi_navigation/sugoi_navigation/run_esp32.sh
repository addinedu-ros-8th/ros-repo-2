#!/bin/bash
source /opt/ros/jazzy/setup.bash
source ~/pinky_violet/install/local_setup.bash
source ~/venv/pinky/bin/activate
export ROS_DOMAIN_ID=23

# 중복 실행 방지
pgrep -f aruco_drive.py && pkill -f aruco_drive.py
pgrep -f esp32_full_sequence.py && pkill -f esp32_full_sequence.py

# 실행
python3 /home/pinky/dev_ws/sugoi/src/esp32_full_sequence.py \
  --case 3 \
  --ip 192.168.4.4 \
  --port 8888 \
  --script /home/pinky/dev_ws/aruco_drive.py
