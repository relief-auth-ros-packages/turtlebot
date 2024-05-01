#!/bin/bash
export ROBOT_TYPE=turtlebot

# Clear logs from previous experiments
#bash ~/relief-support-files/scripts/clear_logs.sh &

# Launch readers
echo "Launching reader applications"

cd ~/catkin_ws/src/relief-rfid-detection/application_java_001625127C5A
echo 0 > control.txt
java -jar 001625127C5A.jar &