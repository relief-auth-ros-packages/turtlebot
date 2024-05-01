#!/bin/bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://relief-nuc-0:11311
export ROBOT_TYPE=turtlebot

# Launch localisation
echo "Launching rfid localisation"

cd ~/relief-support-files/scripts/
./clear_logs.sh

roslaunch relief_rfid_detection localise_rfid_tags.launch &
