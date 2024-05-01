#!/bin/bash

# Launch robot
#screen -S TIFF_robot -d -m roslaunch relief_devel rb1_bringup_live.launch
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://RB1B0-180423AE:11311
export ROBOT_TYPE=rb1
export NUM_CAMERAS=3

./clear_logs.sh
sleep 1
roslaunch relief_devel rb1_bringup_live.launch &
sleep 5
roslaunch relief_devel avanti_live.launch &
sleep 2
roslaunch relief_rfid_antennas_poses_logger avanti_log.launch &
sleep 2
roslaunch relief_rfid_visualisation avanti_visualisation.launch &
