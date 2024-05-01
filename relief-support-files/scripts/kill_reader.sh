#!/bin/bash

## C++ execution (deprecated)
# Obtain reader process id
#reader_pid=$(pgrep speedwayr)

# Kill it
#kill -9 $reader_pid &

cd ~/catkin_ws/src/relief-rfid-detection/application_java_001625127C5A
echo 1 > control.txt
