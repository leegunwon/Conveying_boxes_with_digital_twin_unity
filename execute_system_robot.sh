#!/bin/bash
colcon build
source install/setup.bash

ros2 launch aruco_and_yolo_detection aruco_yolo.launch.py
