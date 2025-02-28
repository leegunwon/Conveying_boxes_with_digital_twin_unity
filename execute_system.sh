#!/bin/bash
cd ~/serial_ws
colcon build
source install/setup.bash

# 첫 번째 인자 (빈 문자열 대신 기본값을 설정)
EXECUTE_CONTROL_NODE=${1:-"False"}

echo "Executing control node: $EXECUTE_CONTROL_NODE"

# EXECUTE_CONTROL_NODE가 'True'이면 실행
if [ "$EXECUTE_CONTROL_NODE" == "True" ]; then
    ros2 launch robot_control conveyer_system_on.launch.py
elif [ "$EXECUTE_CONTROL_NODE" == "False" ]; then
    ros2 launch robot_control system_without_manager.launch.py
fi
