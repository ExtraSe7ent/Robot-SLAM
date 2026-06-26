#!/bin/bash
echo "=========================================="
echo "🧠 STARTING ROBOT BRAIN (MAC VM) "
echo "=========================================="

# 1. Cleanup old ROS processes
echo "[*] Cleaning up old ROS processes..."
pkill -f "ros2 launch" 2>/dev/null
pkill -f "slam_toolbox" 2>/dev/null
pkill -f "rf2o_laser" 2>/dev/null
pkill -f "ekf_node" 2>/dev/null
pkill -f "rosbridge" 2>/dev/null
fuser -k 9090/tcp 2>/dev/null
sleep 2

# 2. Source ROS 2 environment
echo "[*] Sourcing ROS 2 Workspace..."
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 3. Configure DDS Network
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/vmuser/cyclone_dds.xml
export GEMINI_API_KEY="AQ.Ab8RN6IlncQq-dja0ztv7swxwWIjD2j0yzxW6QjVoATPIEIHRg"

# 4. Start the Brain Launch File
echo "[*] Launching rf2o, EKF, SLAM Toolbox, and Rosbridge..."
ros2 launch mac_brain mac_brain.launch.py
