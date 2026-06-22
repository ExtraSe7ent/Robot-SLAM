#!/bin/bash
echo "=========================================="
echo "   STARTING ROBOT BRAIN (MAC VM) "
echo "=========================================="
# 1. Cleanup old ROS processes just in case
echo "[*] Cleaning up old rosbridge processes..."
fuser -k 9090/tcp 2>/dev/null
sleep 1
# 2. Source ROS 2 environment
echo "[*] Sourcing ROS 2 Workspace..."
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
# 3. Configure DDS Network
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/vmuser/cyclone_dds.xml
# 4. Start the Brain Launch File
echo "[*] Launching rf2o, EKF, SLAM Toolbox, and Rosbridge..."
ros2 launch mac_brain mac_brain.launch.py
