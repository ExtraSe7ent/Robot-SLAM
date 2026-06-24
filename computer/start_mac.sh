#!/bin/bash
# start_mac.sh — Khởi động chế độ MAPPING (lập bản đồ mới)
# Chạy trên máy tính chính (Ubuntu VM)
# Yêu cầu: Pi đã chạy start_pi.sh và đang publish /scan_filtered, /imu/data

echo "=== Robot An Ninh — MAPPING MODE ==="

# Kill tiến trình cũ
pkill -f "ros2 launch"    2>/dev/null
pkill -f "slam_toolbox"   2>/dev/null
pkill -f "rf2o"           2>/dev/null
pkill -f "rosbridge"      2>/dev/null
pkill -f "ekf_node"       2>/dev/null
fuser -k 9090/tcp         2>/dev/null
sleep 2

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/vmuser/cyclone_dds.xml

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "Launching mac_brain (mapping)..."

ros2 launch mac_brain mac_brain.launch.py
