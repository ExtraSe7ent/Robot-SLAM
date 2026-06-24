#!/bin/bash
# start_nav.sh — Khởi động chế độ NAVIGATION (dùng map đã lưu)
# Cách dùng: ./start_nav.sh <tên_map>
# Ví dụ:     ./start_nav.sh tang1
# Map phải được lưu trước bằng nút "Serialize map" trên dashboard

MAP_NAME="${1:-}"

if [ -z "$MAP_NAME" ]; then
    echo "Lỗi: cần truyền tên map"
    echo "Cách dùng: ./start_nav.sh <tên_map>"
    echo "Ví dụ:     ./start_nav.sh tang1"
    exit 1
fi

MAP_FILE="$HOME/ros2_ws/maps/${MAP_NAME}.data"
if [ ! -f "$MAP_FILE" ]; then
    echo "Lỗi: không tìm thấy map '$MAP_FILE'"
    echo "Hãy lưu map trước bằng nút 'Serialize map' trên dashboard."
    exit 1
fi

echo "=== Robot An Ninh — NAVIGATION MODE ==="
echo "Sử dụng map: $MAP_NAME"

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
echo "Launching mac_brain (navigation with map=$MAP_NAME)..."

ros2 launch mac_brain mac_nav.launch.py map_name:="$MAP_NAME" &
LAUNCH_PID=$!

# Chờ slam_toolbox sẵn sàng để load map
echo "Chờ slam_toolbox khởi động..."
sleep 10

MAX_WAIT=60; WAITED=0
until ros2 service list 2>/dev/null | grep -q "slam_toolbox/deserialize_map"; do
    sleep 2; WAITED=$((WAITED+2))
    if [ $WAITED -ge $MAX_WAIT ]; then
        echo "Lỗi: slam_toolbox không sẵn sàng sau ${MAX_WAIT}s"
        kill $LAUNCH_PID 2>/dev/null
        exit 1
    fi
done

echo "Đang load map: $MAP_NAME..."
ros2 service call /slam_toolbox/deserialize_map \
    slam_toolbox/srv/DeserializePoseGraph \
    "{filename: '$HOME/ros2_ws/maps/$MAP_NAME', match_type: 1, initial_pose: {x: 0, y: 0, theta: 0}}"

echo "=== Navigation mode đã sẵn sàng ==="
wait $LAUNCH_PID
