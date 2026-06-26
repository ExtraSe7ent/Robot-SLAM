#!/bin/bash
MAP_NAME=${1:-"my_map"}
echo "=========================================="
echo "🗺 NAV MODE — Map: $MAP_NAME"
echo "=========================================="

# Check if map file exists
if [ ! -f "$HOME/ros2_ws/maps/${MAP_NAME}.data" ]; then
    echo "❌ Not found: ~/ros2_ws/maps/${MAP_NAME}.data"
    echo ""
    echo " Available maps:"
    ls ~/ros2_ws/maps/*.data 2>/dev/null \
        | xargs -I{} basename {} .data \
        || echo " (no maps found — please run start_mac.sh to scan a map first)"
    exit 1
fi

# Kill ALL previous ROS session
echo "[*] Killing previous ROS session..."
pkill -f "ros2 launch" 2>/dev/null
pkill -f "slam_toolbox" 2>/dev/null
pkill -f "rf2o_laser" 2>/dev/null
pkill -f "ekf_node" 2>/dev/null
pkill -f "rosbridge" 2>/dev/null
fuser -k 9090/tcp 2>/dev/null
sleep 3

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/vmuser/cyclone_dds.xml
export GEMINI_API_KEY="XXXXXXXXXXXXXXXXXXXXXXXXXXXX"

# Launch full mapping stack (same as start_mac.sh — includes Nav2 after 10s)
echo "[*] Launching mapping stack..."
ros2 launch mac_brain mac_brain.launch.py &
LAUNCH_PID=$!

# Wait for slam_toolbox service to be ready, then load map
(
    echo "[*] Waiting for slam_toolbox to be ready..."
    until ros2 service list 2>/dev/null | grep -q "/slam_toolbox/deserialize_map"; do
        sleep 0.5
    done
    sleep 1
    echo "[*] Loading map: $MAP_NAME"
    ros2 service call /slam_toolbox/deserialize_map \
        slam_toolbox/srv/DeserializePoseGraph \
        "{filename: '/home/vmuser/ros2_ws/maps/$MAP_NAME', \
        match_type: 0, \
        initial_pose: {position: {x: 0.0, y: 0.0, z: 0.0}, \
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
    echo "✅ Map loaded! Dashboard will show the map now."
) &

cleanup() {
    echo "[*] Shutting down..."
    kill $LAUNCH_PID 2>/dev/null
    pkill -f "slam_toolbox" 2>/dev/null
    exit 0
}

trap cleanup SIGINT SIGTERM
wait $LAUNCH_PID
