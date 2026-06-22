#!/bin/bash
echo "=========================================="
echo "   STARTING ROBOT HARDWARE (RASPBERRY PI) "
echo "=========================================="
# 1. Cleanup old processes to prevent port conflicts
echo "[*] Cleaning up old processes (Ports 8000, 8080)..."
fuser -k 8080/tcp 8000/tcp 2>/dev/null
sleep 1
# 2. Source ROS 2 environment
echo "[*] Sourcing ROS 2 Workspace..."
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
# 3. Configure DDS Network
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/ubuntu/cyclone_dds.xml
# 4. Start ROS 2 Hardware Nodes in Background
echo "[*] Launching ROS 2 (LiDAR, IMU, UART Bridge)..."
echo " -> NOTE: If Lidar gets stuck (80008002 error), gently spin it by hand!"
ros2 launch web_pose robot_bringup.launch.py &
PID_ROS=$!
sleep 3
# 5. Start Web Dashboard Server (Port 8000)
echo "[*] Starting Web Server (Port 8000)..."
cd ~/web
python3 -m http.server 8000 > /dev/null 2>&1 &
PID_WEB=$!
# 6. Start Camera Stream (Port 8080)
echo "[*] Starting Camera Stream (Port 8080)..."
python3 ~/web/cam_stream.py > /dev/null 2>&1 &
PID_CAM=$!
# 7. Display connection info
IP=$(hostname -I | awk '{print $1}')
echo "=========================================="
echo "✅ PI HARDWARE IS RUNNING!"
echo "➡️ Dashboard : http://$IP:8000"
echo "➡️ Camera : http://$IP:8080/stream.mjpg"
echo "=========================================="
echo "Press Ctrl+C to safely stop all processes."
# 8. Trap Ctrl+C to cleanly kill background processes
cleanup() {
echo ""
echo "[*] Shutting down processes safely..."
kill $PID_ROS $PID_WEB $PID_CAM 2>/dev/null
fuser -k 8080/tcp 8000/tcp 2>/dev/null
echo "[*] Shutdown complete."
exit 0
}
trap cleanup SIGINT SIGTERM
wait $PID_ROS
