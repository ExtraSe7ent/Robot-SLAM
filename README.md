# Security Robot - Security Robot with SLAM, Autonomous Navigation & AI Threat Detection

A DIY security robot built on ROS 2 Humble featuring real-time SLAM mapping, LiDAR-based odometry, IMU fusion, autonomous navigation, AI-powered threat detection (YOLO + Gemini), and a web-based control dashboard. The system runs across two compute layers: a Raspberry Pi 4 for hardware interfacing, and a separate computer (Ubuntu 22 VM) for heavy SLAM, Nav2, and Security AI algorithms.

---

## Hardware

| Component       | Model                                    | Connection                                        |
| --------------- | ---------------------------------------- | ------------------------------------------------- |
| Microcontroller | STM32F103 (Blue Pill)                    | UART GPIO to Pi (115200 baud)                     |
| Motor driver    | L298N H-Bridge                           | STM32 PWM + direction pins                        |
| Wheels          | 4× Mecanum wheel + DC motor (no encoder) | L298N                                             |
| SBC             | Raspberry Pi 4 Model B                   | —                                                 |
| LiDAR           | RPLidar A1M8 (360°, 10m)                 | USB Micro-B → Pi                                  |
| IMU             | MPU6050 GY-521                           | I2C (SDA/SCL GPIO) → Pi                           |
| Camera          | IMX219 (CSI)                             | Ribbon cable → Pi                                 |
| Power           | 12V LiPo battery                         | DC-DC step-down → 5V for Pi; direct 12V for L298N |

### Wiring summary

```
Battery 12V ─┬─ L298N (12V) ─ 4× DC motor
             └─ DC-DC 5V ──── Raspberry Pi 4

Pi GPIO14/15 (ttyAMA0) ──── STM32 Serial1 (115200 baud)
Pi I2C (bus 1, 0x68) ────── MPU6050 GY-521
Pi USB ──────────────────── RPLidar A1M8 → /dev/rplidar
Pi CSI ──────────────────── IMX219 camera

Pi (WiFi) ←──── ROS 2 CycloneDDS ────→ Computer (Ubuntu 22 VM)
```

---

## Software Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     Raspberry Pi 4                          │
│                                                             │
│  cam_stream.py   ─── MJPEG :8080/stream.mjpg                │
│  mpu6050_driver  ─── /imu/data (50 Hz, yaw rate only)       │
│  sllidar_node    ─── /scan (raw)                            │
│  laser_filter    ─── /scan_filtered (0.15~12 m)             │
│  uart_bridge.py  ─── /cmd_vel + /robot_mode → STM32         │
└──────────────────────┬──────────────────────────────────────┘
                       │ CycloneDDS (WiFi)
┌──────────────────────▼──────────────────────────────────────┐
│                  Computer (Ubuntu 22 VM)                    │
│                                                             │
│  rf2o_laser_odometry ─── /odom_rf2o (LiDAR scan matching)   │
│  robot_localization  ─── /odometry/filtered (EKF fusion)    │
│  slam_toolbox        ─── /map + TF map→odom                 │
│  nav2_bringup        ─── /cmd_vel (autonomous navigation)   │
│  pose_republisher    ─── /robot_pose (for web dashboard)    │
│  draw_handler        ─── /nav/draw_path → Nav2              │
│  security_ai         ─── YOLO + Gemini → /security/status   │
│  rosbridge_suite     ─── WebSocket :9090                    │
└──────────────────────┬──────────────────────────────────────┘
                       │ WebSocket (port 9090)
┌──────────────────────▼──────────────────────────────────────┐
│              Web Dashboard (Browser on Pi :8000)            │
│  Live camera · SLAM map · PIN/PEN navigation · Joystick     │
│  Security AI badge (Safe / Analyzing / DANGEROUS)           │
└─────────────────────────────────────────────────────────────┘
```

---

## Repository Structure

```
Robot-SLAM/
├── stm32/
│   └── Wheel+UltraSonic.ino      # STM32F103 firmware — motor control + ultrasonic
│
├── raspberry_pi/
│   ├── cyclone_dds.xml            # DDS peer: vmuser-virtual-machine.local
│   ├── setup.py                   # ROS 2 package: web_pose
│   ├── start_pi.sh                # One-command Pi startup script
│   ├── launch/
│   │   └── robot_bringup.launch.py    # LiDAR + filter + TFs + IMU + motor bridge
│   ├── lidar/
│   │   └── laser_filter.yaml      # Range filter: 0.15m – 12m
│   ├── imu/
│   │   └── mpu6050_driver.py      # I2C driver → /imu/data @ 50 Hz
│   ├── motor/
│   │   └── uart_bridge.py         # /cmd_vel → UART text commands → STM32
│   ├── camera/
│   │   └── cam_stream.py          # MJPEG server → :8080/stream.mjpg
│   └── web_dashboard/
│       └── index.html             # Control dashboard (roslibjs + Security AI badge)
│
└── computer/
    ├── cyclone_dds.xml             # DDS peer: robotanninh.local
    ├── setup.py                    # ROS 2 package: mac_brain
    ├── start_mac.sh                # One-command startup: MAPPING mode
    ├── start_nav.sh                # One-command startup: NAVIGATION + load saved map
    ├── odometry_ekf/
    │   └── ekf.yaml                # EKF: rf2o odometry (vx) + IMU yaw rate
    ├── slam/
    │   ├── slam_params.yaml        # SLAM Toolbox: mapping mode
    │   └── localization_params.yaml    # SLAM Toolbox: localization mode
    ├── navigation/
    │   ├── nav2_params.yaml        # Nav2: RotationShim + RegulatedPurePursuit
    │   └── mac_nav.launch.py       # Launch: localization with saved map
    ├── nodes/
    │   ├── pose_republisher.py     # TF map→base_link → /robot_pose
    │   ├── draw_handler.py         # /nav/draw_path → NavigateThroughPoses
    │   └── security_ai.py          # YOLO11n-pose + Gemini → /security/status
    └── launch/
        └── mac_brain.launch.py     # Launch: full SLAM + Nav2 + Security AI stack
```

---

## STM32 Firmware (`stm32/`)

**File:** `Wheel+UltraSonic.ino`

Controls 4 mecanum wheels via L298N. Receives one-word text commands over UART at 115200 baud from the Raspberry Pi.

**Pins:**

- `ENA=PA0, ENB=PA1` — PWM enable for left/right motor pairs
- `IN1=PA2, IN2=PA3` — left motor direction
- `IN3=PA4, IN4=PA5` — right motor direction
- `TRIG=PB8, ECHO=PB9` — ultrasonic distance sensor

**Commands received from Pi:**

| Command          | Action                                                     |
| ---------------- | ---------------------------------------------------------- |
| `ON`             | Forward                                                    |
| `BACK`           | Backward                                                   |
| `LEFT`           | Rotate left                                                |
| `RIGHT`          | Rotate right                                               |
| `AUTO`           | Autonomous obstacle avoidance (millis-based state machine) |
| `OFF` / `MANUAL` | Stop                                                       |

**AUTO mode** uses a non-blocking `millis()` state machine. If an obstacle is detected < 40 cm, the robot backs up then turns right.

---

## Raspberry Pi Setup (`raspberry_pi/`)

### 1. Install ROS 2 Humble & dependencies

```bash
# ROS 2 Humble base
sudo apt install ros-humble-ros-base ros-dev-tools
sudo apt install ros-humble-rosbridge-suite ros-humble-rmw-cyclonedds-cpp
sudo apt install ros-humble-laser-filters

# Python libs
pip3 install smbus2 pyserial
```

### 2. Build workspace

```bash
mkdir -p ~/ros2_ws/src ~/ros2_ws/config
cd ~/ros2_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
# Copy raspberry_pi/ folder as the web_pose package
```

```bash
# Copy configs
cp raspberry_pi/lidar/laser_filter.yaml ~/ros2_ws/config/
cp raspberry_pi/cyclone_dds.xml ~/cyclone_dds.xml

cd ~/ros2_ws
colcon build --packages-select web_pose sllidar_ros2 --symlink-install
source ~/.bashrc
```

### 3. Configure environment

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc
echo 'export CYCLONEDDS_URI=file:///home/ubuntu/cyclone_dds.xml' >> ~/.bashrc
```

### 4. One-command start

```bash
chmod +x ~/start_pi.sh
~/start_pi.sh
```

Starts: LiDAR → laser filter → static TFs → IMU driver → UART bridge → web server (`:8000`) → camera stream (`:8080`).

### Topics published by Pi

| Topic            | Type        | Rate  | Description                |
| ---------------- | ----------- | ----- | -------------------------- |
| `/scan_filtered` | `LaserScan` | 10 Hz | Filtered LiDAR (0.15–12 m) |
| `/imu/data`      | `Imu`       | 50 Hz | MPU6050 yaw rate (gyro Z)  |

### Topics consumed by Pi

| Topic         | Type     | Description                     |
| ------------- | -------- | ------------------------------- |
| `/cmd_vel`    | `Twist`  | Velocity command → UART → STM32 |
| `/robot_mode` | `String` | `AUTO` or `MANUAL` → STM32      |

---

## Computer Setup (`computer/`)

### 1. Install ROS 2 Humble & dependencies

```bash
sudo apt install ros-humble-desktop ros-dev-tools
sudo apt install ros-humble-slam-toolbox ros-humble-navigation2 \
  ros-humble-nav2-bringup ros-humble-robot-localization \
  ros-humble-rmw-cyclonedds-cpp ros-humble-rosbridge-suite -y

# Security AI dependencies
sudo apt install python3-pip
pip3 install ultralytics opencv-python requests
pip3 install "numpy<2" --force-reinstall

# Download YOLO11n-pose model
wget https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n-pose.pt -O ~/yolo11n-pose.pt
```

### 2. Build workspace

```bash
mkdir -p ~/ros2_ws/src ~/ros2_ws/config ~/ros2_ws/maps
cd ~/ros2_ws/src
git clone -b humble-devel https://github.com/Adlink-ROS/rf2o_laser_odometry.git
# Copy computer/ folder as the mac_brain package

# Copy configs
cp computer/odometry_ekf/ekf.yaml           ~/ros2_ws/config/
cp computer/slam/slam_params.yaml           ~/ros2_ws/config/
cp computer/slam/localization_params.yaml   ~/ros2_ws/config/
cp computer/navigation/nav2_params.yaml     ~/ros2_ws/config/
cp computer/cyclone_dds.xml                 ~/cyclone_dds.xml

cd ~/ros2_ws
colcon build --packages-select mac_brain rf2o_laser_odometry --symlink-install
source ~/.bashrc
```

### 3. Configure environment

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc
echo 'export CYCLONEDDS_URI=file:///home/vmuser/cyclone_dds.xml' >> ~/.bashrc
echo 'export GEMINI_API_KEY="<your-key>"' >> ~/.bashrc
```

---

## Usage

### Step 1 — Start Pi hardware

```bash
# On Raspberry Pi
~/start_pi.sh
```

### Step 2 — Start computer (Mapping mode)

```bash
# On Ubuntu VM
chmod +x ~/start_mac.sh
~/start_mac.sh
```

### Step 3 — Open dashboard

Open `http://<pi-ip>:8000` in a browser.

Drive the robot around to build the SLAM map. When the map looks good, click **💾 Serialize map** and enter a name (e.g. `living_room`).

### Step 4 — Navigate with saved map

```bash
# On Ubuntu VM
chmod +x ~/start_nav.sh
~/start_nav.sh living_room
```

This launches the full brain stack and automatically calls `deserialize_map` once SLAM Toolbox is ready.

### Step 5 — Dashboard controls

| Control                   | Action                                          |
| ------------------------- | ----------------------------------------------- |
| **📍 PIN**                | Click map to set `NavigateToPose` goal          |
| **✏️ PEN**                | Draw a path on the map → `NavigateThroughPoses` |
| **🗑 ERASE & STOP**       | Cancel navigation and stop robot                |
| **▶ AUTO NAV**            | Toggle Nav2 autonomous mode via `/robot_mode`   |
| **Mouse wheel / ＋ / －** | Zoom the SLAM map                               |
| **⊡ Fit**                 | Auto-fit map to window                          |
| **Space**                 | Emergency stop                                  |
| **Arrow keys / WASD**     | Manual drive                                    |

---

## Security AI (`computer/nodes/security_ai.py`)

Runs on the computer, starts 20 seconds after launch to let SLAM/Nav2 stabilize.

**Pipeline:**

1. **MJPEG capture** — streams from `http://robotanninh.local:8080/stream.mjpg`
2. **YOLO11n-pose** — local inference, detects persons and estimates 17 keypoints
3. **Pose heuristics** — flags suspicious postures (raised arms, forward bend, fast movement, wide arm span) without any API call
4. **Gemini 2.5 Flash** — called only when YOLO flags a suspicious pose, with a 5s cooldown and 500 calls/day limit
5. **Publish** — `/security/status` (JSON String) every 0.5s

**Dashboard badge states:**

| Badge                       | Meaning                                    |
| --------------------------- | ------------------------------------------ |
| `Security AI: —`            | Node not running yet                       |
| `Security AI: No person`    | No person in frame                         |
| `Security AI: Safe`         | Person detected, normal pose               |
| `Security AI: Analyzing...` | Suspicious pose flagged, calling Gemini    |
| `⚠ <reason>`                | Gemini confirmed threat (badge pulses red) |

**Environment variable required:**

```bash
export GEMINI_API_KEY="your-api-key-here"
```

---

## Network Configuration

Both machines must be on the same local network (WiFi).

| Setting              | Value                          |
| -------------------- | ------------------------------ |
| `ROS_DOMAIN_ID`      | `42`                           |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp`           |
| Pi hostname          | `robotanninh`                  |
| Pi mDNS address      | `robotanninh.local`            |
| Computer mDNS        | `vmuser-virtual-machine.local` |
| Rosbridge WebSocket  | `computer:9090`                |
| Camera stream        | `pi:8080/stream.mjpg`          |
| Web dashboard        | `pi:8000`                      |

---

## Key Design Decisions

**No wheel encoders** — `rf2o_laser_odometry` computes odometry by matching consecutive LiDAR scans, eliminating the need for wheel encoders. Mecanum wheels slip laterally, making encoder-based odometry unreliable.

**EKF fusion** — `robot_localization` fuses rf2o linear velocity (X axis only) with MPU6050 yaw rate to produce `/odometry/filtered`. Only these two signals are fused; full 6-DOF is unnecessary for a flat-floor 2D robot.

**RotationShim + RegulatedPurePursuit** — `nav2_params.yaml` wraps RPP with `RotationShimController` so the robot first rotates toward the goal heading before translating. This prevents the robot from driving sideways toward waypoints.

**Split compute** — SLAM Toolbox, Nav2, and Security AI run on the computer to avoid Pi thermal throttling. All ROS 2 topics bridge transparently via CycloneDDS over WiFi.

**Two-stage nav startup** — `start_nav.sh` uses the same full `mac_brain.launch.py` stack (not a separate localization launch). After slam_toolbox starts, it calls `deserialize_map` service to load the saved pose graph. This avoids maintaining two separate launch configurations.

**Security AI two-tier filtering** — YOLO pose heuristics filter out obvious normal cases locally for free. Gemini is only called when a pose is flagged as suspicious, reducing API usage to a few hundred calls per day even in busy environments.

**Deadband matching** — `nav2_params.yaml` sets `deadband_velocity: [0.05, 0.0, 0.15]` to match `LIN_THRESH=0.05` and `ANG_THRESH=0.15` in `uart_bridge.py`, ensuring Nav2 never generates commands the UART bridge would silently drop.

---
