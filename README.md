# Robot An Ninh — Security Robot with SLAM & Autonomous Navigation

A DIY security robot built on ROS 2 Humble featuring real-time SLAM mapping, LiDAR-based odometry, IMU fusion, autonomous navigation, and a web-based control dashboard. The system runs across two compute layers: a Raspberry Pi 4 for hardware interfacing, and a separate computer (Ubuntu VM) for heavy SLAM and Nav2 algorithms.

---

## Hardware

| Component | Model | Connection |
|-----------|-------|------------|
| Microcontroller | STM32F103 (Blue Pill) | UART GPIO to Pi (115200 baud) |
| Motor driver | L298N H-Bridge | STM32 PWM + direction pins |
| Wheels | 4× Mecanum wheel + DC motor (no encoder) | L298N |
| SBC | Raspberry Pi 4 Model B | — |
| LiDAR | RPLidar A1M8 (360°, 10m) | USB Micro-B → Pi |
| IMU | MPU6050 GY-521 | I2C (SDA/SCL GPIO) → Pi |
| Camera | IMX219 (CSI) | Ribbon cable → Pi |
| Power | 12V LiPo battery | DC-DC step-down → 5V for Pi; direct 12V for L298N |

### Wiring summary

```
Battery 12V ─┬─ L298N (12V) ─ 4× DC motor
             └─ DC-DC 5V ──── Raspberry Pi 4

Pi GPIO14/15 (ttyAMA0) ──── STM32 Serial1 (115200 baud)
Pi I2C (bus 1, 0x68) ────── MPU6050 GY-521
Pi USB ──────────────────── RPLidar A1M8 → /dev/rplidar
Pi CSI ──────────────────── IMX219 camera

Pi (WiFi) ←──── ROS 2 CycloneDDS ────→ Computer (Ubuntu VM)
```

---

## Software Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     Raspberry Pi 4                          │
│                                                             │
│  cam_stream.py   ─── MJPEG :8080/stream.mjpg               │
│  mpu6050_driver  ─── /imu/data (50 Hz, yaw rate only)      │
│  sllidar_node    ─── /scan (raw)                            │
│  laser_filter    ─── /scan_filtered (0.15~12 m)            │
│  uart_bridge.py  ─── /cmd_vel + /robot_mode → STM32        │
└──────────────────────┬──────────────────────────────────────┘
                       │ CycloneDDS (ROS_DOMAIN_ID=42)
┌──────────────────────▼──────────────────────────────────────┐
│                  Computer (Ubuntu VM)                        │
│                                                             │
│  rf2o_laser_odometry ─── /odom_rf2o (LiDAR scan matching)  │
│  robot_localization  ─── /odometry/filtered (EKF fusion)   │
│  slam_toolbox        ─── /map + TF map→odom                │
│  nav2_bringup        ─── /cmd_vel (autonomous navigation)  │
│  pose_republisher    ─── /robot_pose (for web dashboard)   │
│  draw_handler        ─── /nav/draw_path → Nav2             │
│  rosbridge_suite     ─── WebSocket :9090                   │
└──────────────────────┬──────────────────────────────────────┘
                       │ WebSocket (roslibjs)
┌──────────────────────▼──────────────────────────────────────┐
│              Web Dashboard (Browser)                         │
│  Live camera · SLAM map · PIN/PEN navigation · Joystick     │
└─────────────────────────────────────────────────────────────┘
```

---

## Repository Structure

```
Robot/
├── stm32/
│   └── Wheel+UltraSonic.ino    # STM32F103 firmware — motor control + ultrasonic
│
├── raspberry_pi/
│   ├── cyclone_dds.xml          # DDS peer: vmuser-virtual-machine.local
│   ├── setup.py                 # ROS 2 package: web_pose
│   ├── start_pi.sh              # One-command Pi startup script
│   ├── launch/
│   │   └── robot_bringup.launch.py  # LiDAR + IMU + motor bridge
│   ├── lidar/
│   │   └── laser_filter.yaml    # Range filter: 0.15m – 12m
│   ├── imu/
│   │   └── mpu6050_driver.py    # I2C driver → /imu/data @ 50 Hz
│   ├── motor/
│   │   └── uart_bridge.py       # /cmd_vel → UART text commands → STM32
│   ├── camera/
│   │   └── cam_stream.py        # MJPEG server → :8080/stream.mjpg
│   └── web_dashboard/
│       └── index.html           # Control dashboard (roslibjs)
│
└── computer/
    ├── cyclone_dds.xml           # DDS peer: robotanninh.local
    ├── setup.py                  # ROS 2 package: mac_brain
    ├── start_mac.sh              # One-command startup: MAPPING mode
    ├── start_nav.sh              # One-command startup: NAVIGATION mode
    ├── odometry_ekf/
    │   └── ekf.yaml              # EKF: rf2o odometry + IMU yaw rate
    ├── slam/
    │   ├── slam_params.yaml      # SLAM Toolbox: mapping mode
    │   └── localization_params.yaml  # SLAM Toolbox: localization mode
    ├── navigation/
    │   ├── nav2_params.yaml      # Nav2: RPP controller + costmaps
    │   └── mac_nav.launch.py     # Launch: navigation with saved map
    ├── nodes/
    │   ├── pose_republisher.py   # TF map→base_link → /robot_pose
    │   └── draw_handler.py       # /nav/draw_path → NavigateThroughPoses
    └── launch/
        └── mac_brain.launch.py   # Launch: full SLAM + Nav2 stack
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

| Command | Action |
|---------|--------|
| `ON` / `UP` | Forward |
| `BACK` | Backward |
| `LEFT` | Rotate left |
| `RIGHT` | Rotate right |
| `UP_LEFT` | Forward-left diagonal |
| `UP_RIGHT` | Forward-right diagonal |
| `DOWN_LEFT` | Backward-left diagonal |
| `DOWN_RIGHT` | Backward-right diagonal |
| `AUTO` | Autonomous obstacle avoidance (millis-based state machine) |
| `MANUAL` / `OFF` | Stop |

**AUTO mode** uses a non-blocking `millis()` state machine. If an obstacle is detected < 40 cm, the robot backs up then turns right.

---

## Raspberry Pi Setup (`raspberry_pi/`)

### Prerequisites

```bash
# ROS 2 Humble
sudo apt install ros-humble-desktop ros-humble-sllidar-ros \
  ros-humble-laser-filters ros-humble-rosbridge-suite \
  python3-smbus

pip3 install pyserial

# Allow GPIO UART
sudo raspi-config  # Interface → Serial → disable login shell, enable UART
```

### ROS 2 package setup

```bash
mkdir -p ~/ros2_ws/src
cp -r raspberry_pi ~/ros2_ws/src/web_pose
cd ~/ros2_ws && colcon build
```

### Configuration

Copy `raspberry_pi/cyclone_dds.xml` to `/home/pi/cyclone_dds.xml`.  
Edit the peer address if your computer hostname differs from `vmuser-virtual-machine.local`.

### One-command start

```bash
chmod +x ~/ros2_ws/start_pi.sh
~/ros2_ws/start_pi.sh
```

This starts: LiDAR → laser filter → IMU driver → motor UART bridge → web server → camera stream.

### Topics published by Pi

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/scan_filtered` | `LaserScan` | 10 Hz | Filtered LiDAR (0.15–12 m) |
| `/imu/data` | `Imu` | 50 Hz | MPU6050 yaw rate (gyro Z) |

### Topics consumed by Pi

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `Twist` | Velocity command → UART → STM32 |
| `/robot_mode` | `String` | `AUTO` or `MANUAL` → STM32 |

---

## Computer Setup (`computer/`)

### Prerequisites

```bash
# Ubuntu 22.04 + ROS 2 Humble
sudo apt install ros-humble-desktop ros-humble-slam-toolbox \
  ros-humble-nav2-bringup ros-humble-rosbridge-suite \
  ros-humble-robot-localization ros-humble-rf2o-laser-odometry \
  ros-humble-rmw-cyclonedds-cpp
```

### ROS 2 package setup

```bash
mkdir -p ~/ros2_ws/src ~/ros2_ws/config ~/ros2_ws/maps
cp -r computer ~/ros2_ws/src/mac_brain

# Copy config files
cp computer/odometry_ekf/ekf.yaml           ~/ros2_ws/config/
cp computer/slam/slam_params.yaml           ~/ros2_ws/config/
cp computer/slam/localization_params.yaml   ~/ros2_ws/config/
cp computer/navigation/nav2_params.yaml     ~/ros2_ws/config/
cp computer/cyclone_dds.xml                 ~/cyclone_dds.xml

cd ~/ros2_ws && colcon build
```

Edit `~/cyclone_dds.xml` if your Pi hostname differs from `robotanninh.local`.

---

## Usage

### Step 1 — Start Pi

```bash
# On Raspberry Pi
~/ros2_ws/start_pi.sh
```

### Step 2 — Start computer (Mapping mode)

```bash
# On computer / Ubuntu VM
chmod +x ~/ros2_ws/start_mac.sh
~/ros2_ws/start_mac.sh
```

### Step 3 — Open dashboard

Open `http://<pi-ip>:8000` in a browser.

Drive the robot around to build the SLAM map. When the map looks complete, click **💾 Serialize map**, enter a name (e.g. `tang1`).

### Step 4 — Navigate with saved map

```bash
# On computer / Ubuntu VM
chmod +x ~/ros2_ws/start_nav.sh
~/ros2_ws/start_nav.sh tang1
```

### Step 5 — Send navigation goals from dashboard

| Control | Action |
|---------|--------|
| **📍 PIN** | Click map to set `NavigateToPose` goal |
| **✏️ PEN** | Draw a path on the map → `NavigateThroughPoses` |
| **🗑 ERASE & STOP** | Cancel navigation and stop robot |
| **Mouse wheel / ＋ / －** | Zoom the map |
| **⊡ Fit** | Auto-fit map to window |
| **Space** | Emergency stop |
| **Arrow keys / WASD** | Manual drive |

---

## Network Configuration

Both machines must be on the same local network.

| Setting | Value |
|---------|-------|
| `ROS_DOMAIN_ID` | `42` |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` |
| Pi hostname | `robotanninh.local` |
| Computer hostname | `vmuser-virtual-machine.local` |
| Rosbridge WebSocket | `computer:9090` |
| Camera stream | `pi:8080/stream.mjpg` |
| Web dashboard | `pi:8000` |

---

## Key Design Decisions

**No wheel encoders** — `rf2o_laser_odometry` computes odometry by matching consecutive LiDAR scans, eliminating the need for wheel encoders. Mecanum wheels slip laterally, making encoder-based odometry unreliable anyway.

**EKF fusion** — `robot_localization` fuses rf2o linear velocity (X axis only) with MPU6050 yaw rate to produce `/odometry/filtered`. Only these two signals are fused; full 6-DOF is unnecessary for a flat-floor 2D robot.

**Split compute** — SLAM Toolbox + Nav2 run on the computer to avoid Pi thermal throttling during compute-intensive mapping. All ROS 2 topics bridge transparently via CycloneDDS across WiFi.

**Deadband matching** — `nav2_params.yaml` sets `deadband_velocity: [0.05, 0.0, 0.15]` to exactly match `LIN_THRESH=0.05` and `ANG_THRESH=0.15` in `uart_bridge.py`, ensuring Nav2 does not generate commands that the UART bridge would silently drop.

**Nav2 10s delay** — `mac_brain.launch.py` delays Nav2 startup by 10 seconds so SLAM Toolbox has time to publish the first `/map` and EKF can establish the `odom→base_link` TF before Nav2 starts querying them.

---

## License

MIT
