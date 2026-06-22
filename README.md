# Autonomous Security Robot System

This repository contains the software for an autonomous security robot built with ROS 2 (Humble). The architecture is distributed across three main computing layers: a Main Computer for heavy computations (SLAM & Navigation), a Raspberry Pi 4 for hardware interfacing and web dashboard hosting, and an STM32 microcontroller for real-time motor control.

## Hardware Architecture

The robot is built using the following components:

*   **Chassis & Motors:** 4 Mecanum wheels driven by 4 DC motors (without encoders).
*   **Motor Driver:** 1x L298N dual H-bridge motor driver controlling all 4 motors.
*   **Low-level Controller:** STM32 microcontroller. The L298N is connected to the STM32 for PWM and direction control. The STM32 connects to the Raspberry Pi 4 via GPIO pins (UART communication).
*   **Main Computer (Robot Brain):** Raspberry Pi 4 Model B.
*   **Sensors:**
    *   **IMU:** MPU6050 (GY-521) connected to the Raspberry Pi via GPIO (I2C).
    *   **LiDAR:** RPLidar A1M8 connected to the Raspberry Pi via a micro USB cable.
    *   **Camera:** IMX219 Camera Module connected to the Raspberry Pi via the CSI flat ribbon cable.
*   **Power Supply:** A 12V battery powers the entire robot. A step-down buck converter is used to supply stable 5V power to the Raspberry Pi.

## Software Architecture & Libraries

The software relies heavily on ROS 2 and several specialized libraries to process sensor data, perform mapping, and serve a web interface.

### 1. Main Computer (`computer/`)
This layer handles the heavy lifting, such as sensor fusion and mapping, using the following packages:
*   **ROS 2 Humble Desktop:** The core middleware framework.
*   **CycloneDDS (`rmw_cyclonedds_cpp`):** Used as the default RMW implementation to seamlessly bridge the network between the main computer and the Raspberry Pi.
*   **`rf2o_laser_odometry`:** A fast and precise algorithm used to estimate the 2D planar odometry of the robot from planar laser scans. Since the robot does not use wheel encoders, this is critical for tracking movement.
*   **Robot Localization (`robot_localization`):** Utilizes an Extended Kalman Filter (EKF) to fuse the laser odometry (`rf2o`) and IMU data (`mpu6050`), providing a smooth and accurate coordinate transformation.
*   **SLAM Toolbox (`slam_toolbox`):** Provides 2D SLAM (Simultaneous Localization and Mapping) capabilities to map the environment in real-time.
*   **Rosbridge Suite (`rosbridge_server`):** Exposes ROS 2 topics, services, and parameters via WebSockets, allowing the web dashboard to communicate with the ROS network.
*   **Navigation2 (`navigation2`, `nav2_bringup`):** Libraries set up for future path planning and autonomous navigation.

### 2. Raspberry Pi 4 (`raspberry_pi/`)
This layer handles hardware interfacing and hosts the user interface.
*   **ROS 2 Humble Base:** Minimal ROS 2 installation.
*   **Slamtec ROS 2 (`sllidar_ros2`):** The official ROS 2 driver for the RPLidar A1M8, publishing `/scan` data.
*   **Laser Filters (`laser_filters`):** Used to apply a `LaserScanRangeFilter` to crop out false reflections from the robot's own chassis (noise < 15cm).
*   **Python `smbus2`:** Used to write the custom I2C driver (`mpu6050_driver.py`) to read and calibrate the MPU6050 IMU sensor.
*   **Python `pyserial`:** Used in `uart_bridge.py` to establish reliable UART communication with the STM32.
*   **`rpicam-apps` (libcamera):** Specifically uses `rpicam-vid` to capture and stream the IMX219 camera feed as a low-latency MJPEG stream (`cam_stream.py`), bypassing ROS 2 overhead for video to prevent memory leaks.
*   **Web Technologies (`roslibjs`):** The dashboard (`index.html`) relies on `roslib.min.js` to connect to the Rosbridge WebSocket, allowing browser-based joystick control and live map rendering.

### 3. STM32 Microcontroller (`stm32/robot_final.ino`)
*   **Arduino Core for STM32:** The firmware is written using the Arduino framework for STM32.
*   It directly drives the L298N motor driver using hardware PWM and digital I/O.
*   Implements a custom non-blocking state machine to ensure continuous UART command parsing without delays.
