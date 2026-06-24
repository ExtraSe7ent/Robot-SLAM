"""
Launch file — Raspberry Pi Hardware Nodes
Khởi động đồng thời: LiDAR → Laser Filter → Static TFs → IMU → UART Bridge (STM32)
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    filter_config = os.path.join(
        os.path.expanduser('~'), 'ros2_ws', 'config', 'laser_filter.yaml'
    )

    return LaunchDescription([

        # ── 1. LiDAR Driver: RPLidar A1M8 ──────────────────────────────
        # Publish /scan (raw) tại 115200 baud qua /dev/rplidar
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port':     '/dev/rplidar',
                'serial_baudrate': 115200,
                'frame_id':        'laser',
                'scan_mode':       'Standard'
            }],
            output='screen'
        ),

        # ── 2. Laser Filter: Lọc nhiễu LiDAR ───────────────────────────
        # /scan → /scan_filtered (bỏ điểm < 15cm và > 12m)
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter',
            parameters=[filter_config],
            remappings=[
                ('scan',          '/scan'),
                ('scan_filtered', '/scan_filtered')
            ],
            output='screen'
        ),

        # ── 3. Static TF: base_link → laser (LiDAR ở độ cao 0.23m) ────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.23',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'base_link', '--child-frame-id', 'laser'
            ],
            output='screen'
        ),

        # ── 4. Static TF: base_link → imu_link (IMU ở độ cao 0.09m) ───
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.09',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'base_link', '--child-frame-id', 'imu_link'
            ],
            output='screen'
        ),

        # ── 5. IMU Driver: MPU6050 GY-521 ──────────────────────────────
        # Publish /imu/data (gyroscope Z) qua I2C
        Node(
            package='web_pose',
            executable='mpu6050_driver',
            name='mpu6050_driver',
            output='screen'
        ),

        # ── 6. UART Bridge: Pi → STM32 → L298N → Motors ────────────────
        # Subscribe /cmd_vel, /robot_mode → gửi lệnh text qua /dev/ttyAMA0
        Node(
            package='web_pose',
            executable='uart_bridge',
            name='uart_bridge',
            output='screen'
        ),
    ])
