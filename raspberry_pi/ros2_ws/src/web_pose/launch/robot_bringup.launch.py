import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Construct the absolute path to the C++ filter config file
    filter_config = os.path.join(
        os.path.expanduser('~'), 'ros2_ws', 'config', 'laser_filter.yaml'
    )

    return LaunchDescription([
        # 1. LiDAR Driver Node
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port': '/dev/rplidar',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'scan_mode': 'Standard'
            }],
            output='screen'
        ),

        # 2. C++ Laser Scan Filter Node (Cuts off noise < 15cm)
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter',
            parameters=[filter_config],
            remappings=[
                ('scan', '/scan'),
                ('scan_filtered', '/scan_filtered')
            ],
            output='screen'
        ),

        # 3. Static Transform: base_link -> laser (LiDAR height: 0.23m)
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

        # 4. Static Transform: base_link -> imu_link (IMU height: 0.09m)
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

        # 5. MPU6050 IMU Driver Node
        Node(
            package='web_pose',
            executable='mpu6050_driver',
            name='mpu6050_driver',
            output='screen'
        ),

        # 6. STM32 UART Bridge Node
        Node(
            package='web_pose',
            executable='uart_bridge',
            name='uart_bridge',
            output='screen'
        )
    ])
