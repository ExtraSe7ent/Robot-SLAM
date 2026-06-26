import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource, AnyLaunchDescriptionSource)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    rosbridge_dir    = get_package_share_directory('rosbridge_server')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    ekf_config_path  = os.path.expanduser('~/ros2_ws/config/ekf.yaml')
    slam_config_path = os.path.expanduser('~/ros2_ws/config/slam_params.yaml')
    nav2_params_path = os.path.expanduser('~/ros2_ws/config/nav2_params.yaml')

    return LaunchDescription([

        # 1. rf2o_laser_odometry — odometry từ scan LiDAR (không cần encoder)
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/scan_filtered',
                'odom_topic':       '/odom_rf2o',
                'publish_tf':       False,
                'base_frame_id':    'base_link',
                'odom_frame_id':    'odom',
                'init_pose_from_topic': '',
                'freq':             20.0,
            }]
        ),

        # 2. EKF — kết hợp rf2o odometry + IMU yaw rate
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path],
        ),

        # 3. SLAM Toolbox — lập bản đồ realtime
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')),
            launch_arguments={
                'use_sim_time':    'false',
                'slam_params_file': slam_config_path,
            }.items()
        ),

        # 4. Rosbridge — WebSocket server cho web dashboard
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(rosbridge_dir, 'launch', 'rosbridge_websocket_launch.xml')),
            launch_arguments={'port': '9090'}.items()
        ),

        # 5. Pose republisher — TF map→base_link → /robot_pose
        Node(
            package='mac_brain',
            executable='pose_republisher',
            name='pose_republisher',
            output='screen',
        ),

        # 6. Nav2 — khởi động sau 10s để SLAM/EKF ổn định trước
        TimerAction(period=10.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file':  nav2_params_path,
                    'autostart':    'true',
                }.items()
            )
        ]),

        # 7. Draw Handler
        Node(
            package='mac_brain',
            executable='draw_handler',
            name='draw_handler',
            output='screen',
        ),

        # 8. Security AI 
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='mac_brain',
                    executable='security_ai',
                    name='security_ai',
                    output='screen',
                ),
            ]
        ),

    ])
