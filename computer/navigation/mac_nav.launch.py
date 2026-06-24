import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource, AnyLaunchDescriptionSource)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        description='Tên map đã lưu (không có đuôi) — phải tồn tại trong ~/ros2_ws/maps/'
    )
    map_name = LaunchConfiguration('map_name')

    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    rosbridge_dir    = get_package_share_directory('rosbridge_server')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    ekf_config_path   = os.path.expanduser('~/ros2_ws/config/ekf.yaml')
    loc_config_path   = os.path.expanduser('~/ros2_ws/config/localization_params.yaml')
    nav2_params_path  = os.path.expanduser('~/ros2_ws/config/nav2_params.yaml')

    return LaunchDescription([
        map_name_arg,

        # 1. rf2o_laser_odometry
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

        # 2. EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path],
        ),

        # 3. SLAM Toolbox — chế độ localization (load map cũ)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_dir, 'launch', 'localization_launch.py')),
            launch_arguments={
                'use_sim_time':    'false',
                'slam_params_file': loc_config_path,
            }.items()
        ),

        # 4. Rosbridge
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(rosbridge_dir, 'launch', 'rosbridge_websocket_launch.xml')),
            launch_arguments={'port': '9090'}.items()
        ),

        # 5. Pose republisher
        Node(
            package='mac_brain',
            executable='pose_republisher',
            name='pose_republisher',
            output='screen',
        ),

        # 6. Nav2
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

        # 7. Draw handler
        Node(
            package='mac_brain',
            executable='draw_handler',
            name='draw_handler',
            output='screen',
        ),

    ])
