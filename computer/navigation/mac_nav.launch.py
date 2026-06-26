import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rosbridge_dir    = get_package_share_directory('rosbridge_server')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    ekf_config_path  = os.path.expanduser('~/ros2_ws/config/ekf.yaml')
    loc_config_path  = os.path.expanduser('~/ros2_ws/config/localization_params.yaml')
    nav2_params_path = os.path.expanduser('~/ros2_ws/config/nav2_params.yaml')
    maps_dir         = os.path.expanduser('~/ros2_ws/maps')

    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='my_map',
        description='Map file name in ~/ros2_ws/maps/ (without file extension)'
    )

    map_file_path = PathJoinSubstitution([maps_dir, LaunchConfiguration('map_name')])

    return LaunchDescription([
        map_name_arg,

        # 1. LiDAR odometry via rf2o
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
                'freq':             20.0
            }]
        ),

        # 2. EKF — fuse rf2o (vx) + IMU (yaw_rate) → /odometry/filtered + TF odom→base_link
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path]
        ),

        # 3. SLAM Toolbox localization mode — load saved map, publish TF map→odom
        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                loc_config_path,
                {'map_file_name': map_file_path}
            ]
        ),

        # 4. Nav2 — path planning and robot control (no AMCL, slam_toolbox handles map→odom TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file':  nav2_params_path,
                'autostart':    'true'
            }.items()
        ),

        # 5. Rosbridge WebSocket server on port 9090 for Web Dashboard
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(rosbridge_dir, 'launch', 'rosbridge_websocket_launch.xml')
            ),
            launch_arguments={'port': '9090'}.items()
        ),

        # 6. Pose republisher: TF map→base_link → /robot_pose topic for dashboard
        Node(
            package='mac_brain',
            executable='pose_republisher',
            name='pose_republisher',
            output='screen'
        ),
    ])
