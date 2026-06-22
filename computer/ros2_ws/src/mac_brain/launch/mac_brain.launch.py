import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    rosbridge_dir = get_package_share_directory('rosbridge_server')
    
    ekf_config_path = os.path.expanduser('~/ros2_ws/config/ekf.yaml')
    slam_config_path = os.path.expanduser('~/ros2_ws/config/slam_params.yaml')

    return LaunchDescription([
        # 1. Calculate Odometry from Lidar (rf2o)
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/scan_filtered',
                'odom_topic': '/odom_rf2o',
                'publish_tf': False, # TF is handled by EKF
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'freq': 20.0
            }]
        ),

        # 2. EKF Node for smooth coordinates (Fusing Lidar and IMU)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path]
        ),

        # 3. Launch SLAM for mapping (using configured increased Queue Size)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'slam_params_file': slam_config_path
            }.items()
        ),

        # 4. Open WebSocket Server on port 9090 for Web Dashboard
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(rosbridge_dir, 'launch', 'rosbridge_websocket_launch.xml')
            ),
            launch_arguments={'port': '9090'}.items()
        )
    ])
