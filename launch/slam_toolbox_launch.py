from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('homebot_0')

    homebot_launch = os.path.join(pkg_share, 'launch', 'homebot_launch.py')
    lidar_launch = os.path.join(pkg_share, 'launch', 'lidar_launch.py')
    slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox_config.yaml')
    ekf_mapping = os.path.join(pkg_share, 'config', 'ekf_mapping.yaml')

    return LaunchDescription([
        # Robot hardware, base, tfs
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(homebot_launch)
        ),
        # LIDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch)
        ),
        # EKF: wheel odom + IMU, outputs /odom and tf odom->base_footprint
        Node(
            package='robot_localization',
            node_executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_mapping, {'use_sim_time': False}],
            remappings=[
                ('/odometry/filtered', '/odom'),   # EKF publishes tf and /odom topic
                ('/imu', '/imu'),
                ('/odom_raw', '/odom_raw')
            ]
        ),
        # SLAM Toolbox: builds map, publishes map->odom
        Node(
            package='slam_toolbox',
            node_executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params]
        )
    ])
