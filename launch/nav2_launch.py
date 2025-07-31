from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('homebot_0')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    ekf_params = os.path.join(pkg_share, 'config', 'ekf_localization.yaml')
    map_yaml = os.path.join(pkg_share, 'maps', 'my_map.yaml')  # <--- Use your map!
    lidar_launch = os.path.join(pkg_share, 'launch', 'lidar_launch.py')

    return LaunchDescription([
        # Bring up robot hardware node
        Node(
            package='homebot_0',
            node_executable='homebot',
            name='homebot',
            output='screen'
        ),
        # Bring up your LIDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch)
        ),
        # Static TF: base_footprint -> imu (if you use IMU frame)
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            name='base_footprint_to_imu',
            arguments=['0', '0', '0.07', '0', '0', '0', 'base_footprint', 'base_imu_link'],
            output='screen'
        ),
        # EKF Sensor Fusion for odometry
        Node(
            package='robot_localization',
            node_executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params],
            remappings=[
                ('/odometry/filtered', '/odom'),
                ('/imu', '/imu'),
                ('/odom_raw', '/odom_raw'),
            ]
        ),
        # Map Server
        Node(
            package='nav2_map_server',
            node_executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_params, {'yaml_filename': map_yaml}]
        ),
        # AMCL (Monte Carlo Localization)
        Node(
            package='nav2_amcl',
            node_executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params]
        ),
        # Nav2 Core Nodes (Planner, Controller, Recovery, BT)
        Node(
            package='nav2_controller',
            node_executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params]
        ),
        Node(
            package='nav2_planner',
            node_executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params]
        ),
        Node(
            package='nav2_recoveries',
            node_executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[nav2_params]
        ),
        Node(
            package='nav2_bt_navigator',
            node_executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params]
        ),
        Node(
            package='nav2_lifecycle_manager',
            node_executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[nav2_params, {
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'planner_server',
                    'controller_server',
                    'recoveries_server',
                    'bt_navigator'
                ]
            }]
        )
    ])

