from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='homebot_0',
            node_executable='homebot',
            name='homebot',
            output='screen',
            parameters=[{
                'port_name': '/dev/ttyACM0',
                'linear_correction': 1.0,
                'angular_correction': 1.0,
                'publish_odom_transform': False,
                'use_sim_time': False
            }],
            remappings=[
                ('/odom', '/odom_raw')
            ]
        ),
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            name='base_footprint_to_imu',
            arguments=['0', '0', '0.07', '0', '0', '0', 'base_footprint', 'base_imu_link'],
            output='screen'
        )
    ])

