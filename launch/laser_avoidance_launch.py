from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('homebot_0')
    lidar_launch = os.path.join(pkg_share, 'launch', 'lidar_launch.py')

    return LaunchDescription([
        # Launch the main robot node (homebot)
        Node(
            package='homebot_0',
            node_executable='homebot',
            name='homebot',
            output='screen',
            parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml')],
            remappings=[('/odom', '/odom_raw')]
        ),

        # Bring up your LIDAR (RPLIDAR A1 etc)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch)
        ),


        # Launch the laser avoidance node
        Node(
            package='homebot_0',
            node_executable='laser_avoidance.py',
            name='laser_avoidance',
            output='screen'
        ),
    ])
