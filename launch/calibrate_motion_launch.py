from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Choose mode via launch argument (linear or angular)
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='linear',
            description='Calibration mode: linear or angular'
        ),
        Node(
            package='homebot_0',
            executable='calibrate_motion.py',
            name='calibrate_motion',
            output='screen',
            parameters=[
                {
                    'mode': LaunchConfiguration('mode'),
                    # Linear calibration defaults:
                    'test_distance': 1.0,
                    'speed': 0.3,
                    'tolerance': 0.01,
                    'odom_linear_scale_correction': 1.0,
                    # Angular calibration defaults (overridden if mode=angular):
                    'test_angle': 360.0,
                    'odom_angular_scale_correction': 1.0,
                    # Common
                    'start_test': True,
                    'base_frame': 'base_footprint',
                    'odom_frame': 'odom',
                    'rate': 20
                }
            ]
        )
    ])

