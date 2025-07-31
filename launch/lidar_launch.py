from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the RPLidar A1 node from sllidar_ros2
        Node(
    package='rplidar_ros',
    node_executable='rplidar_node',
    name='rplidar_node',
    output='screen',
    parameters=[{
        'serial_port': '/dev/ttyACM1',
        'frame_id': 'laser_frame',
        'serial_baudrate': 115200,
        'inverted': False,
        'angle_compensate': True
         }]
	),


        # Static transform: base_footprint -> laser_frame
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            name='base_footprint_to_laser',
            arguments=['0', '0', '0.15', '3.14', '0', '0', 'base_footprint', 'laser_frame'],
            output='screen'
        )
    ])

