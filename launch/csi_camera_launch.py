from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_homebot = get_package_share_directory('homebot_0')
    cam_calib_path = os.path.join(pkg_homebot, 'config', 'camera_calibration', 'cam_640x480.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('sensor_id', default_value='0'),
        DeclareLaunchArgument('cam_name', default_value='csi_cam_0'),
        DeclareLaunchArgument('frame_id', default_value='/csi_cam_0_link'),
        DeclareLaunchArgument('sync_sink', default_value='false'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('fps', default_value='20'),
        DeclareLaunchArgument('flip_method', default_value='0'),
        DeclareLaunchArgument('load_camera_info', default_value='true'),
        DeclareLaunchArgument('camera_info_url', default_value=f'file://{cam_calib_path}'),

        Node(
            package='gscam',
            node_executable='gscam_node',  # In ROS 2, usually 'gscam_node'
            name=LaunchConfiguration('cam_name'),
            output='screen',
            parameters=[{
                'camera_name': LaunchConfiguration('cam_name'),
                'frame_id': LaunchConfiguration('frame_id'),
                'sync_sink': LaunchConfiguration('sync_sink'),
                'camera_id': LaunchConfiguration('sensor_id'),
                'image_width': LaunchConfiguration('width'),
                'image_height': LaunchConfiguration('height'),
                'target_fps': LaunchConfiguration('fps'),
                'camera_info_url': LaunchConfiguration('camera_info_url'),
                'gscam_config': (
                    'nvarguscamerasrc sensor-id=' + LaunchConfiguration('sensor_id').perform({}) +
                    ' ! video/x-raw(memory:NVMM), width=(int)' + LaunchConfiguration('width').perform({}) +
                    ', height=(int)' + LaunchConfiguration('height').perform({}) +
                    ', format=(string)NV12, framerate=(fraction)' + LaunchConfiguration('fps').perform({}) +
                    '/1 ! nvvidconv flip-method=' + LaunchConfiguration('flip_method').perform({}) +
                    ' ! videoconvert'
                )
            }],
            remappings=[
                ('camera/image_raw', [LaunchConfiguration('cam_name'), '/image_raw']),
                ('/set_camera_info', [LaunchConfiguration('cam_name'), '/set_camera_info'])
            ]
        )
    ])

