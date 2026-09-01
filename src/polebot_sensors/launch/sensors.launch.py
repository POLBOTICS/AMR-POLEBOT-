#!/usr/bin/env python3
"""
POLEBOT Sensors Launch
Brings up LiDAR, IMU, and Camera drivers.

NOTE: Odometry and TF (odom → base_link) are handled by
      tongyi_canopen_driver — do NOT launch a separate odometry
      node from this file to avoid conflicts.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_ip     = LaunchConfiguration('lidar_ip')

    # =================== LiDAR ===========================
    # Autonics LSC Series — Ethernet (192.168.0.1:8000)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('polebot_sensors'),
                'launch',
                'lidar.launch.py',
            ])
        ]),
        launch_arguments={
            'lidar_ip':  lidar_ip,
            'frame_id':  'laser',
        }.items(),
    )

    # =================== IMU =============================
    imu_node = Node(
        package='imu_tools',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu',
            'gain': 0.1,
            'zeta': 0.0,
            'fixed_frame': 'base_link',
        }],
        remappings=[
            ('/imu/data_raw', '/imu/data_raw'),
            ('/imu/data', '/imu/data'),
        ],
    )

    # =================== Camera ==========================
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [640, 480],
            'camera_frame_id': 'camera_optical_frame',
            'pixel_format': 'YUYV',
        }],
        remappings=[
            ('/image_raw', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('lidar_ip',    default_value='192.168.0.1',
                              description='Autonics LSC IP address'),
        lidar_launch,
        imu_node,
        camera_node,
    ])
