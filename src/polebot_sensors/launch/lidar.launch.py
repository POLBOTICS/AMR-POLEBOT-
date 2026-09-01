#!/usr/bin/env python3
"""
POLEBOT LiDAR Launch — Autonics LSC Series (Ethernet)
Standalone launch untuk test LiDAR tanpa sistem lengkap.

Usage:
  ros2 launch polebot_sensors lidar.launch.py
  ros2 launch polebot_sensors lidar.launch.py lidar_ip:=192.168.0.1
  ros2 launch polebot_sensors lidar.launch.py rviz:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    lidar_ip    = LaunchConfiguration('lidar_ip')
    lidar_port  = LaunchConfiguration('lidar_port')
    frame_id    = LaunchConfiguration('frame_id')
    use_rviz    = LaunchConfiguration('rviz')

    declare_lidar_ip   = DeclareLaunchArgument(
        'lidar_ip', default_value='192.168.0.1',
        description='Autonics LSC IP address')

    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port', default_value='8000',
        description='Autonics LSC port number')

    declare_frame_id   = DeclareLaunchArgument(
        'frame_id', default_value='laser',
        description='LiDAR frame ID (must match URDF)')

    declare_rviz       = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz2 for scan visualization')

    # ── Autonics LSC Node ──────────────────────────────────────────────────────
    lsc_node = Node(
        package='lsc_ros2_driver',
        executable='autonics_lsc_lidar',
        name='lsc_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('polebot_sensors'),
                'config',
                'lsc_lidar_params.yaml',
            ]),
            {
                'addr':     lidar_ip,
                'port':     lidar_port,
                'frame_id': frame_id,
            },
        ],
        remappings=[
            ('scan', '/scan'),
        ],
    )

    # ── RViz2 (optional, for debug) ────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('polebot_sensors'),
            'rviz',
            'lidar_view.rviz',
        ])],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        declare_lidar_ip,
        declare_lidar_port,
        declare_frame_id,
        declare_rviz,
        lsc_node,
        rviz_node,
    ])
