#!/usr/bin/env python3
"""
POLEBOT SLAM Launch — Online Mapping Mode
Launches: Autonics LSC LiDAR + SLAM Toolbox (async) + optional RViz2

Usage:
  # Full SLAM with LiDAR + RViz (default)
  ros2 launch polebot_slam slam.launch.py

  # SLAM only — LiDAR already running separately
  ros2 launch polebot_slam slam.launch.py launch_lidar:=false

  # Localization mode (load existing map)
  ros2 launch polebot_slam slam.launch.py mode:=localization

  # Save map after mapping session:
  ros2 run nav2_map_server map_saver_cli -f ~/Desktop/AMR-POLEBOT-WS/maps/polman_lab
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # =================== Arguments ===================
    use_sim_time = LaunchConfiguration('use_sim_time')
    mode         = LaunchConfiguration('mode')
    launch_lidar = LaunchConfiguration('launch_lidar')
    use_rviz     = LaunchConfiguration('use_rviz')
    lidar_ip     = LaunchConfiguration('lidar_ip')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock')

    declare_mode = DeclareLaunchArgument(
        'mode', default_value='mapping',
        description='SLAM Toolbox mode: mapping | localization')

    declare_launch_lidar = DeclareLaunchArgument(
        'launch_lidar', default_value='true',
        description='Launch Autonics LSC LiDAR (false = already running)')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2 for map visualization')

    declare_lidar_ip = DeclareLaunchArgument(
        'lidar_ip', default_value='192.168.0.1',
        description='Autonics LSC IP address')

    # =================== LiDAR (Autonics LSC) ===================
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('polebot_sensors'),
                'launch',
                'lidar.launch.py',
            ])
        ]),
        launch_arguments={
            'lidar_ip': lidar_ip,
            'frame_id': 'laser',
        }.items(),
        condition=IfCondition(launch_lidar),
    )

    # =================== SLAM Toolbox ===================
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('polebot_slam'),
                'config',
                'slam_toolbox_params.yaml',
            ]),
            {
                'use_sim_time': use_sim_time,
                'mode': mode,
            },
        ],
    )

    # =================== RViz2 ===================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('polebot_slam'),
            'rviz',
            'slam_view.rviz',
        ])],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_mode,
        declare_launch_lidar,
        declare_use_rviz,
        declare_lidar_ip,

        # Step 1: Start LiDAR
        lidar_launch,

        # Step 2: Start SLAM Toolbox (1s delay — wait for /scan)
        TimerAction(period=1.0, actions=[slam_node]),

        # Step 3: Start RViz2
        TimerAction(period=2.0, actions=[rviz_node]),
    ])

