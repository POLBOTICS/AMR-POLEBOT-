"""
polebot_slam_bringup.launch.py
Launch file specifically for SLAM operations.

This launches:
1. Robot State Publisher (URDF / TF tree)
2. Autonics LiDAR Driver (/scan)
3. SLAM Toolbox (Mapping)
4. RViz2 (Visualization)

Usage (after motor driver is running in another terminal):
  ros2 launch polebot_bringup polebot_slam_bringup.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock')

    # 1. Robot Description (URDF & TF Tree)
    # Provides static transforms like base_link -> laser
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('polebot_description'),
                'launch',
                'display.launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_gui': 'false', # Disable joint state publisher GUI
        }.items(),
    )

    # 2. SLAM Stack (includes LiDAR, SLAM Toolbox, and RViz)
    # We use the existing slam.launch.py from polebot_slam
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('polebot_slam'),
                'launch',
                'slam.launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'launch_lidar': 'true',
            'use_rviz': 'true',
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,
        
        # Bring up URDF and transforms immediately
        robot_description_launch,
        
        # Bring up SLAM stack after a short delay to ensure TF is ready
        TimerAction(period=1.0, actions=[slam_launch]),
    ])
