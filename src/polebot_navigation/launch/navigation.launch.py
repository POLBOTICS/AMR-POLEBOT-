import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    polebot_nav_dir = get_package_share_directory('polebot_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file', default=os.path.join(polebot_nav_dir, 'config', 'nav2_params.yaml'))
    lidar_ip = LaunchConfiguration('lidar_ip', default='192.168.0.1')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        description='Full path to map yaml file to load')

    # 1. Robot Description (URDF & TF Tree)
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

    # 2. LiDAR (Autonics LSC)
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
    )

    # 3. Nav2 Bringup
    # NOTE: Nav2 Jazzy bringup_launch.py uses PythonExpression() to evaluate
    # conditions such as `slam and use_localization`. These are evaluated as
    # raw Python expressions, so booleans MUST use Python-style capitalized
    # True/False (not 'true'/'false' which would cause NameError).
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'autostart': 'True',          # Must be capital — evaluated as Python
            'slam': 'False',              # Must be capital — evaluated as Python
            'use_localization': 'True',   # Must be capital — evaluated as Python
            'use_composition': 'False',   # Must be capital — evaluated as Python
            'params_file': params_file
        }.items()
    )
    # 4. Costmap Filters
    # For now, default to the workspace maps directory for masks
    default_maps_dir = '/home/mirae/Desktop/AMR-POLEBOT-WS/maps'
    costmap_filters_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(polebot_nav_dir, 'launch', 'costmap_filters.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'keepout_mask': os.path.join(default_maps_dir, 'keepout_mask.yaml'),
            'speed_mask': os.path.join(default_maps_dir, 'speed_mask.yaml')
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_map_yaml_cmd)
    
    # Add all essential components
    ld.add_action(robot_description_launch)
    ld.add_action(lidar_launch)
    
    # Start Nav2 after a short delay to ensure TF and Sensors are up
    ld.add_action(TimerAction(period=2.0, actions=[nav2_bringup_cmd]))
    ld.add_action(TimerAction(period=2.5, actions=[costmap_filters_cmd]))

    return ld
