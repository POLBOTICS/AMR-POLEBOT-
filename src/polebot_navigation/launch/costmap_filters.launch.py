import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the launch directory
    polebot_nav_dir = get_package_share_directory('polebot_navigation')

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    keepout_mask_yaml_file = LaunchConfiguration('keepout_mask')
    speed_mask_yaml_file = LaunchConfiguration('speed_mask')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(polebot_nav_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_keepout_mask_yaml_file_cmd = DeclareLaunchArgument(
        'keepout_mask',
        description='Full path to keepout filter mask yaml file to load')

    declare_speed_mask_yaml_file_cmd = DeclareLaunchArgument(
        'speed_mask',
        description='Full path to speed filter mask yaml file to load')

    # Nodes
    # Keepout Filter Mask Server
    keepout_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='keepout_mask_server',
        output='screen',
        emulate_tty=True,
        parameters=[params_file,
                    {'yaml_filename': keepout_mask_yaml_file,
                     'topic_name': '/keepout_filter_mask'}])

    # Keepout Filter Info Server
    keepout_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='keepout_filter_info_server',
        output='screen',
        emulate_tty=True,
        parameters=[params_file])

    # Speed Filter Mask Server
    speed_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='speed_mask_server',
        output='screen',
        emulate_tty=True,
        parameters=[params_file,
                    {'yaml_filename': speed_mask_yaml_file,
                     'topic_name': '/speed_filter_mask'}]) 

    # Speed Filter Info Server
    speed_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='speed_filter_info_server',
        output='screen',
        emulate_tty=True,
        parameters=[params_file])

    # Lifecycle Manager
    lifecycle_manager_filters = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_filters',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['keepout_mask_server', 
                                    'keepout_filter_info_server',
                                    'speed_mask_server',
                                    'speed_filter_info_server']}])

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_keepout_mask_yaml_file_cmd)
    ld.add_action(declare_speed_mask_yaml_file_cmd)

    ld.add_action(keepout_mask_server)
    ld.add_action(keepout_filter_info_server)
    ld.add_action(speed_mask_server)
    ld.add_action(speed_filter_info_server)
    ld.add_action(lifecycle_manager_filters)

    return ld
