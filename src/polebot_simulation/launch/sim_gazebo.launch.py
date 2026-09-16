import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Package Directories
    pkg_sim = FindPackageShare('polebot_simulation')
    pkg_desc = FindPackageShare('polebot_description')
    pkg_slam = FindPackageShare('polebot_slam')
    pkg_web = FindPackageShare('polebot_web_interface')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Set Gazebo Resource Path for Meshes
    pkg_desc_share = get_package_share_directory('polebot_description')
    set_gz_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_desc_share, '..')
    )
    
    # 1. Start Robot State Publisher (URDF)
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_desc, 'launch', 'display.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time, 'use_gui': 'false', 'use_rviz': 'false'}.items()
    )

    # 2. Start Gazebo Harmonic
    world_file = PathJoinSubstitution([pkg_sim, 'worlds', 'warehouse.sdf'])
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': ['-r ', world_file]}.items()
    )

    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'warehouse',
            '-name', 'polebot',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.2'
        ],
        output='screen'
    )

    # 4. ROS-GZ Bridge
    bridge_params = PathJoinSubstitution([pkg_sim, 'config', 'ros_gz_bridge.yaml'])
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_params,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # 5. SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_slam, 'launch', 'slam.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'launch_lidar': 'false', # Lidar provided by gazebo
            'use_rviz': 'false'      # We use web interface
        }.items()
    )

    # 6. Web Interface
    web_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_web, 'launch', 'web_interface.launch.py']))
    )

    return LaunchDescription([
        # Important: Set use_sim_time globally
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time'),
        
        rsp_launch,
        set_gz_env,
        gazebo_launch,
        TimerAction(period=3.0, actions=[spawn_node]),
        bridge_node,
        
        # Start SLAM and Web UI after Gazebo is fully up
        TimerAction(period=8.0, actions=[slam_launch, web_launch]),
    ])
