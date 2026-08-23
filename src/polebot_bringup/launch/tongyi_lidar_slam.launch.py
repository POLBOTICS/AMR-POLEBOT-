import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def generate_launch_description():
    pkg_polebot_amr_bringup = get_package_share_directory('polebot_amr_bringup')
    pkg_polebot_amr_description = get_package_share_directory('polebot_amr_description')
    pkg_tongyi = get_package_share_directory('tongyi_canopen_driver')

    teleop = LaunchConfiguration('teleop')
    use_rviz = LaunchConfiguration('use_rviz')

    xacro_file = os.path.join(
        pkg_polebot_amr_description,
        'src',
        'description',
        'polebot_amr_description.sdf'
    )
    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={'package_path': pkg_polebot_amr_description}
    ).toxml()

    rviz_config = os.path.join(
        pkg_polebot_amr_description,
        'rviz',
        'polebot_amr_nav.rviz'
    )

    slam_params_file = os.path.join(
        pkg_polebot_amr_bringup,
        'config',
        'polebot_amr_mapper_params.yaml'
    )

    tongyi_config_file = PathJoinSubstitution([
        FindPackageShare('tongyi_canopen_driver'),
        'config',
        'tongyi_direct_test.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'teleop',
            default_value='false',
            description='Launch keyboard teleop for manual /cmd_vel debugging.'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz for visualization.'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_config,
                'use_sim_time': False,
            }],
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),

        Node(
            package='tongyi_canopen_driver',
            executable='tongyi_canopen_node',
            name='tongyi_canopen_node',
            output='screen',
            parameters=[tongyi_config_file],
        ),

        Node(
            package='lsc_ros2_driver',
            executable='autonics_lsc_lidar',
            name='autonics_lidar',
            output='screen',
            parameters=[{
                'addr': '192.168.0.1',
                'port': 8000,
                'frame_id': 'lidar_link_corrected',
                'range_min': 0.05,
                'range_max': 25.0,
                'intensities': True,
                'rate': 5.0,
            }],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_to_corrected_tf',
            arguments=['0', '0', '0', '-1.5708', '0', '0', 'lidar_link', 'lidar_link_corrected'],
        ),

        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_file],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
        ),

        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            remappings=[('/cmd_vel', '/cmd_vel')],
            condition=IfCondition(teleop),
        ),
    ])
