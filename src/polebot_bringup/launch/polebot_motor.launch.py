"""
polebot_motor.launch.py
Launch file for POLEBOT motor control via tongyi_canopen_driver.

Usage:
  ros2 launch polebot_bringup polebot_motor.launch.py
  ros2 launch polebot_bringup polebot_motor.launch.py can_interface:=can1
  ros2 launch polebot_bringup polebot_motor.launch.py auto_enable:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # =================== Arguments ===================
    can_interface  = LaunchConfiguration('can_interface')
    auto_enable    = LaunchConfiguration('auto_enable')
    use_sim_time   = LaunchConfiguration('use_sim_time')
    config_file    = LaunchConfiguration('config_file')

    declare_can_interface = DeclareLaunchArgument(
        'can_interface', default_value='can0',
        description='SocketCAN interface name (can0, can1, etc.)')

    declare_auto_enable = DeclareLaunchArgument(
        'auto_enable', default_value='false',
        description='Auto-enable drives on startup (UNSAFE — keep false for first test)')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock')

    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('polebot_bringup'),
            'config',
            'tongyi_canopen_params.yaml',
        ]),
        description='Path to tongyi_canopen_driver YAML config')

    # =================== CAN Interface Setup ===================
    # Automatically bring up the CAN interface before launching the node
    setup_can = ExecuteProcess(
        cmd=[
            'bash', '-c',
            [
                'sudo ip link set ', can_interface, ' down 2>/dev/null || true && ',
                'sudo ip link set ', can_interface, ' type can bitrate 500000 && ',
                'sudo ip link set ', can_interface, ' up && ',
                'echo "[polebot_motor] CAN interface ', can_interface, ' is UP at 500kbps"',
            ]
        ],
        output='screen',
    )

    # =================== TongYi CANopen Node ===================
    tongyi_node = Node(
        package='tongyi_canopen_driver',
        executable='tongyi_canopen_node',
        name='tongyi_canopen_node',
        namespace='polebot',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': use_sim_time,
                'can_interface': can_interface,
                'auto_enable': auto_enable,
            },
        ],
        remappings=[
            # Remap to POLEBOT standard topics
            ('/polebot/cmd_vel', '/cmd_vel'),
            ('/polebot/odom',    '/odom'),
        ],
    )

    # =================== Launch Description ===================
    return LaunchDescription([
        declare_can_interface,
        declare_auto_enable,
        declare_use_sim_time,
        declare_config_file,

        # Step 1: bring up CAN interface
        setup_can,

        # Step 2: launch driver node (1 second after CAN is up)
        TimerAction(period=1.0, actions=[tongyi_node]),
    ])
