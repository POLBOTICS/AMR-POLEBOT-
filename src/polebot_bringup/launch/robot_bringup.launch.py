import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    teleop_arg = DeclareLaunchArgument(
        'teleop', default_value='false',
        description='Set true untuk aktifkan keyboard teleop (hanya berlaku saat use_control=false)'
    )
    use_control_arg = DeclareLaunchArgument(
        'use_control', default_value='false',
        description=(
            'false = teleop langsung ke /cmd_vel (baseline mentah). '
            'true  = motion_profiler + gain_scheduling_pid otomatis generate /cmd_vel, teleop diabaikan.'
        )
    )
    teleop = LaunchConfiguration('teleop')
    use_control = LaunchConfiguration('use_control')

    baseline_condition = IfCondition(
        PythonExpression(["'", teleop, "' == 'true' and '", use_control, "' == 'false'"])
    )

    tongyi_launch = os.path.join(
        get_package_share_directory('tongyi_canopen_driver'),
        'launch', 'tongyi_bringup.launch.py'
    )
    motor_driver = TimerAction(
        period=1.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(tongyi_launch),
                launch_arguments={'teleop': 'false'}.items(),
            )
        ]
    )

    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        remappings=[('/cmd_vel', '/teleop_raw')],
        condition=baseline_condition,
    )

    baseline_relay = Node(
        package='topic_tools', executable='relay', name='teleop_passthrough',
        arguments=['/teleop_raw', '/cmd_vel'],
        condition=baseline_condition,
    )

    control_launch = os.path.join(
        get_package_share_directory('polebot_control'),
        'launch', 'control_container.launch.py'
    )
    control_container = TimerAction(
        period=1.5,  # start setelah driver siap
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(control_launch),
                condition=IfCondition(use_control),
            )
        ]
    )

    return LaunchDescription([
        teleop_arg, use_control_arg,
        motor_driver, teleop_node,
        baseline_relay,
        control_container,
    ])
