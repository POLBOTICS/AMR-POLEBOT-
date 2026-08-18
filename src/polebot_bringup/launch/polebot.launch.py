from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    GroupAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # =================== Launch Arguments ===================
    use_sim_time     = LaunchConfiguration('use_sim_time')
    use_rviz         = LaunchConfiguration('use_rviz')
    use_nav          = LaunchConfiguration('use_nav')
    use_slam         = LaunchConfiguration('use_slam')
    use_motor        = LaunchConfiguration('use_motor')
    robot_namespace  = LaunchConfiguration('robot_namespace')
    can_interface    = LaunchConfiguration('can_interface')

    declare_use_sim_time    = DeclareLaunchArgument('use_sim_time',    default_value='false',      description='Use simulation clock')
    declare_use_rviz        = DeclareLaunchArgument('use_rviz',        default_value='true',       description='Launch RViz2')
    declare_use_nav         = DeclareLaunchArgument('use_nav',         default_value='true',       description='Launch Nav2 navigation stack')
    declare_use_slam        = DeclareLaunchArgument('use_slam',        default_value='false',      description='Launch SLAM (mapping mode)')
    declare_use_motor       = DeclareLaunchArgument('use_motor',       default_value='true',       description='Launch TongYi CANopen motor driver')
    declare_can_interface   = DeclareLaunchArgument('can_interface',   default_value='can0',       description='SocketCAN interface (can0, can1, ...)')
    declare_robot_namespace = DeclareLaunchArgument('robot_namespace', default_value='polebot',    description='Robot namespace')

    # =================== Robot Description ==================
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
            'use_gui': 'false',
        }.items(),
    )

    # =================== Sensors ===========================
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('polebot_sensors'),
                'launch',
                'sensors.launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # =================== SLAM (mapping mode) ================
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('polebot_slam'),
                'launch',
                'slam.launch.py',
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(use_slam),
    )

    # =================== Navigation ========================
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('polebot_navigation'),
                'launch',
                'navigation.launch.py',
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(use_nav),
    )



    # =================== Motor Driver (TongYi CANopen) ======
    motor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('polebot_bringup'),
                'launch',
                'polebot_motor.launch.py',
            ])
        ]),
        launch_arguments={
            'can_interface': can_interface,
            'use_sim_time':  use_sim_time,
            'auto_enable':   'false',
        }.items(),
        condition=IfCondition(use_motor),
    )



    return LaunchDescription([
        # Declare arguments
        declare_use_sim_time,
        declare_use_rviz,
        declare_use_nav,
        declare_use_slam,
        declare_use_motor,
        declare_can_interface,
        declare_robot_namespace,

        # Bring up robot description
        robot_description_launch,

        # Launch motor driver (CAN setup + node)
        TimerAction(period=1.0, actions=[motor_launch]),

        # Launch sensors
        TimerAction(period=2.0, actions=[sensors_launch]),

        # Launch navigation or SLAM
        TimerAction(period=4.0, actions=[slam_launch, nav_launch]),


    ])
