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
    use_dashboard    = LaunchConfiguration('use_dashboard')
    robot_namespace  = LaunchConfiguration('robot_namespace')

    declare_use_sim_time    = DeclareLaunchArgument('use_sim_time',    default_value='false',      description='Use simulation clock')
    declare_use_rviz        = DeclareLaunchArgument('use_rviz',        default_value='true',       description='Launch RViz2')
    declare_use_nav         = DeclareLaunchArgument('use_nav',         default_value='true',       description='Launch Nav2 navigation stack')
    declare_use_slam        = DeclareLaunchArgument('use_slam',        default_value='false',      description='Launch SLAM (mapping mode)')
    declare_use_dashboard   = DeclareLaunchArgument('use_dashboard',   default_value='true',       description='Launch web dashboard')
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

    # =================== Dashboard =========================
    dashboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('polebot_dashboard'),
                'launch',
                'dashboard.launch.py',
            ])
        ]),
        condition=IfCondition(use_dashboard),
    )

    # =================== Robot Status Publisher ============
    robot_status_node = Node(
        package='polebot_control',
        executable='robot_status_publisher',
        name='robot_status_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        # Declare arguments
        declare_use_sim_time,
        declare_use_rviz,
        declare_use_nav,
        declare_use_slam,
        declare_use_dashboard,
        declare_robot_namespace,

        # Bring up robot
        robot_description_launch,

        # Launch sensors with short delay
        TimerAction(period=2.0, actions=[sensors_launch]),

        # Launch navigation or SLAM
        TimerAction(period=4.0, actions=[slam_launch, nav_launch]),

        # Launch dashboard
        TimerAction(period=3.0, actions=[dashboard_launch]),

        # Status publisher
        TimerAction(period=2.0, actions=[robot_status_node]),
    ])
