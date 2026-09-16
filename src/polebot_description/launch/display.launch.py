from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_gui = LaunchConfiguration('use_gui', default='true')

    # Get URDF via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('polebot_description'),
            'urdf',
            'polebot.urdf.xacro',
        ]),
    ])

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # Joint state publisher GUI (for visualization only)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=__import__('launch.conditions', fromlist=['IfCondition']).IfCondition(use_gui),
    )

    # RViz
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('polebot_description'),
        'launch',
        'polebot_amr_nav.rviz',
    ])

    use_rviz = LaunchConfiguration('use_rviz', default='false')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=__import__('launch.conditions', fromlist=['IfCondition']).IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation clock'),
        DeclareLaunchArgument('use_gui', default_value='true',
                              description='Launch joint_state_publisher_gui'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='Launch RViz'),
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node,
    ])
