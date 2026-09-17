"""Launch file untuk Tes 1: Uji Motion Control.

Menguji kemampuan kendali melacak lintasan geometri acuan tanpa peta/rintangan.
Mendukung kendali PID (DiffDrive) maupun SMC (Effort Control) pada konfigurasi
robot solo, trolley_fixed, maupun trolley_pivot.

Cara pakai:
  # Uji PID pada jalur S-Curve:
  ros2 launch polebot_control sim_motion_control.launch.py controller:=pid trajectory_mode:=s_curve robot_config:=solo

  # Uji SMC pada jalur U-Curve dengan trolley pivot:
  ros2 launch polebot_control sim_motion_control.launch.py controller:=smc trajectory_mode:=u_curve robot_config:=trolley_pivot
"""
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_config = LaunchConfiguration('robot_config').perform(context)
    controller_type = LaunchConfiguration('controller').perform(context)
    trajectory_mode = LaunchConfiguration('trajectory_mode').perform(context)

    config_map = {
        'solo':          {'include_trolley': 'false', 'hitch_type': 'revolute'},
        'trolley_fixed': {'include_trolley': 'true',  'hitch_type': 'fixed'},
        'trolley_pivot': {'include_trolley': 'true',  'hitch_type': 'revolute'},
    }
    if robot_config not in config_map:
        raise ValueError(f"robot_config='{robot_config}' tidak dikenal. Pilihan: {list(config_map.keys())}")
    sel = config_map[robot_config]

    act_mode = 'effort_control' if controller_type == 'smc' else 'diffdrive'

    polebot_control = get_package_share_directory("polebot_control")
    polebot_desc = get_package_share_directory("polebot_amr_description")
    ros_gz_sim = get_package_share_directory("ros_gz_sim")

    world_path = os.path.join(polebot_control, "worlds", "straight_track.world.sdf")
    sdf_path = os.path.join(polebot_desc, "src", "description", "polebot_amr_description.sdf")
    controllers_yaml = os.path.join(polebot_control, "config", "ros2_controllers.yaml")

    robot_desc_xml = xacro.process_file(
        sdf_path,
        mappings={
            'package_path': polebot_desc,
            'mapping_mode': 'false',
            'actuation_mode': act_mode,
            'controllers_yaml': controllers_yaml,
            **sel,
        }
    ).toxml()

    temp_sdf = f"/tmp/polebot_amr_motion_{act_mode}.sdf"
    with open(temp_sdf, 'w') as f:
        f.write(robot_desc_xml)

    rsp_node = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen', parameters=[{'robot_description': robot_desc_xml, 'use_sim_time': True}]
    )

    gz_sim_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r {world_path}"}.items(),
    )

    spawn_yaw = "0.785398" if trajectory_mode == 'figure_8' else "0.0"
    spawn_node = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-file", temp_sdf, "-name", "polebot_amr", "-x", "0.0", "-y", "0.0", "-z", "0.38", "-Y", spawn_yaw],
    )

    # Base bridge topics
    bridge_args = [
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
    ]
    if controller_type == 'pid':
        bridge_args.extend([
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        ])

    bridge_node = Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen",
        arguments=bridge_args,
    )

    rviz2_node = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', os.path.join(polebot_control, 'config', 'polebot.rviz')],
        parameters=[{'use_sim_time': True}]
    )

    # Geometric Trajectory Generator
    path_profile_node = Node(
        package="polebot_control", executable="path_profile_node", output="screen",
        parameters=[
            {"use_sim_time": True, "publish_rate": 30.0, "frame_id": "odom", "trajectory_mode": trajectory_mode},
            {"distance": 3.0, "straight_1": 2.5, "straight_2": 2.0, "straight_3": 2.5},
            {"turn_radius_1": 1.2, "turn_radius_2": 1.0, "turn_angle_1_deg": 90.0, "turn_angle_2_deg": 90.0},
            {"turn_dir_1": "right", "turn_dir_2": "left", "v_max": 0.25, "a_max": 0.08},
            {"repeat": False, "hold_time": 1.0, "samples_per_meter": 40},
        ],
    )

    # Controller-specific Nodes
    controller_nodes = []
    if controller_type == 'smc':
        controller_nodes.extend([
            Node(package='controller_manager', executable='spawner',
                 arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'], output='screen'),
            Node(package='controller_manager', executable='spawner',
                 arguments=['joint_group_effort_controller', '--param-file', controllers_yaml,
                            '--controller-manager', '/controller_manager'], output='screen'),
            Node(package='polebot_control', executable='wheel_odom_publisher', output='screen',
                 parameters=[{'use_sim_time': True}]),
            Node(package='polebot_control', executable='odom_to_tf', output='screen',
                 parameters=[{'use_sim_time': True}]),
            Node(package='polebot_control', executable='sliding_mode_controller', output='screen',
                 parameters=[{'use_sim_time': True, 'default_mode': 2 if robot_config == 'trolley_pivot' else (1 if robot_config == 'trolley_fixed' else 0)}]),
        ])
    else:
        controller_nodes.extend([
            Node(package="polebot_control", executable="odom_to_posearray_node", output="screen",
                 parameters=[{"use_sim_time": True, "odom_topic": "/odom", "posearray_topic": "/model/ddr/pose", "frame_id": "odom"}]),
            Node(package="polebot_control", executable="pitdt_profiled_pure_pursuit_controller_node", output="screen",
                 parameters=[
                     {"use_sim_time": True, "pose_topic": "/model/ddr/pose",
                      "reference_path_topic": "/reference_path", "cmd_vel_topic": "/cmd_vel"},
                     {"control_rate": 40.0, "lookahead_distance": 0.25, "stop_distance": 0.05, "finish_distance": 0.08},
                     {"max_v": 0.30, "max_w": 0.60},
                     {"motion_profile_type": "s_curve", "max_jerk": 0.16,
                      "profile_v_max": 0.1727, "profile_a_max": 0.08, "profile_d_max": 0.10,
                      "approach_distance": 0.20, "approach_speed": 0.05,
                      "profile_min_v": 0.025, "profile_min_v_disable_distance": 0.20,
                      "max_lateral_accel": 0.25, "profile_alpha_max": 0.70},
                     {"use_gain_scheduling": True,
                      "kp_accel": 2.8, "ki_accel": 0.00, "kd_accel": 0.08,
                      "kp_cruise": 3.2, "ki_cruise": 0.00, "kd_cruise": 0.10,
                      "kp_decel": 3.0, "ki_decel": 0.00, "kd_decel": 0.12,
                      "kp_approach": 1.6, "ki_approach": 0.00, "kd_approach": 0.05},
                     {"kp_lin": 0.35, "ki_lin": 0.00, "kd_lin": 0.02, "u0_lin": 0.00, "ramp_lin": 0.25, "v_fb_max": 0.08, "i_lin_limit": 1.0},
                     {"kp_ang": 0.65, "ki_ang": 0.00, "kd_ang": 0.02, "u0_ang": 0.03, "ramp_ang": 0.30, "w_fb_max": 0.30, "i_ang_limit": 1.0},
                     {"reacquire_distance": 0.25, "reacquire_heading_deg": 40.0, "reacquire_v_limit": 0.12, "reacquire_min_v": 0.04, "reacquire_w_limit": 0.60},
                     {"print_debug": False},
                 ]),
        ])

    static_map_to_odom = Node(
        package='tf2_ros', executable='static_transform_publisher', output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    return [
        static_map_to_odom,
        rsp_node, gz_sim_node, bridge_node,
        TimerAction(period=3.0, actions=[spawn_node, rviz2_node]),
        TimerAction(period=6.0, actions=[path_profile_node]),
        TimerAction(period=8.0, actions=controller_nodes),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='pid', description='Pilihan kendali: pid | smc'),
        DeclareLaunchArgument('trajectory_mode', default_value='s_curve',
                              description='straight | l_left | l_right | arc_left | arc_right | s_curve | complex_course | u_curve | figure_8'),
        DeclareLaunchArgument('robot_config', default_value='solo',
                              description='solo | trolley_fixed | trolley_pivot'),
        OpaqueFunction(function=launch_setup),
    ])
