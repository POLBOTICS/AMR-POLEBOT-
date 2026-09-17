"""Launch file untuk Tes 2: Uji Path Planning berbasis peta (BFS).

Mendukung:
1. Mode Gazebo Fisika (use_gazebo:=true, default):
   - Simulasi fisika lengkap di Gazebo Sim + ROS 2 Control (SMC / PID) + RViz.
2. Mode Simulator Kinematis Mandiri di RViz (use_gazebo:=false):
   - Tanpa Gazebo, startup instan (< 1 detik), sangat ringan & stabil.
   - Kinematika AMR DiffDrive dan trailer (Fixed/Pivot) disimulasikan mandiri.
   - Mendukung kendali SMC, PID, atau auto_follow:=true (direct pure pursuit).
   - Mendukung reposisi robot di RViz via alat 2D Pose Estimate (/initialpose).

Cara pakai:
  # 1. Mode RViz Kinematis Mandiri (Cepat & Ringan):
  ros2 launch polebot_control sim_path_planning.launch.py use_gazebo:=false robot_config:=trolley_pivot

  # 2. Mode RViz Kinematis dengan Auto-Follow Jalur Langsung:
  ros2 launch polebot_control sim_path_planning.launch.py use_gazebo:=false robot_config:=trolley_pivot auto_follow:=true

  # 3. Mode Gazebo Fisika Penuh:
  ros2 launch polebot_control sim_path_planning.launch.py use_gazebo:=true controller:=smc robot_config:=trolley_pivot
"""
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_config = LaunchConfiguration('robot_config').perform(context)
    controller_type = LaunchConfiguration('controller').perform(context)
    world_choice = LaunchConfiguration('world').perform(context)
    map_mode = LaunchConfiguration('map_mode').perform(context)
    use_gazebo = LaunchConfiguration('use_gazebo').perform(context).lower() == 'true'
    auto_follow = LaunchConfiguration('auto_follow').perform(context).lower() == 'true'
    teleop_opt = LaunchConfiguration('teleop').perform(context).lower() == 'true'

    if teleop_opt and controller_type == 'smc':
        controller_type = 'teleop'

    use_sim_time = use_gazebo  # True jika memakai Gazebo clock, False jika simulator mandiri (wall-clock)

    config_map = {
        'solo':          {'include_trolley': 'false', 'hitch_type': 'revolute'},
        'trolley_fixed': {'include_trolley': 'true',  'hitch_type': 'fixed'},
        'trolley_pivot': {'include_trolley': 'true',  'hitch_type': 'revolute'},
    }
    if robot_config not in config_map:
        raise ValueError(f"robot_config='{robot_config}' tidak dikenal. Pilihan: {list(config_map.keys())}")
    sel = config_map[robot_config]
    if robot_config == 'trolley_fixed':
        joint_type_val = 'fixed'
        mode_int = 1
    elif robot_config == 'trolley_pivot':
        joint_type_val = 'pivot'
        mode_int = 2
    else:
        joint_type_val = 'solo'
        mode_int = 0

    act_mode = 'diffdrive' if controller_type in ('pid', 'teleop') else 'effort_control'

    polebot_control = get_package_share_directory("polebot_control")
    polebot_desc = get_package_share_directory("polebot_amr_description")
    ros_gz_sim = get_package_share_directory("ros_gz_sim")
    sdf_path = os.path.join(polebot_desc, "src", "description", "polebot_amr_description.sdf")

    if world_choice == 'factory':
        world_path = os.path.join(polebot_desc, "world", "factory_warehouse.world.sdf")
        map_yaml = os.path.join(polebot_control, "config", "factory_map.yaml")
        spawn_x, spawn_y, spawn_z, spawn_yaw = "-7.0", "-5.5", "0.38", "0.0"
    else:
        world_path = os.path.join(polebot_desc, "world", "my_world.sdf")
        map_yaml = os.path.join(polebot_control, "config", "test_map.yaml")
        spawn_x, spawn_y, spawn_z, spawn_yaw = "0.0", "0.0", "0.38", "0.0"

    custom_map = LaunchConfiguration('map').perform(context).strip()
    if custom_map:
        if os.path.isabs(custom_map):
            map_yaml = custom_map
        else:
            map_yaml = os.path.join(polebot_control, "config", custom_map)

    controllers_yaml = os.path.join(polebot_control, "config", "ros2_controllers.yaml")
    slam_params = os.path.join(polebot_control, "config", "mapper_params.yaml")
    amcl_params = os.path.join(polebot_control, "config", "amcl_params.yaml")

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

    rsp_node = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen', parameters=[{'robot_description': robot_desc_xml, 'use_sim_time': use_sim_time}]
    )

    rviz2_node = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', os.path.join(polebot_control, 'config', 'polebot.rviz')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ═════════════════════════════════════════════════════════════════════════
    # CABANG A: MODE SIMULATOR KINEMATIS MANDIRI DI RVIZ (use_gazebo == False)
    # ═════════════════════════════════════════════════════════════════════════
    if not use_gazebo:
        kin_sim_node = Node(
            package='polebot_control', executable='kinematic_simulator', name='kinematic_simulator',
            output='screen', parameters=[{
                'use_sim_time': False,
                'spawn_x': float(spawn_x),
                'spawn_y': float(spawn_y),
                'spawn_yaw': float(spawn_yaw),
                'default_mode': mode_int,
                'auto_follow': auto_follow,
                'target_speed': 0.35,
                'control_rate': 50.0,
            }]
        )

        map_server_node = Node(
            package='nav2_map_server', executable='map_server', name='map_server', output='screen',
            parameters=[{'yaml_filename': map_yaml, 'use_sim_time': False, 'topic_name': 'map', 'frame_id': 'map'}],
        )
        lifecycle_manager_node = Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
            output='screen', parameters=[{'use_sim_time': False, 'autostart': True, 'node_names': ['map_server']}],
        )

        mode_selector = Node(
            package="polebot_control", executable="trajectory_mode_selector", name="trajectory_mode_selector",
            output="screen", parameters=[{"use_sim_time": False, "joint_type": joint_type_val}]
        )
        bfs_planner_node = Node(
            package="polebot_control", executable="bfs_planner", name="bfs_planner",
            output="screen", parameters=[{"use_sim_time": False}]
        )
        planning_action = TimerAction(period=2.0, actions=[mode_selector, bfs_planner_node])

        controller_actions = []
        if not auto_follow:
            if controller_type == 'smc':
                smc_node = Node(
                    package='polebot_control', executable='sliding_mode_controller', name='sliding_mode_controller',
                    output='screen', parameters=[{'use_sim_time': False, 'default_mode': mode_int}],
                )
                controller_actions.append(TimerAction(period=2.5, actions=[smc_node]))
            elif controller_type == 'pid':
                odom_to_posearray = Node(
                    package="polebot_control", executable="odom_to_posearray_node", output="screen",
                    parameters=[{"use_sim_time": False, "odom_topic": "/odom", "posearray_topic": "/model/ddr/pose", "frame_id": "odom"}],
                )
                pid_node = Node(
                    package="polebot_control", executable="pitdt_profiled_pure_pursuit_controller_node", output="screen",
                    parameters=[
                        {"use_sim_time": False, "pose_topic": "/model/ddr/pose",
                         "reference_path_topic": "/planned_path", "cmd_vel_topic": "/cmd_vel"},
                        {"control_rate": 40.0, "lookahead_distance": 0.50, "stop_distance": 0.15, "finish_distance": 0.20},
                        {"max_v": 0.50, "max_w": 1.00},
                        {"profile_v_max": 0.40, "profile_a_max": 0.50, "profile_d_max": 0.50,
                         "profile_min_v": 0.15, "profile_min_v_disable_distance": 0.20,
                         "max_lateral_accel": 0.20, "profile_alpha_max": 1.0},
                        {"kp_lin": 0.35, "ki_lin": 0.00, "kd_lin": 0.02, "u0_lin": 0.00, "ramp_lin": 0.25, "v_fb_max": 0.00, "i_lin_limit": 1.0},
                        {"kp_ang": 0.65, "ki_ang": 0.00, "kd_ang": 0.02, "u0_ang": 0.03, "ramp_ang": 0.30, "w_fb_max": 0.50, "i_ang_limit": 1.0},
                        {"reacquire_distance": 0.25, "reacquire_heading_deg": 40.0, "reacquire_v_limit": 0.12, "reacquire_min_v": 0.10, "reacquire_w_limit": 0.60},
                        {"print_debug": False},
                    ],
                )
                controller_actions.append(TimerAction(period=2.5, actions=[odom_to_posearray, pid_node]))
            elif controller_type == 'teleop':
                pass

        teleop_actions = []
        if teleop_opt:
            teleop_node = Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard',
                name='teleop_twist_keyboard',
                output='screen',
                prefix='xterm -e',
                parameters=[{'use_sim_time': False}],
            )
            teleop_actions.append(TimerAction(period=3.0, actions=[teleop_node]))

        planning_actions = [planning_action] if controller_type != 'teleop' else []

        return [
            rsp_node,
            rviz2_node,
            kin_sim_node,
            map_server_node,
            lifecycle_manager_node,
            *planning_actions,
            *controller_actions,
            *teleop_actions,
        ]

    # ═════════════════════════════════════════════════════════════════════════
    # CABANG B: MODE SIMULASI FISIKA LENGKAP GAZEBO (use_gazebo == True)
    # ═════════════════════════════════════════════════════════════════════════
    temp_sdf = f"/tmp/polebot_amr_planning_{act_mode}.sdf"
    with open(temp_sdf, 'w') as f:
        f.write(robot_desc_xml)

    gz_sim_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r {world_path}"}.items(),
    )

    spawn_node = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-file", temp_sdf, "-name", "polebot_amr",
                   "-x", spawn_x, "-y", spawn_y, "-z", spawn_z, "-Y", spawn_yaw],
    )

    bridge_args = [
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "/model/polebot_amr/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
    ]
    if controller_type in ('pid', 'teleop'):
        bridge_args.extend([
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        ])

    bridge_node = Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen",
        arguments=bridge_args,
    )

    init_x = float(spawn_x)
    init_y = float(spawn_y)
    init_yaw = float(spawn_yaw)

    try:
        if os.path.exists(map_yaml):
            import yaml
            with open(map_yaml, 'r') as f:
                map_meta = yaml.safe_load(f)
            orig = map_meta.get('origin', [0.0, 0.0, 0.0])
            if init_x < orig[0] or init_y < orig[1]:
                init_x = 0.0
                init_y = 0.0
                init_yaw = 0.0
    except Exception:
        pass

    map_actions = []
    if map_mode == 'amcl':
        map_server_node = Node(
            package='nav2_map_server', executable='map_server', name='map_server', output='screen',
            parameters=[{'yaml_filename': map_yaml, 'use_sim_time': use_sim_time, 'topic_name': 'map', 'frame_id': 'map'}],
        )
        amcl_node = Node(
            package='nav2_amcl', executable='amcl', name='amcl', output='screen',
            parameters=[
                amcl_params,
                {
                    'use_sim_time': use_sim_time,
                    'initial_pose.x': init_x,
                    'initial_pose.y': init_y,
                    'initial_pose.z': 0.0,
                    'initial_pose.yaw': init_yaw,
                }
            ],
        )
        lifecycle_manager_node = Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization',
            output='screen', parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'node_names': ['map_server', 'amcl']}],
        )
        map_actions.extend([
            TimerAction(period=5.0, actions=[map_server_node]),
            TimerAction(period=6.0, actions=[amcl_node]),
            TimerAction(period=7.0, actions=[lifecycle_manager_node]),
        ])
    elif map_mode == 'slam':
        slam_node = Node(
            package='slam_toolbox', executable='async_slam_toolbox_node',
            name='slam_toolbox', output='screen',
            parameters=[slam_params, {'use_sim_time': use_sim_time}]
        )
        slam_configure = TimerAction(period=10.0, actions=[
            ExecuteProcess(cmd=['ros2', 'lifecycle', 'set', 'slam_toolbox', 'configure'], output='screen'),
        ])
        slam_activate = TimerAction(period=12.0, actions=[
            ExecuteProcess(cmd=['ros2', 'lifecycle', 'set', 'slam_toolbox', 'activate'], output='screen'),
        ])
        map_actions.extend([
            TimerAction(period=6.0, actions=[slam_node]),
            slam_configure,
            slam_activate,
        ])
    else:  # 'static' — open-loop static transform publisher (fallback legacy)
        map_server_node = Node(
            package='nav2_map_server', executable='map_server', name='map_server', output='screen',
            parameters=[{'yaml_filename': map_yaml, 'use_sim_time': use_sim_time, 'topic_name': 'map', 'frame_id': 'map'}],
        )
        lifecycle_manager_node = Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
            output='screen', parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'node_names': ['map_server']}],
        )
        static_map_to_odom = Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=[spawn_x, spawn_y, '0', '0', '0', spawn_yaw, 'map', 'odom'],
        )
        map_actions.extend([
            static_map_to_odom,
            TimerAction(period=5.0, actions=[map_server_node]),
            TimerAction(period=7.0, actions=[lifecycle_manager_node]),
        ])

    mode_selector = Node(
        package="polebot_control", executable="trajectory_mode_selector", name="trajectory_mode_selector",
        output="screen", parameters=[{"use_sim_time": True, "joint_type": joint_type_val}]
    )
    bfs_planner_node = Node(
        package="polebot_control", executable="bfs_planner", name="bfs_planner",
        output="screen", parameters=[{"use_sim_time": True}]
    )
    planning_nodes = TimerAction(period=15.0, actions=[mode_selector, bfs_planner_node])

    odom_nodes = [
        Node(package='polebot_control', executable='wheel_odom_publisher', name='wheel_odom_publisher',
             output='screen', parameters=[{
                 'use_sim_time': True,
                 'spawn_x': float(spawn_x),
                 'spawn_y': float(spawn_y),
                 'spawn_yaw': float(spawn_yaw),
                 'use_ground_truth': False,
                 'gazebo_odom_topic': '/model/polebot_amr/odometry',
             }]),
        Node(package='polebot_control', executable='odom_to_tf', name='odom_to_tf',
             output='screen', parameters=[{'use_sim_time': True}]),
    ]

    controller_nodes = []
    if controller_type == 'smc':
        controller_nodes.extend([
            *odom_nodes,
            Node(package='controller_manager', executable='spawner',
                 arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'], output='screen'),
            Node(package='controller_manager', executable='spawner',
                 arguments=['joint_group_effort_controller', '--param-file', controllers_yaml,
                            '--controller-manager', '/controller_manager'], output='screen'),
            Node(package='polebot_control', executable='sliding_mode_controller', name='sliding_mode_controller',
                 output='screen', parameters=[{'use_sim_time': True, 'default_mode': mode_int}]),
        ])
    elif controller_type == 'pid':
        controller_nodes.extend([
            *odom_nodes,
            Node(package="polebot_control", executable="odom_to_posearray_node", output="screen",
                 parameters=[{"use_sim_time": True, "odom_topic": "/odom", "posearray_topic": "/model/ddr/pose", "frame_id": "odom"}]),
            Node(package="polebot_control", executable="pitdt_profiled_pure_pursuit_controller_node", output="screen",
                 parameters=[
                     {"use_sim_time": True, "pose_topic": "/model/ddr/pose",
                      "reference_path_topic": "/planned_path", "cmd_vel_topic": "/cmd_vel"},
                     {"control_rate": 40.0, "lookahead_distance": 0.50, "stop_distance": 0.15, "finish_distance": 0.20},
                     {"max_v": 0.50, "max_w": 1.00},
                     {"profile_v_max": 0.40, "profile_a_max": 0.50, "profile_d_max": 0.50,
                      "profile_min_v": 0.15, "profile_min_v_disable_distance": 0.20,
                      "max_lateral_accel": 0.20, "profile_alpha_max": 1.0},
                     {"kp_lin": 0.35, "ki_lin": 0.00, "kd_lin": 0.02, "u0_lin": 0.00, "ramp_lin": 0.25, "v_fb_max": 0.00, "i_lin_limit": 1.0},
                     {"kp_ang": 0.65, "ki_ang": 0.00, "kd_ang": 0.02, "u0_ang": 0.03, "ramp_ang": 0.30, "w_fb_max": 0.50, "i_ang_limit": 1.0},
                     {"reacquire_distance": 0.25, "reacquire_heading_deg": 40.0, "reacquire_v_limit": 0.12, "reacquire_min_v": 0.10, "reacquire_w_limit": 0.60},
                     {"print_debug": False},
                 ]),
        ])
    elif controller_type == 'teleop':
        controller_nodes.extend([
            *odom_nodes,
        ])
    controller_action = TimerAction(period=17.0, actions=controller_nodes)

    planning_actions = [planning_nodes] if controller_type != 'teleop' else []

    teleop_actions = []
    if teleop_opt:
        teleop_node = Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e',
            parameters=[{'use_sim_time': use_sim_time}],
        )
        teleop_actions.append(TimerAction(period=8.0, actions=[teleop_node]))

    return [
        rsp_node, gz_sim_node, bridge_node,
        TimerAction(period=3.0, actions=[spawn_node]),
        TimerAction(period=5.0, actions=[rviz2_node]),
        *map_actions,
        *planning_actions,
        controller_action,
        *teleop_actions,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_gazebo', default_value='true', description='true: jalankan Gazebo; false: simulator kinematis mandiri di RViz'),
        DeclareLaunchArgument('auto_follow', default_value='false', description='true: simulator kinematis melacak jalur secara mandiri (direct pure pursuit); false: gunakan controller'),
        DeclareLaunchArgument('controller', default_value='smc', description='Pilihan kendali: smc | pid | teleop'),
        DeclareLaunchArgument('teleop', default_value='false', description='true: otomatis buka jendela xterm keyboard teleop; false: manual lewat terminal lain'),
        DeclareLaunchArgument('robot_config', default_value='trolley_pivot', description='solo | trolley_fixed | trolley_pivot'),
        DeclareLaunchArgument('world', default_value='factory', description='Pilihan dunia: factory (pabrik/gudang industri luas) | my_world'),
        DeclareLaunchArgument('map_mode', default_value='amcl', description='Pilihan peta/lokalisasi: amcl (closed-loop AMCL + factory_map) | slam (slam_toolbox) | static (open-loop static TF)'),
        DeclareLaunchArgument('map', default_value='', description='Path atau nama file yaml peta kustom (opsional)'),
        OpaqueFunction(function=launch_setup),
    ])
