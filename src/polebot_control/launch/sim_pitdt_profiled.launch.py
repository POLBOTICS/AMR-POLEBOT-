import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    trajectory_mode_arg = DeclareLaunchArgument(
        'trajectory_mode', 
        default_value='s_curve',
        description='Pilihan lintasan: straight, l_left, l_right, arc_left, arc_right, s_curve, complex_course, u_curve, figure_8'
    )
    trajectory_mode = LaunchConfiguration('trajectory_mode')

    polebot_control_share = get_package_share_directory("polebot_control")
    polebot_desc_share = get_package_share_directory("polebot_amr_description")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    world_path = os.path.join(polebot_control_share, "worlds", "straight_track.world.sdf")
    polebot_xacro_sdf = os.path.join(polebot_desc_share, "src", "description", "polebot_amr_description.sdf")

    robot_description_config = xacro.process_file(
        polebot_xacro_sdf,
        mappings={
            'package_path': polebot_desc_share,
            'include_trolley': 'false',
            'hitch_type': 'revolute',
            'mapping_mode': 'false'
        }
    ).toxml()

    temp_sdf_path = "/tmp/polebot_amr_generated.sdf"
    with open(temp_sdf_path, 'w') as f:
        f.write(robot_description_config)

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': True}]
    )

    # NOTE: joint_state_publisher generik (paket ROS untuk URDF) dihapus.
    # robot_description kita SDF, bukan URDF, jadi paket itu gagal
    # membaca daftar joint-nya. Joint state roda yang asli sudah datang
    # dari plugin gz-sim-joint-state-publisher-system di Gazebo, dan
    # sekarang dijembatani lewat topic /joint_states di bawah.

    rviz_config_path = os.path.join(polebot_control_share, 'config', 'polebot.rviz')
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r {world_path}"}.items(),
    )

    spawn_polebot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-file", temp_sdf_path, "-name", "polebot_amr", "-x", "0.0", "-y", "0.0", "-z", "0.38"],
    )

    # Bridge disamakan dengan versi polman-mbd-ros2-agus yang sudah terbukti
    # jalan: plugin gz-sim-diff-drive-system publish TF & odom di topic
    # native "/odom" dan "/tf" (bukan lagi "/model/polebot_amr/tf"), jadi
    # tidak perlu remapping lagi. /joint_states juga ditambahkan supaya
    # robot_state_publisher dapat TF roda yang benar untuk RViz.
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        ],
    )

    odom_to_posearray_node = Node(
        package="polebot_control",
        executable="odom_to_posearray_node",
        output="screen",
        parameters=[{"use_sim_time": True, "odom_topic": "/odom", "posearray_topic": "/model/ddr/pose", "frame_id": "odom"}],
    )

    path_profile_node = Node(
        package="polebot_control",
        executable="path_profile_node",
        output="screen",
        parameters=[
            {"use_sim_time": True, "publish_rate": 30.0, "frame_id": "odom", "trajectory_mode": trajectory_mode},
            {"distance": 3.0, "straight_1": 2.5, "straight_2": 2.0, "straight_3": 2.5},
            {"turn_radius_1": 1.2, "turn_radius_2": 1.0, "turn_angle_1_deg": 90.0, "turn_angle_2_deg": 90.0},
            {"turn_dir_1": "right", "turn_dir_2": "left", "v_max": 0.25, "a_max": 0.08},
            {"repeat": False, "hold_time": 1.0, "samples_per_meter": 40},
        ],
    )

    pitdt_profiled_controller_node = Node(
        package="polebot_control",
        executable="pitdt_profiled_pure_pursuit_controller_node",
        output="screen",
        parameters=[
            {"use_sim_time": True, "pose_topic": "/model/ddr/pose", "reference_path_topic": "/reference_path", "cmd_vel_topic": "/cmd_vel"},
            {"control_rate": 40.0, "lookahead_distance": 0.50, "stop_distance": 0.05, "finish_distance": 0.08},
            {"max_v": 0.50, "max_w": 1.00},
            {"profile_v_max": 0.40, "profile_a_max": 0.50, "profile_d_max": 0.50, "profile_min_v": 0.15, "profile_min_v_disable_distance": 0.20, "max_lateral_accel": 0.20, "profile_alpha_max": 1.0},
            {"kp_lin": 0.35, "ki_lin": 0.00, "kd_lin": 0.02, "u0_lin": 0.00, "ramp_lin": 0.25, "v_fb_max": 0.00, "i_lin_limit": 1.0},
            {"kp_ang": 0.65, "ki_ang": 0.00, "kd_ang": 0.02, "u0_ang": 0.03, "ramp_ang": 0.30, "w_fb_max": 0.50, "i_ang_limit": 1.0},
            {"reacquire_distance": 0.25, "reacquire_heading_deg": 40.0, "reacquire_v_limit": 0.12, "reacquire_min_v": 0.10, "reacquire_w_limit": 0.60},
            {"print_debug": True},
        ],
    )

    return LaunchDescription([
        trajectory_mode_arg,
        rsp_node, gz_sim, bridge,
        TimerAction(period=3.0, actions=[spawn_polebot, rviz2_node]),
        TimerAction(period=6.0, actions=[odom_to_posearray_node]),
        TimerAction(period=10.0, actions=[path_profile_node, pitdt_profiled_controller_node]),
    ])