import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    controller_type = LaunchConfiguration('controller').perform(context).lower()
    planner_type = LaunchConfiguration('planner').perform(context).lower()

    actions = []

    # ── Path Planner Nodes ──────────────────────────────────────────────
    if planner_type in ('bfs', 'astar'):
        bfs_node = Node(
            package='polebot_research_control',
            executable='bfs_planner',
            name='bfs_planner',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'algorithm': planner_type,
            }],
        )
        actions.append(bfs_node)

    # ── Diff Drive Controller Nodes ────────────────────────────────────
    if controller_type == 'pid':
        odom_to_pose = Node(
            package='polebot_research_control',
            executable='odom_to_posearray_node',
            name='odom_to_posearray_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'odom_topic': '/odom',
                'posearray_topic': '/model/ddr/pose',
                'frame_id': 'odom',
            }],
        )
        pid_node = Node(
            package='polebot_research_control',
            executable='pitdt_profiled_pure_pursuit_controller_node',
            name='pitdt_profiled_pure_pursuit_controller_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'pose_topic': '/model/ddr/pose',
                'reference_path_topic': '/reference_path',
                'cmd_vel_topic': '/cmd_vel',
                'control_rate': 40.0,
                'lookahead_distance': 0.35,
                'stop_distance': 0.08,
                'finish_distance': 0.12,
                'max_v': 0.18,
                'max_w': 0.40,
                'motion_profile_type': 's_curve',
                'max_jerk': 0.16,
                'profile_v_max': 0.18,
                'profile_a_max': 0.40,
                'profile_d_max': 0.50,
                'approach_distance': 0.20,
                'approach_speed': 0.05,
                'profile_min_v': 0.025,
                'profile_min_v_disable_distance': 0.20,
                'max_lateral_accel': 0.20,
                'profile_alpha_max': 0.70,
                'use_gain_scheduling': True,
                'kp_accel': 2.8, 'ki_accel': 0.00, 'kd_accel': 0.08,
                'kp_cruise': 3.2, 'ki_cruise': 0.00, 'kd_cruise': 0.10,
                'kp_decel': 3.0, 'ki_decel': 0.00, 'kd_decel': 0.12,
                'kp_approach': 1.6, 'ki_approach': 0.00, 'kd_approach': 0.05,
                'kp_lin': 0.35, 'ki_lin': 0.00, 'kd_lin': 0.02,
                'kp_ang': 0.65, 'ki_ang': 0.00, 'kd_ang': 0.02,
                'print_debug': False,
            }],
        )
        actions.extend([odom_to_pose, pid_node])

    elif controller_type == 'smc':
        smc_node = Node(
            package='polebot_research_control',
            executable='sliding_mode_controller',
            name='sliding_mode_controller',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'default_mode': 0,  # 0=Solo
                'enable_obstacle_stop': True,
            }],
        )
        actions.append(smc_node)

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='none', description='pid | smc | none'),
        DeclareLaunchArgument('planner', default_value='none', description='bfs | astar | none'),
        OpaqueFunction(function=launch_setup),
    ])
