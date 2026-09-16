#!/usr/bin/env python3

"""
csa_navigator_revised.py

Job 3 global-planner revision for PoleBot AMR CSA navigation.

Architecture status:
- This node is a standalone hybrid planner/controller that runs beside the
  existing ROS2/Nav2 bringup.
- Global/candidate path generation is CSA-based waypoint optimization with
  tournament selection, Levy-flight exploration, obstacle-clearance fitness,
  and route-profile diversity for Path 1/2/3.
- Local velocity selection remains CSA-assisted trajectory scoring.
- This file is NOT a Nav2 plugin yet. It publishes Twist commands to the
  configured command topic.
- Job 2 added ROS2 parameters for repeatable tuning and experiments.
Job 3 replaced the A* global path generator with CSA global planning.
- Job 4 added trap-escape (boxed-in front+left+right) reverse recovery.
- Job 5 added regenerate-on-failed-set candidate path regeneration.
- Job 6 adds front-blocked-and-stuck escape (U-turn + regenerate) and
  publishes traveled path history to /traveled_path for RViz.

Default simulation interface:
- map input        : /map
- scan input       : /scan
- goal input       : /goal_pose
- velocity output  : /cmd_vel_nav
- status output    : /csa_planner_status
- planned path     : /selected_path , /candidate_path_1..3
- traveled history : /traveled_path
"""

import math
import os
import random
from collections import deque
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import tf2_ros
from tf2_ros import TransformException


GridCell = Tuple[int, int]
Point2D = Tuple[float, float]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(value, high))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class MultiPathCSALocalPlanner(Node):
    def __init__(self):
        super().__init__("csa_navigator_revised")

        # =========================================================
        # PARAMETERIZED CONFIGURATION / JOB 2
        # =========================================================
        # Job 2 objective: nilai tuning utama tidak lagi terkunci di kode.
        # Parameter dapat dioverride lewat:
        #   python3 csa_navigator_revised.py --ros-args -p goal_tolerance:=0.30
        self.run_mode = self.declare_parameter(
            "run_mode",
            "standalone_csa_global_local_controller",
        ).value

        # Topic interface. Default sudah cocok dengan hasil validasi:
        # CSA -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel -> bridge_ros_gz.
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav").value
        self.cmd_vel_mirror_topic = self.declare_parameter("cmd_vel_mirror_topic", "").value
        self.scan_topic = self.declare_parameter("scan_topic", "/scan").value
        self.map_topic = self.declare_parameter("map_topic", "/map").value
        self.goal_topic = self.declare_parameter("goal_topic", "/goal_pose").value
        self.status_topic = self.declare_parameter("status_topic", "/csa_planner_status").value
        self.selected_path_topic = self.declare_parameter("selected_path_topic", "/selected_path").value
        self.candidate_path_prefix = self.declare_parameter("candidate_path_prefix", "/candidate_path_").value

        # Frames and TF.
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.fallback_base_frame = self.declare_parameter("fallback_base_frame", "base_footprint").value
        self.tf_cache_time_sec = max(1.0, float(self.declare_parameter("tf_cache_time_sec", 10.0).value))
        self.tf_lookup_timeout_sec = max(0.001, float(self.declare_parameter("tf_lookup_timeout_sec", 0.08).value))
        self.active_base_frame = self.base_frame

        # Occupancy-grid interpretation.
        self.occupied_threshold = int(self.declare_parameter("occupied_threshold", 50).value)
        self.unknown_is_obstacle = bool(self.declare_parameter("unknown_is_obstacle", True).value)

        # Multi-path generation.
        self.max_candidate_paths = int(self.declare_parameter("max_candidate_paths", 3).value)
        self.max_candidate_paths = max(1, min(self.max_candidate_paths, 8))

        # CSA global planner.
        # Setiap global path adalah nest/solusi berisi beberapa waypoint antara start-goal.
        # Path 1 memakai profil direct, Path 2 left detour, Path 3 right detour.
        # Profil berikutnya bergantian kiri/kanan dengan offset lebih besar.
        self.global_planner_type = self.declare_parameter("global_planner_type", "csa_tournament_waypoint").value
        self.global_csa_population = max(8, int(self.declare_parameter("global_csa_population", 48).value))
        self.global_csa_iterations = max(1, int(self.declare_parameter("global_csa_iterations", 70).value))
        self.global_csa_waypoints = max(2, int(self.declare_parameter("global_csa_waypoints", 5).value))
        self.global_csa_tournament_size = max(2, int(self.declare_parameter("global_csa_tournament_size", 4).value))
        self.global_csa_pa = clamp(float(self.declare_parameter("global_csa_pa", 0.25).value), 0.0, 1.0)
        self.global_csa_levy_step_m = max(0.01, float(self.declare_parameter("global_csa_levy_step_m", 0.85).value))
        self.global_csa_gaussian_step_m = max(0.01, float(self.declare_parameter("global_csa_gaussian_step_m", 0.30).value))
        self.global_csa_sampling_step_m = max(0.02, float(self.declare_parameter("global_csa_sampling_step_m", 0.05).value))
        self.global_csa_side_offset_m = max(0.0, float(self.declare_parameter("global_csa_side_offset_m", 1.20).value))
        self.global_csa_side_weight = max(0.0, float(self.declare_parameter("global_csa_side_weight", 0.45).value))
        self.global_csa_length_weight = max(0.0, float(self.declare_parameter("global_csa_length_weight", 1.0).value))
        self.global_csa_smoothness_weight = max(0.0, float(self.declare_parameter("global_csa_smoothness_weight", 0.35).value))
        self.global_csa_obstacle_weight = max(0.0, float(self.declare_parameter("global_csa_obstacle_weight", 2.8).value))
        self.global_csa_collision_penalty = max(1.0, float(self.declare_parameter("global_csa_collision_penalty", 500.0).value))
        self.global_csa_outside_map_penalty = max(1.0, float(self.declare_parameter("global_csa_outside_map_penalty", 800.0).value))
        self.global_csa_history_weight = max(0.0, float(self.declare_parameter("global_csa_history_weight", 1.0).value))
        self.global_csa_similarity_penalty = max(0.0, float(self.declare_parameter("global_csa_similarity_penalty", 250.0).value))
        self.global_csa_min_clearance_floor_m = max(0.01, float(self.declare_parameter("global_csa_min_clearance_floor_m", 0.04).value))
        self.global_csa_direct_seed_enabled = bool(self.declare_parameter("global_csa_direct_seed_enabled", True).value)

        self.max_regeneration_count = int(self.declare_parameter("max_regeneration_count", 5).value)
        self.max_regeneration_count = max(0, self.max_regeneration_count)
        self.path_generation_attempt_multiplier = int(self.declare_parameter("path_generation_attempt_multiplier", 6).value)
        self.path_generation_attempt_multiplier = max(1, self.path_generation_attempt_multiplier)

        self.path_densify_step_m = max(0.02, float(self.declare_parameter("path_densify_step_m", 0.10).value))
        self.path_penalty_stride_divisor = max(1, int(self.declare_parameter("path_penalty_stride_divisor", 160).value))
        self.nearest_free_max_radius_cells = max(1, int(self.declare_parameter("nearest_free_max_radius_cells", 40).value))
        self.nearest_free_required_clearance = max(0.0, float(self.declare_parameter("nearest_free_required_clearance", 0.05).value))
        self.path_similarity_threshold = float(self.declare_parameter("path_similarity_threshold", 0.80).value)
        self.blocked_history_similarity_threshold = float(self.declare_parameter("blocked_history_similarity_threshold", 0.70).value)

        self.blocked_history_penalty_radius_m = max(0.05, float(self.declare_parameter("blocked_history_penalty_radius_m", 0.85).value))
        self.blocked_history_penalty_base = float(self.declare_parameter("blocked_history_penalty_base", 14.0).value)
        self.blocked_history_penalty_regen_gain = float(self.declare_parameter("blocked_history_penalty_regen_gain", 3.0).value)
        self.blocked_obstacle_penalty_radius_m = max(0.05, float(self.declare_parameter("blocked_obstacle_penalty_radius_m", 0.45).value))
        self.blocked_obstacle_penalty_value = float(self.declare_parameter("blocked_obstacle_penalty_value", 80.0).value)
        self.similar_path_penalty_radius_m = max(0.05, float(self.declare_parameter("similar_path_penalty_radius_m", 0.60).value))
        self.similar_path_penalty_base = float(self.declare_parameter("similar_path_penalty_base", 6.0).value)
        self.blocked_similarity_penalty_radius_m = max(0.05, float(self.declare_parameter("blocked_similarity_penalty_radius_m", 0.70).value))
        self.blocked_similarity_penalty_base = float(self.declare_parameter("blocked_similarity_penalty_base", 12.0).value)
        self.accepted_path_penalty_radius_m = max(0.05, float(self.declare_parameter("accepted_path_penalty_radius_m", 0.75).value))
        self.accepted_path_penalty_base = float(self.declare_parameter("accepted_path_penalty_base", 5.0).value)
        self.accepted_path_penalty_attempt_gain = float(self.declare_parameter("accepted_path_penalty_attempt_gain", 2.0).value)

        self.levy_regen_count_base = max(0, int(self.declare_parameter("levy_regen_count_base", 18).value))
        self.levy_regen_count_gain = max(0, int(self.declare_parameter("levy_regen_count_gain", 6).value))
        self.levy_lateral_gain_cells = max(0.0, float(self.declare_parameter("levy_lateral_gain_cells", 8.0).value))
        self.levy_lateral_min_cells = max(0.0, float(self.declare_parameter("levy_lateral_min_cells", 4.0).value))
        self.levy_lateral_max_cells = max(self.levy_lateral_min_cells, float(self.declare_parameter("levy_lateral_max_cells", 28.0).value))
        self.levy_ratio_min = float(self.declare_parameter("levy_ratio_min", 0.20).value)
        self.levy_ratio_max = float(self.declare_parameter("levy_ratio_max", 0.85).value)
        if self.levy_ratio_max < self.levy_ratio_min:
            self.levy_ratio_min, self.levy_ratio_max = self.levy_ratio_max, self.levy_ratio_min
        self.levy_penalty_radius_m = max(0.05, float(self.declare_parameter("levy_penalty_radius_m", 0.35).value))
        self.levy_penalty_min = float(self.declare_parameter("levy_penalty_min", 3.0).value)
        self.levy_penalty_max = float(self.declare_parameter("levy_penalty_max", 8.0).value)
        if self.levy_penalty_max < self.levy_penalty_min:
            self.levy_penalty_min, self.levy_penalty_max = self.levy_penalty_max, self.levy_penalty_min

        # Path score weights for candidate ranking.
        self.path_score_length_weight = float(self.declare_parameter("path_score_length_weight", 1.0).value)
        self.path_score_turn_weight = float(self.declare_parameter("path_score_turn_weight", 0.35).value)
        self.path_score_min_clearance_weight = float(self.declare_parameter("path_score_min_clearance_weight", 1.5).value)
        self.path_score_avg_clearance_weight = float(self.declare_parameter("path_score_avg_clearance_weight", 0.5).value)
        self.path_score_clearance_floor_m = max(0.01, float(self.declare_parameter("path_score_clearance_floor_m", 0.05).value))

        # Auto path switching.
        self.auto_path_switch_enabled = bool(self.declare_parameter("auto_path_switch_enabled", True).value)
        self.auto_switch_current_far_m = max(0.0, float(self.declare_parameter("auto_switch_current_far_m", 0.75).value))
        self.auto_switch_candidate_near_m = max(0.0, float(self.declare_parameter("auto_switch_candidate_near_m", 0.45).value))
        self.auto_switch_margin_m = max(0.0, float(self.declare_parameter("auto_switch_margin_m", 0.25).value))
        self.auto_switch_cooldown_sec = max(0.0, float(self.declare_parameter("auto_switch_cooldown_sec", 2.0).value))

        # Robot geometry and safety clearances.
        self.robot_radius = max(0.01, float(self.declare_parameter("robot_radius", 0.23).value))
        self.wheel_radius_m = max(0.001, float(self.declare_parameter("wheel_radius_m", 0.079).value))
        self.wheel_separation_m = max(0.001, float(self.declare_parameter("wheel_separation_m", 0.590).value))
        self.path_clearance_margin_m = max(0.0, float(self.declare_parameter("path_clearance_margin_m", 0.25).value))
        self.trajectory_clearance_margin_m = max(0.0, float(self.declare_parameter("trajectory_clearance_margin_m", 0.25).value))
        self.path_required_clearance = self.robot_radius + self.path_clearance_margin_m
        self.trajectory_required_clearance = self.robot_radius + self.trajectory_clearance_margin_m
        self.dynamic_safety_margin = max(0.0, float(self.declare_parameter("dynamic_safety_margin", 0.35).value))

        self.goal_tolerance = max(0.05, float(self.declare_parameter("goal_tolerance", 0.45).value))
        self.final_stop_distance = max(self.goal_tolerance, float(self.declare_parameter("final_stop_distance", 0.65).value))

        # Control loop and pure pursuit.
        self.control_frequency = max(1.0, float(self.declare_parameter("control_frequency", 10.0).value))
        self.control_period = 1.0 / self.control_frequency
        self.lookahead_distance = max(0.05, float(self.declare_parameter("lookahead_distance", 0.85).value))
        self.near_goal_distance = max(0.05, float(self.declare_parameter("near_goal_distance", 1.0).value))
        self.near_goal_lookahead_distance = max(0.05, float(self.declare_parameter("near_goal_lookahead_distance", 0.45).value))
        self.forward_point_min_x = float(self.declare_parameter("forward_point_min_x", -0.10).value)
        self.goal_forward_stop_x = float(self.declare_parameter("goal_forward_stop_x", 0.10).value)

        self.v_min = max(0.0, float(self.declare_parameter("v_min", 0.00).value))
        self.v_max = max(self.v_min, float(self.declare_parameter("v_max", 0.40).value))
        self.v_preferred = clamp(float(self.declare_parameter("v_preferred", 0.36).value), self.v_min, self.v_max)
        self.v_hard_obstacle = clamp(float(self.declare_parameter("v_hard_obstacle", 0.20).value), self.v_min, self.v_max)
        self.v_soft_obstacle = clamp(float(self.declare_parameter("v_soft_obstacle", 0.30).value), self.v_min, self.v_max)
        self.v_near_goal_max = clamp(float(self.declare_parameter("v_near_goal_max", 0.24).value), self.v_min, self.v_max)
        self.w_max_clear = max(0.05, float(self.declare_parameter("w_max_clear", 0.58).value))
        self.w_max_obstacle = max(self.w_max_clear, float(self.declare_parameter("w_max_obstacle", 0.95).value))
        self.max_delta_v = max(0.0, float(self.declare_parameter("max_delta_v", 0.050).value))
        self.max_delta_w = max(0.0, float(self.declare_parameter("max_delta_w", 0.090).value))
        self.angular_deadband = max(0.0, float(self.declare_parameter("angular_deadband", 0.030).value))

        self.curvature_angle_scale = max(0.1, float(self.declare_parameter("curvature_angle_scale", 1.7).value))
        self.curvature_factor_min = clamp(float(self.declare_parameter("curvature_factor_min", 0.45).value), 0.0, 1.0)

        # CSA velocity optimizer.
        self.random_seed = int(self.declare_parameter("random_seed", -1).value)
        if self.random_seed >= 0:
            random.seed(self.random_seed)

        self.pop_size = max(4, int(self.declare_parameter("csa_pop_size", 28).value))
        self.max_iter = max(0, int(self.declare_parameter("csa_max_iter", 3).value))
        self.pa = clamp(float(self.declare_parameter("csa_pa", 0.25).value), 0.0, 1.0)
        self.beta = clamp(float(self.declare_parameter("csa_beta", 1.5).value), 1.01, 1.99)
        self.levy_alpha = max(0.0, float(self.declare_parameter("csa_levy_alpha", 0.055).value))
        self.prediction_time = max(0.05, float(self.declare_parameter("prediction_time", 1.15).value))
        self.prediction_dt = max(0.02, float(self.declare_parameter("prediction_dt", 0.10).value))
        self.prediction_steps = max(1, int(self.prediction_time / self.prediction_dt))
        self.candidate_gaussian_probability = clamp(float(self.declare_parameter("candidate_gaussian_probability", 0.84).value), 0.0, 1.0)
        self.candidate_v_std = max(0.0, float(self.declare_parameter("candidate_v_std", 0.050).value))
        self.candidate_w_std = max(0.0, float(self.declare_parameter("candidate_w_std", 0.120).value))
        self.candidate_random_v_min = clamp(float(self.declare_parameter("candidate_random_v_min", 0.08).value), self.v_min, self.v_max)

        # Obstacle perception thresholds.
        self.obstacle_range = max(0.10, float(self.declare_parameter("obstacle_range", 2.20).value))
        self.front_angle_deg = max(1.0, float(self.declare_parameter("front_angle_deg", 42.0).value))
        self.front_angle = math.radians(self.front_angle_deg)
        self.soft_obstacle_distance = max(0.05, float(self.declare_parameter("soft_obstacle_distance", 1.50).value))
        self.hard_obstacle_distance = max(0.02, float(self.declare_parameter("hard_obstacle_distance", 0.90).value))
        self.emergency_distance = max(0.01, float(self.declare_parameter("emergency_distance", 0.45).value))
        self.dynamic_replan_distance = max(
            self.emergency_distance,
            float(self.declare_parameter("dynamic_replan_distance", 1.35).value),
        )
        self.full_block_front_distance = max(0.01, float(self.declare_parameter("full_block_front_distance", 0.48).value))
        self.full_block_side_clearance = max(0.01, float(self.declare_parameter("full_block_side_clearance", 0.28).value))
        self.block_confirm_limit = max(1, int(self.declare_parameter("block_confirm_limit", 12).value))

        # Trap escape: used when the robot is boxed in by close front-left-right obstacles.
        # This is intentionally independent from normal CSA local avoidance so the robot
        # does not keep searching around a path that is physically blocked.
        self.trap_escape_enabled = bool(self.declare_parameter("trap_escape_enabled", True).value)
        self.trap_escape_front_distance = max(0.05, float(self.declare_parameter("trap_escape_front_distance", 0.75).value))
        self.trap_escape_side_distance = max(0.05, float(self.declare_parameter("trap_escape_side_distance", 0.55).value))
        self.trap_escape_confirm_sec = max(0.05, float(self.declare_parameter("trap_escape_confirm_sec", 1.0).value))
        self.trap_escape_reverse_sec = max(0.05, float(self.declare_parameter("trap_escape_reverse_sec", 1.15).value))
        self.trap_escape_switch_after_reverse = bool(self.declare_parameter("trap_escape_switch_after_reverse", True).value)
        self.trap_escape_regen_if_no_path = bool(self.declare_parameter("trap_escape_regen_if_no_path", True).value)

        # JOB 6 - Front-blocked escape.
        self.front_stuck_escape_enabled = bool(self.declare_parameter("front_stuck_escape_enabled", True).value)
        self.front_stuck_front_distance = max(0.05, float(self.declare_parameter("front_stuck_front_distance", 0.70).value))
        self.front_stuck_progress_meter = max(0.0, float(self.declare_parameter("front_stuck_progress_meter", 0.08).value))
        self.front_stuck_confirm_sec = max(0.1, float(self.declare_parameter("front_stuck_confirm_sec", 4.0).value))
        self.front_stuck_uturn_sec = max(0.05, float(self.declare_parameter("front_stuck_uturn_sec", 1.6).value))
        self.front_stuck_uturn_speed = float(self.declare_parameter("front_stuck_uturn_speed", 0.65).value)

        # JOB 6 - History jalur yang benar-benar ditempuh PoleBot (untuk RViz).
        self.traveled_path_enabled = bool(self.declare_parameter("traveled_path_enabled", True).value)
        self.traveled_path_topic = self.declare_parameter("traveled_path_topic", "/traveled_path").value
        self.traveled_path_min_step_m = max(0.01, float(self.declare_parameter("traveled_path_min_step_m", 0.05).value))
        self.traveled_path_max_points = max(50, int(self.declare_parameter("traveled_path_max_points", 5000).value))

        # STEP A - Costmap dinamis Nav2.
        # Subscribe ke /local_costmap/costmap dan /global_costmap/costmap dari Nav2.
        # Local costmap diperbarui real-time saat obstacle baru (statis atau dinamis)
        # terdeteksi sensor, sehingga passable check dan trajectory scoring otomatis
        # memperhitungkan obstacle yang tidak ada di /map statis.
        # Sesuai Tujuan 2 KTI: "menambahkan lapisan costmap dinamis untuk penanganan
        # rintangan dinamis secara real-time."
        self.costmap_enabled = bool(self.declare_parameter("costmap_enabled", True).value)
        self.local_costmap_topic = self.declare_parameter(
            "local_costmap_topic", "/local_costmap/costmap"
        ).value
        self.global_costmap_topic = self.declare_parameter(
            "global_costmap_topic", "/global_costmap/costmap"
        ).value
        # Nav2 costmap cost values (internal 0-254 → OccupancyGrid 0-100):
        # ≥ costmap_inscribed_threshold: robot footprint menabrak (inscribed radius)
        # ≥ costmap_occupied_threshold: lethal/terhalang total
        self.costmap_inscribed_threshold = int(self.declare_parameter("costmap_inscribed_threshold", 65).value)
        self.costmap_occupied_threshold = int(self.declare_parameter("costmap_occupied_threshold", 98).value)
        # Seberapa agresif costmap lokal mempengaruhi clearance estimate.
        # 1.0 = fully trust costmap inflation gradient; 0.0 = hanya pakai distance_grid.
        self.costmap_clearance_weight = clamp(
            float(self.declare_parameter("costmap_clearance_weight", 0.60).value), 0.0, 1.0
        )
        # Gunakan global costmap untuk global path planning (CSA).
        self.use_global_costmap_for_planning = bool(
            self.declare_parameter("use_global_costmap_for_planning", True).value
        )
        # Gunakan local costmap untuk trajectory scoring (CSA lokal).
        self.use_local_costmap_for_scoring = bool(
            self.declare_parameter("use_local_costmap_for_scoring", True).value
        )

        self.scan_stride_target_points = max(30, int(self.declare_parameter("scan_stride_target_points", 360).value))
        self.ignore_obstacle_behind_x = float(self.declare_parameter("ignore_obstacle_behind_x", -0.10).value)
        self.capture_blocking_distance_m = max(0.05, float(self.declare_parameter("capture_blocking_distance_m", 1.80).value))
        self.capture_blocking_angle_deg = max(1.0, float(self.declare_parameter("capture_blocking_angle_deg", 75.0).value))
        self.capture_blocking_angle = math.radians(self.capture_blocking_angle_deg)
        self.max_blocked_obstacle_cells = max(1, int(self.declare_parameter("max_blocked_obstacle_cells", 400).value))

        self.obstacle_bias_base = float(self.declare_parameter("obstacle_bias_base", 0.10).value)
        self.obstacle_bias_gain = float(self.declare_parameter("obstacle_bias_gain", 0.45).value)

        self.hard_geometry_front_distance = max(0.01, float(self.declare_parameter("hard_geometry_front_distance", 0.62).value))
        self.hard_geometry_side_distance = max(0.01, float(self.declare_parameter("hard_geometry_side_distance", 0.36).value))
        self.very_low_valid_ratio = clamp(float(self.declare_parameter("very_low_valid_ratio", 0.10).value), 0.0, 1.0)

        # Recovery and initial turn.
        self.reverse_duration_sec = max(0.0, float(self.declare_parameter("reverse_duration_sec", 1.0).value))
        self.reverse_speed = float(self.declare_parameter("reverse_speed", -0.18).value)
        self.reverse_required_before_switch = max(1, int(self.declare_parameter("reverse_required_before_switch", 2).value))
        self.uturn_duration_sec = max(0.0, float(self.declare_parameter("uturn_duration_sec", 1.6).value))
        self.uturn_angular_speed = float(self.declare_parameter("uturn_angular_speed", 0.65).value)
        self.recovery_cooldown_sec = max(0.0, float(self.declare_parameter("recovery_cooldown_sec", 1.5).value))
        self.initial_turn_duration_sec = max(0.0, float(self.declare_parameter("initial_turn_duration_sec", 4.0).value))
        self.initial_turn_angular_speed_search = float(self.declare_parameter("initial_turn_angular_speed_search", 0.50).value)
        self.initial_turn_angular_speed_track = float(self.declare_parameter("initial_turn_angular_speed_track", 0.55).value)
        self.initial_turn_done_x = float(self.declare_parameter("initial_turn_done_x", 0.10).value)
        self.initial_turn_done_bearing_deg = max(1.0, float(self.declare_parameter("initial_turn_done_bearing_deg", 55.0).value))
        self.goal_behind_x = float(self.declare_parameter("goal_behind_x", -0.20).value)
        self.goal_behind_angle_deg = max(1.0, float(self.declare_parameter("goal_behind_angle_deg", 115.0).value))

        # Stall and diagnostics.
        self.stall_timeout = max(0.1, float(self.declare_parameter("stall_timeout", 5.0).value))
        self.min_progress_meter = max(0.0, float(self.declare_parameter("min_progress_meter", 0.05).value))
        self.no_valid_timeout = max(0.1, float(self.declare_parameter("no_valid_timeout", 2.0).value))
        self.hide_obstacle_terminal_log = bool(self.declare_parameter("hide_obstacle_terminal_log", False).value)
        self.log_interval_sec = max(0.05, float(self.declare_parameter("log_interval_sec", 0.90).value))

        # CSA trajectory fitness weights.
        self.traj_path_average_weight = float(self.declare_parameter("traj_path_average_weight", 0.12).value)
        self.traj_obstacle_cost_floor = max(0.001, float(self.declare_parameter("traj_obstacle_cost_floor", 0.05).value))
        self.traj_speed_w_weight = float(self.declare_parameter("traj_speed_w_weight", 0.28).value)
        self.traj_smooth_v_weight = float(self.declare_parameter("traj_smooth_v_weight", 0.50).value)
        self.traj_smooth_w_weight = float(self.declare_parameter("traj_smooth_w_weight", 1.35).value)
        self.traj_negative_progress_weight = float(self.declare_parameter("traj_negative_progress_weight", 4.0).value)
        self.traj_positive_progress_weight = float(self.declare_parameter("traj_positive_progress_weight", -1.30).value)
        self.traj_hard_path_weight = float(self.declare_parameter("traj_hard_path_weight", 3.0).value)
        self.traj_hard_obstacle_weight = float(self.declare_parameter("traj_hard_obstacle_weight", 32.0).value)
        self.traj_hard_heading_weight = float(self.declare_parameter("traj_hard_heading_weight", 1.8).value)
        self.traj_soft_path_weight = float(self.declare_parameter("traj_soft_path_weight", 5.2).value)
        self.traj_soft_obstacle_weight = float(self.declare_parameter("traj_soft_obstacle_weight", 18.0).value)
        self.traj_soft_heading_weight = float(self.declare_parameter("traj_soft_heading_weight", 2.0).value)
        self.traj_clear_path_weight = float(self.declare_parameter("traj_clear_path_weight", 7.0).value)
        self.traj_clear_obstacle_weight = float(self.declare_parameter("traj_clear_obstacle_weight", 8.0).value)
        self.traj_clear_heading_weight = float(self.declare_parameter("traj_clear_heading_weight", 2.4).value)
        self.traj_map_obstacle_weight = float(self.declare_parameter("traj_map_obstacle_weight", 5.0).value)
        self.traj_speed_cost_weight = float(self.declare_parameter("traj_speed_cost_weight", 1.0).value)
        self.traj_smooth_cost_weight = float(self.declare_parameter("traj_smooth_cost_weight", 1.2).value)
        self.traj_angular_cost_weight = float(self.declare_parameter("traj_angular_cost_weight", 0.25).value)
        self.invalid_trajectory_cost = float(self.declare_parameter("invalid_trajectory_cost", 1_000_000.0).value)

        # Seed velocities for CSA. These remain deterministic parameters so
        # experiments can be repeated without hidden tuning in the code.
        self.seed_v_ref_scale_1 = float(self.declare_parameter("seed_v_ref_scale_1", 0.95).value)
        self.seed_w_ref_scale_1 = float(self.declare_parameter("seed_w_ref_scale_1", 0.75).value)
        self.seed_v_ref_scale_2 = float(self.declare_parameter("seed_v_ref_scale_2", 0.90).value)
        self.seed_w_offset = float(self.declare_parameter("seed_w_offset", 0.10).value)
        self.seed_v_bias_min = clamp(float(self.declare_parameter("seed_v_bias_min", 0.24).value), self.v_min, self.v_max)
        self.seed_v_mid = clamp(float(self.declare_parameter("seed_v_mid", 0.28).value), self.v_min, self.v_max)
        self.seed_v_low = clamp(float(self.declare_parameter("seed_v_low", 0.22).value), self.v_min, self.v_max)
        self.seed_w_bias_offset = float(self.declare_parameter("seed_w_bias_offset", 0.20).value)
        self.seed_v_turn = clamp(float(self.declare_parameter("seed_v_turn", 0.16).value), self.v_min, self.v_max)
        self.seed_w_turn = float(self.declare_parameter("seed_w_turn", 0.45).value)

        # =========================================================
        # ROS INTERFACE
        # =========================================================
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.cmd_vel_mirror_pub = None
        if self.cmd_vel_mirror_topic and self.cmd_vel_mirror_topic != self.cmd_vel_topic:
            self.cmd_vel_mirror_pub = self.create_publisher(Twist, self.cmd_vel_mirror_topic, 10)

        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.selected_path_pub = self.create_publisher(Path, self.selected_path_topic, 10)
        self.candidate_path_pubs = [
            self.create_publisher(Path, f"{self.candidate_path_prefix}{i + 1}", 10)
            for i in range(self.max_candidate_paths)
        ]

        # JOB 6 - publisher history jalur yang ditempuh robot.
        self.traveled_path_pub = None
        if self.traveled_path_enabled:
            self.traveled_path_pub = self.create_publisher(Path, self.traveled_path_topic, 10)

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, map_qos)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, self.goal_topic, self.goal_callback, 10)
        self.plan_topic = self.declare_parameter("plan_topic", "/plan").value
        self.plan_sub = self.create_subscription(Path, self.plan_topic, self.plan_callback, 10)

        # STEP A - subscribe ke costmap Nav2.
        # Nav2 costmap publish dengan TRANSIENT_LOCAL sehingga subscriber baru
        # langsung menerima pesan terakhir (latched-like behavior di ROS2).
        if self.costmap_enabled:
            costmap_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.local_costmap_sub = self.create_subscription(
                OccupancyGrid, self.local_costmap_topic,
                self.local_costmap_callback, costmap_qos,
            )
            self.global_costmap_sub = self.create_subscription(
                OccupancyGrid, self.global_costmap_topic,
                self.global_costmap_callback, costmap_qos,
            )
        else:
            self.local_costmap_sub = None
            self.global_costmap_sub = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=self.tf_cache_time_sec))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # =========================================================
        # RUNTIME STATE
        # =========================================================
        self.map_msg: Optional[OccupancyGrid] = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_data: List[int] = []
        self.distance_grid: List[float] = []

        # STEP A - runtime costmap dinamis Nav2.
        self.local_costmap_data: Optional[List[int]] = None
        self.local_costmap_width = 0
        self.local_costmap_height = 0
        self.local_costmap_resolution = 0.05
        self.local_costmap_origin_x = 0.0
        self.local_costmap_origin_y = 0.0
        self.local_costmap_stamp: float = 0.0

        self.global_costmap_data: Optional[List[int]] = None
        self.global_costmap_width = 0
        self.global_costmap_height = 0
        self.global_costmap_resolution = 0.05
        self.global_costmap_origin_x = 0.0
        self.global_costmap_origin_y = 0.0
        self.global_costmap_stamp: float = 0.0

        self.latest_scan: Optional[LaserScan] = None
        self.goal_pose: Optional[PoseStamped] = None
        self.pending_goal: Optional[PoseStamped] = None

        self.candidate_paths_cells: List[List[GridCell]] = []
        self.candidate_paths_world: List[List[Point2D]] = []
        self.candidate_costs: List[float] = []
        self.path_blocked: List[bool] = []
        self.selected_path_index: Optional[int] = None
        self.path_progress_index = 0
        self.blocked_path_history: List[List[GridCell]] = []
        self.blocked_obstacle_cells: List[GridCell] = []
        self.regeneration_count = 0

        self.auto_switch_cooldown_ticks = 0
        self.auto_switch_cooldown_limit = int(self.auto_switch_cooldown_sec * self.control_frequency)

        self.prev_v_cmd = 0.0
        self.prev_w_cmd = 0.0

        self.block_confirm_ticks = 0
        self.block_confirm_reason = "NONE"

        self.trap_escape_ticks = 0
        self.trap_escape_confirm_limit = max(1, int(self.trap_escape_confirm_sec * self.control_frequency))
        self.trap_escape_reverse_ticks = 0
        self.trap_escape_reverse_tick_limit = max(1, int(self.trap_escape_reverse_sec * self.control_frequency))
        self.trap_escape_reason = "NONE"

        # JOB 6 - runtime front-stuck escape.
        self.front_stuck_start_time: Optional[float] = None
        self.front_stuck_uturn_ticks = 0
        self.front_stuck_uturn_tick_limit = max(1, int(self.front_stuck_uturn_sec * self.control_frequency))
        self.front_stuck_uturn_direction = 1.0
        self.front_stuck_reason = "NONE"

        # JOB 6 - history jalur yang ditempuh.
        self.traveled_points: List[Point2D] = []

        self.state = "WAIT_GOAL"
        self.reverse_ticks = 0
        self.reverse_tick_limit = int(self.reverse_duration_sec * self.control_frequency)
        self.reverse_count_current_path = 0
        self.uturn_ticks = 0
        self.uturn_tick_limit = int(self.uturn_duration_sec * self.control_frequency)
        self.uturn_direction = 1.0
        self.recovery_cooldown_ticks = 0
        self.dynamic_replan_cooldown_ticks = 0
        self.recovery_cooldown_limit = int(self.recovery_cooldown_sec * self.control_frequency)
        self.initial_turn_done = True
        self.initial_turn_ticks = 0
        self.initial_turn_tick_limit = int(self.initial_turn_duration_sec * self.control_frequency)
        self.initial_turn_direction = 1.0

        self.last_goal_distance: Optional[float] = None
        self.stall_start_time: Optional[float] = None
        self.no_valid_start_time: Optional[float] = None

        self.last_log_text = ""
        self.last_log_time = self.now_sec()

        self.timer = self.create_timer(self.control_period, self.control_loop)

        self.get_logger().info("CSA Navigator Revised aktif.")
        self.get_logger().info(f"Mode arsitektur: {self.run_mode} | bukan Nav2 plugin murni.")
        self.get_logger().info("Global/candidate path: CSA waypoint planner + tournament selection + Levy flight.")
        self.get_logger().info("Local command optimizer: CSA-assisted velocity trajectory scoring.")
        self.get_logger().info(f"Output velocity utama: {self.cmd_vel_topic}")
        if self.cmd_vel_mirror_pub is not None:
            self.get_logger().warn(f"Mirror velocity aktif: {self.cmd_vel_mirror_topic}")
        else:
            self.get_logger().info("Mirror velocity nonaktif; hanya publish ke output utama.")
        self.get_logger().info(f"Input scan: {self.scan_topic} | map: {self.map_topic} | goal: {self.goal_topic}")
        self.get_logger().info(f"Frame: map={self.map_frame}, base={self.base_frame}, fallback={self.fallback_base_frame}")
        self.get_logger().info(
            "Parameter utama: "
            f"goal_tol={self.goal_tolerance:.2f}m, v_max={self.v_max:.2f}m/s, "
            f"lookahead={self.lookahead_distance:.2f}m, pop={self.pop_size}, "
            f"pa={self.pa:.2f}, beta={self.beta:.2f}, seed={self.random_seed}"
        )
        self.get_logger().info(
            f"Candidate paths={self.max_candidate_paths}, global_csa_pop={self.global_csa_population}, global_csa_iter={self.global_csa_iterations}, control_frequency={self.control_frequency:.1f} Hz"
        )
        self.get_logger().info(
            "Trap escape: "
            f"enabled={self.trap_escape_enabled}, "
            f"front<{self.trap_escape_front_distance:.2f}m, "
            f"left/right<{self.trap_escape_side_distance:.2f}m, "
            f"confirm={self.trap_escape_confirm_sec:.2f}s, "
            f"reverse={self.trap_escape_reverse_sec:.2f}s"
        )
        self.get_logger().info(
            "Front-stuck escape (Job 6): "
            f"enabled={self.front_stuck_escape_enabled}, "
            f"front<{self.front_stuck_front_distance:.2f}m, "
            f"confirm={self.front_stuck_confirm_sec:.1f}s -> uturn {self.front_stuck_uturn_sec:.1f}s -> regenerate"
        )
        if self.traveled_path_pub is not None:
            self.get_logger().info(f"Traveled path history (Job 6): {self.traveled_path_topic}")
        if self.costmap_enabled:
            self.get_logger().info(
                "Costmap dinamis Nav2 (Step A): "
                f"local={self.local_costmap_topic}, global={self.global_costmap_topic} | "
                f"inscribed_threshold={self.costmap_inscribed_threshold}, "
                f"occupied_threshold={self.costmap_occupied_threshold} | "
                f"use_global_for_planning={self.use_global_costmap_for_planning}, "
                f"use_local_for_scoring={self.use_local_costmap_for_scoring}"
            )
        else:
            self.get_logger().info("Costmap dinamis Nav2 (Step A): disabled — hanya pakai static /map")

    # =============================================================
    # JOB 6 - TRAVELED PATH HISTORY
    # =============================================================
    def record_traveled_point(self, robot_pose_map: Tuple[float, float, float]) -> None:
        if not self.traveled_path_enabled:
            return
        rx, ry, _ = robot_pose_map
        if self.traveled_points:
            lx, ly = self.traveled_points[-1]
            if math.hypot(rx - lx, ry - ly) < self.traveled_path_min_step_m:
                return
        self.traveled_points.append((rx, ry))
        if len(self.traveled_points) > self.traveled_path_max_points:
            self.traveled_points = self.traveled_points[-self.traveled_path_max_points:]
        self.publish_traveled_path()

    def publish_traveled_path(self) -> None:
        if self.traveled_path_pub is None:
            return
        msg = Path()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for wx, wy in self.traveled_points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.traveled_path_pub.publish(msg)

    def reset_traveled_path(self) -> None:
        self.traveled_points = []
        self.publish_traveled_path()

    # =============================================================
    # JOB 6 - FRONT-STUCK ESCAPE
    # =============================================================
    def update_front_stuck_confirmation(
        self, front_min: float, goal_distance: float,
    ) -> Tuple[bool, str]:
        if not self.front_stuck_escape_enabled:
            self.front_stuck_start_time = None
            return False, "FRONT_STUCK_DISABLED"
        now = self.now_sec()
        front_blocked = front_min < self.front_stuck_front_distance
        making_progress = False
        if self.last_goal_distance is not None:
            making_progress = (self.last_goal_distance - goal_distance) >= self.front_stuck_progress_meter
        if front_blocked and not making_progress:
            if self.front_stuck_start_time is None:
                self.front_stuck_start_time = now
            elapsed = now - self.front_stuck_start_time
            reason = (
                f"FRONT_BLOCKED_AND_STUCK | front={front_min:.2f}<{self.front_stuck_front_distance:.2f} | "
                f"stuck={elapsed:.1f}s/{self.front_stuck_confirm_sec:.1f}s"
            )
            return elapsed >= self.front_stuck_confirm_sec, reason
        self.front_stuck_start_time = None
        return False, "FRONT_STUCK_NOT_CONFIRMED"

    def start_front_stuck_uturn(
        self, reason: str, robot_pose_map: Tuple[float, float, float],
        obstacle_points: List[Point2D], left_min: float, right_min: float,
    ) -> None:
        self.stop_robot()
        self.capture_blocking_obstacles(robot_pose_map, obstacle_points)
        self.mark_current_path_blocked(reason)
        self.front_stuck_uturn_direction = 1.0 if left_min >= right_min else -1.0
        self.front_stuck_uturn_ticks = self.front_stuck_uturn_tick_limit
        self.state = "FRONT_STUCK_UTURN"
        self.front_stuck_start_time = None
        self.block_confirm_ticks = 0
        self.prev_v_cmd = 0.0
        self.prev_w_cmd = 0.0
        self.log_status(
            f"FRONT_STUCK_UTURN_START | path={self.current_path_number_text()} | "
            f"dir={'LEFT' if self.front_stuck_uturn_direction > 0 else 'RIGHT'} | reason={reason}",
            level="warn", force=True,
        )

    def finish_front_stuck_uturn(self) -> None:
        self.stop_robot()
        self.front_stuck_uturn_ticks = 0
        self.log_status(
            "FRONT_STUCK_REGEN | delete candidate 1-3 | generate new detour paths",
            level="warn", force=True,
        )
        self.regenerate_paths_after_all_blocked("front_blocked_stuck_uturn_then_regenerate")
    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def log_status(self, text: str, level: str = "info", force: bool = False) -> None:
        now = self.now_sec()
        dt = now - self.last_log_time

        if force or text != self.last_log_text or dt >= self.log_interval_sec:
            self.last_log_text = text
            self.last_log_time = now

            if level == "warn":
                self.get_logger().warn(text)
            elif level == "error":
                self.get_logger().error(text)
            else:
                self.get_logger().info(text)

            msg = String()
            msg.data = text
            self.status_pub.publish(msg)

    # =============================================================
    # CALLBACKS
    # =============================================================
    def map_callback(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_data = list(msg.data)

        self.compute_distance_grid()

        self.log_status(
            f"MAP_READY | size={self.map_width}x{self.map_height} | res={self.map_resolution:.3f}",
            level="info",
            force=True,
        )

        if self.pending_goal is not None:
            self.generate_paths_to_goal(self.pending_goal, reset_history=False)
            self.pending_goal = None

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    # STEP A - Costmap callbacks
    def local_costmap_callback(self, msg: OccupancyGrid) -> None:
        self.local_costmap_data = list(msg.data)
        self.local_costmap_width = msg.info.width
        self.local_costmap_height = msg.info.height
        self.local_costmap_resolution = msg.info.resolution if msg.info.resolution > 0 else 0.05
        self.local_costmap_origin_x = msg.info.origin.position.x
        self.local_costmap_origin_y = msg.info.origin.position.y
        self.local_costmap_stamp = self.now_sec()

    def global_costmap_callback(self, msg: OccupancyGrid) -> None:
        self.global_costmap_data = list(msg.data)
        self.global_costmap_width = msg.info.width
        self.global_costmap_height = msg.info.height
        self.global_costmap_resolution = msg.info.resolution if msg.info.resolution > 0 else 0.05
        self.global_costmap_origin_x = msg.info.origin.position.x
        self.global_costmap_origin_y = msg.info.origin.position.y
        self.global_costmap_stamp = self.now_sec()

    def goal_callback(self, msg: PoseStamped) -> None:
        self.goal_pose = msg
        self.pending_goal = msg

        self.reset_runtime_for_new_goal(reset_history=True)

        self.log_status(
            f"GOAL_RECEIVED | frame={msg.header.frame_id} | x={msg.pose.position.x:.2f} | y={msg.pose.position.y:.2f}",
            level="warn",
            force=True,
        )

        self.generate_paths_to_goal(msg, reset_history=True)

    def plan_callback(self, msg: Path) -> None:
        """Use Nav2's global plan endpoint as CSA's goal source.

        This covers RViz NavigateToPose goals, which are sent through an
        action rather than /goal_pose.
        """
        if not msg.poses:
            return

        final_pose = msg.poses[-1]
        if self.goal_pose is not None:
            old_goal = self.goal_xy_in_map(self.goal_pose)
            new_goal = self.goal_xy_in_map(final_pose)
            if old_goal is not None and new_goal is not None and math.hypot(
                old_goal[0] - new_goal[0], old_goal[1] - new_goal[1]
            ) < 0.05:
                return

        self.goal_callback(final_pose)

    # =============================================================
    # TF
    # =============================================================
    def lookup_transform(self, target_frame: str, source_frame: str):
        return self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            Time(),
            timeout=Duration(seconds=self.tf_lookup_timeout_sec),
        )

    def get_robot_pose_map(self) -> Optional[Tuple[float, float, float]]:
        for frame in [self.base_frame, self.fallback_base_frame]:
            try:
                transform = self.lookup_transform(self.map_frame, frame)
                self.active_base_frame = frame

                x = transform.transform.translation.x
                y = transform.transform.translation.y
                yaw = yaw_from_quaternion(transform.transform.rotation)

                return x, y, yaw
            except TransformException:
                continue

        return None

    @staticmethod
    def transform_xy(transform, x: float, y: float) -> Point2D:
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        yaw = yaw_from_quaternion(transform.transform.rotation)

        cy = math.cos(yaw)
        sy = math.sin(yaw)

        out_x = tx + cy * x - sy * y
        out_y = ty + sy * x + cy * y

        return out_x, out_y

    @staticmethod
    def base_to_map_point(robot_pose_map: Tuple[float, float, float], bx: float, by: float) -> Point2D:
        rx, ry, ryaw = robot_pose_map

        mx = rx + math.cos(ryaw) * bx - math.sin(ryaw) * by
        my = ry + math.sin(ryaw) * bx + math.cos(ryaw) * by

        return mx, my

    def goal_base_coordinates(self, goal_msg: PoseStamped) -> Optional[Point2D]:
        goal_xy = self.goal_xy_in_map(goal_msg)
        if goal_xy is None:
            return None

        try:
            transform = self.lookup_transform(self.active_base_frame, self.map_frame)
        except TransformException:
            return None

        return self.transform_xy(transform, goal_xy[0], goal_xy[1])

    # =============================================================
    # MAP GRID
    # =============================================================
    def in_bounds(self, cell: GridCell) -> bool:
        x, y = cell
        return 0 <= x < self.map_width and 0 <= y < self.map_height

    def index(self, cell: GridCell) -> int:
        x, y = cell
        return y * self.map_width + x

    def is_occupied_cell(self, cell: GridCell) -> bool:
        if not self.in_bounds(cell):
            return True

        value = self.map_data[self.index(cell)]

        if value < 0:
            return self.unknown_is_obstacle

        return value >= self.occupied_threshold

    # STEP A - Costmap dinamis helpers
    def _costmap_world_to_idx(
        self, wx: float, wy: float,
        origin_x: float, origin_y: float,
        resolution: float, width: int, height: int,
    ) -> Optional[int]:
        cx = int((wx - origin_x) / resolution)
        cy = int((wy - origin_y) / resolution)
        if not (0 <= cx < width and 0 <= cy < height):
            return None
        return cy * width + cx

    def costmap_cell_cost(self, cell: GridCell, use_local: bool = True) -> int:
        """
        Kembalikan cost Nav2 costmap untuk sebuah sel (0-100, -1=unknown).
        Menggunakan local atau global costmap tergantung parameter use_local.
        Kembalikan 0 (bebas) jika costmap belum tersedia.
        """
        if not self.costmap_enabled:
            return 0

        if use_local:
            data = self.local_costmap_data
            if data is None:
                return 0
            ox, oy = self.local_costmap_origin_x, self.local_costmap_origin_y
            res = self.local_costmap_resolution
            w, h = self.local_costmap_width, self.local_costmap_height
        else:
            data = self.global_costmap_data
            if data is None:
                return 0
            ox, oy = self.global_costmap_origin_x, self.global_costmap_origin_y
            res = self.global_costmap_resolution
            w, h = self.global_costmap_width, self.global_costmap_height

        wx, wy = self.grid_to_world(cell)
        idx = self._costmap_world_to_idx(wx, wy, ox, oy, res, w, h)
        if idx is None:
            return 0
        return int(data[idx])

    def is_occupied_cell_costmap(self, cell: GridCell, use_local: bool = True) -> bool:
        """
        True jika costmap Nav2 menganggap sel ini terblokir (lethal atau inscribed).
        Sel yang hanya inflasi (cost < inscribed_threshold) dianggap masih bisa dilalui
        tapi akan mempengaruhi clearance estimate.
        """
        cost = self.costmap_cell_cost(cell, use_local=use_local)
        return cost >= self.costmap_occupied_threshold or cost < 0

    def clearance_from_costmap(self, cell: GridCell) -> float:
        """
        Estimasi clearance dari gradient costmap Nav2.
        Nav2 inflation layer mengisi sel sekitar obstacle dengan cost yang mengecil
        sesuai jarak (cost = 253 * e^(-k*d), k = cost_scaling_factor, default 10).
        Kita invert formula itu untuk estimasi jarak ke obstacle terdekat.
        Jika costmap tidak tersedia, kembalikan inf (tidak membatasi).
        """
        if not self.costmap_enabled or self.local_costmap_data is None:
            return float("inf")

        cost = self.costmap_cell_cost(cell, use_local=True)

        if cost <= 0:
            return float("inf")  # bebas

        if cost >= self.costmap_occupied_threshold:
            return 0.0  # terblokir

        # Estimasi jarak pakai inversi formula inflasi Nav2.
        # Nilai internal Nav2: 0-254; OccupancyGrid publish 0-100.
        # Rescale balik ke range internal 0-253:
        internal_cost = (cost / 100.0) * 253.0
        internal_cost = max(1.0, min(internal_cost, 252.0))

        cost_scaling_factor = 10.0  # Nav2 default
        try:
            d_estimate = -math.log(internal_cost / 253.0) / cost_scaling_factor
        except (ValueError, ZeroDivisionError):
            d_estimate = 0.0

        return max(0.0, d_estimate)

    def clearance_meter(self, cell: GridCell) -> float:
        if not self.in_bounds(cell):
            return 0.0

        if not self.distance_grid:
            return 0.0

        static_clearance = self.distance_grid[self.index(cell)] * self.map_resolution

        if not self.costmap_enabled or self.local_costmap_data is None or self.costmap_clearance_weight <= 0.0:
            return static_clearance

        # Gabungkan clearance statis + estimasi dari costmap (ambil minimum).
        costmap_clearance = self.clearance_from_costmap(cell)
        w = self.costmap_clearance_weight
        combined = (1.0 - w) * static_clearance + w * min(static_clearance, costmap_clearance)
        return max(0.0, combined)

    def dynamic_obstacle_clearance(self, cell: GridCell) -> float:
        if not self.blocked_obstacle_cells:
            return float("inf")

        cx, cy = cell
        min_distance = float("inf")
        for ox, oy in self.blocked_obstacle_cells:
            distance = math.hypot(cx - ox, cy - oy) * self.map_resolution
            min_distance = min(min_distance, distance)

        return min_distance - self.robot_radius

    def passable(self, cell: GridCell, required_clearance: Optional[float] = None) -> bool:
        if not self.in_bounds(cell):
            return False

        # Cek occupancy dari static map.
        if self.is_occupied_cell(cell):
            return False

        # STEP A: cek juga dari costmap dinamis Nav2.
        # Gunakan global costmap untuk path planning (obstacle yang nav2 sudah tahu),
        # gunakan local costmap untuk trajectory scoring (lebih real-time).
        if self.costmap_enabled:
            if self.use_global_costmap_for_planning and self.is_occupied_cell_costmap(cell, use_local=False):
                return False
            if self.use_local_costmap_for_scoring and self.is_occupied_cell_costmap(cell, use_local=True):
                return False

        if required_clearance is None:
            required_clearance = self.path_required_clearance

        return self.clearance_meter(cell) >= required_clearance

    def world_to_grid(self, x: float, y: float) -> Optional[GridCell]:
        gx = int((x - self.map_origin_x) / self.map_resolution)
        gy = int((y - self.map_origin_y) / self.map_resolution)
        cell = (gx, gy)
        if not self.in_bounds(cell):
            return None
        return cell

    def grid_to_world(self, cell: GridCell) -> Point2D:
        gx, gy = cell
        wx = self.map_origin_x + (gx + 0.5) * self.map_resolution
        wy = self.map_origin_y + (gy + 0.5) * self.map_resolution
        return wx, wy

    def compute_distance_grid(self) -> None:
        total = self.map_width * self.map_height
        self.distance_grid = [float("inf")] * total

        q = deque()

        for y in range(self.map_height):
            for x in range(self.map_width):
                cell = (x, y)
                idx = self.index(cell)

                if self.is_occupied_cell(cell):
                    self.distance_grid[idx] = 0.0
                    q.append(cell)

        neighbors = [
            (1, 0),
            (-1, 0),
            (0, 1),
            (0, -1),
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1),
        ]

        while q:
            cx, cy = q.popleft()
            cidx = self.index((cx, cy))

            for dx, dy in neighbors:
                nx = cx + dx
                ny = cy + dy
                ncell = (nx, ny)

                if not self.in_bounds(ncell):
                    continue

                nidx = self.index(ncell)
                step = math.sqrt(2.0) if dx != 0 and dy != 0 else 1.0
                new_dist = self.distance_grid[cidx] + step

                if new_dist < self.distance_grid[nidx]:
                    self.distance_grid[nidx] = new_dist
                    q.append(ncell)

    def nearest_free_cell(self, start: GridCell, max_radius_cells: Optional[int] = None) -> Optional[GridCell]:
        if max_radius_cells is None:
            max_radius_cells = self.nearest_free_max_radius_cells

        if self.in_bounds(start) and self.passable(start, required_clearance=self.nearest_free_required_clearance):
            return start

        q = deque([start])
        visited = {start}

        while q:
            cell = q.popleft()
            cx, cy = cell

            if abs(cx - start[0]) > max_radius_cells or abs(cy - start[1]) > max_radius_cells:
                continue

            if self.in_bounds(cell) and self.passable(cell, required_clearance=self.nearest_free_required_clearance):
                return cell

            for dx, dy in [
                (1, 0),
                (-1, 0),
                (0, 1),
                (0, -1),
                (1, 1),
                (1, -1),
                (-1, 1),
                (-1, -1),
            ]:
                ncell = (cx + dx, cy + dy)

                if ncell not in visited:
                    visited.add(ncell)
                    q.append(ncell)

        return None

    # =============================================================
    # PATH PENALTY HELPERS
    # =============================================================
    def add_penalty_around_cell(
        self,
        penalty_grid: Dict[GridCell, float],
        center: GridCell,
        radius_cells: int,
        penalty_value: float,
    ) -> None:
        cx, cy = center

        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue

                cell = (cx + dx, cy + dy)

                if not self.in_bounds(cell):
                    continue

                distance = math.sqrt(dx * dx + dy * dy)
                weight = penalty_value * (1.0 - distance / max(radius_cells, 1))
                penalty_grid[cell] = penalty_grid.get(cell, 0.0) + max(weight, 0.0)

    def add_path_penalty(
        self,
        penalty_grid: Dict[GridCell, float],
        path: List[GridCell],
        radius_m: float,
        penalty_value: float,
    ) -> None:
        radius_cells = max(1, int(radius_m / self.map_resolution))
        stride = max(1, len(path) // self.path_penalty_stride_divisor)

        for cell in path[::stride]:
            self.add_penalty_around_cell(
                penalty_grid,
                cell,
                radius_cells=radius_cells,
                penalty_value=penalty_value,
            )

    def build_penalty_grid(self) -> Dict[GridCell, float]:
        penalty_grid: Dict[GridCell, float] = {}

        for path in self.blocked_path_history:
            # Jalur lama yang gagal dibuat mahal agar path baru tidak melewati area yang sama.
            self.add_path_penalty(
                penalty_grid,
                path,
                radius_m=self.blocked_history_penalty_radius_m,
                penalty_value=self.blocked_history_penalty_base + self.blocked_history_penalty_regen_gain * self.regeneration_count,
            )

        for cell in self.blocked_obstacle_cells:
            self.add_penalty_around_cell(
                penalty_grid,
                cell,
                radius_cells=max(2, int(self.blocked_obstacle_penalty_radius_m / self.map_resolution)),
                penalty_value=self.blocked_obstacle_penalty_value,
            )

        return penalty_grid

    # =============================================================
    # GLOBAL CSA PATH PLANNER
    # =============================================================
    def route_profile_for_candidate(self, candidate_index: int) -> Tuple[str, float]:
        """
        Path 1: direct profile.
        Path 2: left detour profile.
        Path 3: right detour profile.
        Path >3: alternating wider left/right detours.
        """
        if candidate_index <= 0:
            return "DIRECT", 0.0
        if candidate_index == 1:
            return "LEFT_DETOUR", self.global_csa_side_offset_m
        if candidate_index == 2:
            return "RIGHT_DETOUR", -self.global_csa_side_offset_m

        level = 1 + (candidate_index - 1) // 2
        sign = 1.0 if candidate_index % 2 == 1 else -1.0
        name = "LEFT_WIDE_DETOUR" if sign > 0.0 else "RIGHT_WIDE_DETOUR"
        return name, sign * self.global_csa_side_offset_m * float(level)

    def line_basis_world(self, start_world: Point2D, goal_world: Point2D) -> Tuple[Point2D, Point2D, float]:
        sx, sy = start_world
        gx, gy = goal_world
        dx = gx - sx
        dy = gy - sy
        length = math.hypot(dx, dy)

        if length < 1e-9:
            return (1.0, 0.0), (0.0, 1.0), 1e-9

        tangent = (dx / length, dy / length)
        normal = (-tangent[1], tangent[0])
        return tangent, normal, length

    def signed_lateral_distance(
        self,
        point_world: Point2D,
        start_world: Point2D,
        normal: Point2D,
    ) -> float:
        px, py = point_world
        sx, sy = start_world
        nx, ny = normal
        return (px - sx) * nx + (py - sy) * ny

    def clamp_world_point_to_map(self, point_world: Point2D) -> Point2D:
        if self.map_width <= 0 or self.map_height <= 0:
            return point_world

        x, y = point_world
        gx = int((x - self.map_origin_x) / self.map_resolution)
        gy = int((y - self.map_origin_y) / self.map_resolution)
        gx = max(0, min(self.map_width - 1, gx))
        gy = max(0, min(self.map_height - 1, gy))
        return self.grid_to_world((gx, gy))

    def repair_world_point_to_free(self, point_world: Point2D) -> Point2D:
        point_world = self.clamp_world_point_to_map(point_world)
        cell = self.world_to_grid(point_world[0], point_world[1])

        if cell is None:
            return point_world

        if self.passable(cell, required_clearance=self.nearest_free_required_clearance):
            return point_world

        free_cell = self.nearest_free_cell(cell, max_radius_cells=self.nearest_free_max_radius_cells)
        if free_cell is None:
            return point_world

        return self.grid_to_world(free_cell)

    def make_initial_csa_nest(
        self,
        start_world: Point2D,
        goal_world: Point2D,
        side_target_m: float,
        jitter_scale: float,
    ) -> List[Point2D]:
        tangent, normal, length = self.line_basis_world(start_world, goal_world)
        sx, sy = start_world
        tx, ty = tangent
        nx, ny = normal

        waypoints: List[Point2D] = []
        for i in range(self.global_csa_waypoints):
            ratio = (i + 1) / float(self.global_csa_waypoints + 1)
            lateral_shape = math.sin(math.pi * ratio)

            base_x = sx + tx * length * ratio
            base_y = sy + ty * length * ratio

            along_noise = random.gauss(0.0, 0.22 * jitter_scale)
            lateral_noise = random.gauss(0.0, jitter_scale)
            lateral = side_target_m * lateral_shape + lateral_noise

            wx = base_x + tx * along_noise + nx * lateral
            wy = base_y + ty * along_noise + ny * lateral
            waypoints.append(self.repair_world_point_to_free((wx, wy)))

        return waypoints

    def sample_segment_cells_world(self, a: Point2D, b: Point2D) -> List[GridCell]:
        ax, ay = a
        bx, by = b
        dist = math.hypot(bx - ax, by - ay)
        steps = max(1, int(dist / self.global_csa_sampling_step_m))

        cells: List[GridCell] = []
        last_cell: Optional[GridCell] = None
        for k in range(steps + 1):
            t = k / float(steps)
            x = ax + t * (bx - ax)
            y = ay + t * (by - ay)
            cell = self.world_to_grid(x, y)
            if cell is None:
                continue
            if cell != last_cell:
                cells.append(cell)
                last_cell = cell

        return cells

    def cells_from_waypoint_path(self, points_world: List[Point2D]) -> List[GridCell]:
        if len(points_world) < 2:
            return []

        out: List[GridCell] = []
        last_cell: Optional[GridCell] = None

        for i in range(1, len(points_world)):
            segment_cells = self.sample_segment_cells_world(points_world[i - 1], points_world[i])
            for cell in segment_cells:
                if cell != last_cell:
                    out.append(cell)
                    last_cell = cell

        return out

    def global_csa_path_metrics(
        self,
        nest: List[Point2D],
        start_world: Point2D,
        goal_world: Point2D,
        side_target_m: float,
        penalty_grid: Dict[GridCell, float],
        accepted_paths: List[List[GridCell]],
    ) -> Tuple[float, List[GridCell], Dict[str, float]]:
        points_world = [start_world] + nest + [goal_world]
        cells = self.cells_from_waypoint_path(points_world)

        if not cells:
            return float("inf"), [], {
                "length_m": 0.0,
                "turn_sum": 0.0,
                "min_clearance": 0.0,
                "collision_count": 999.0,
                "outside_count": 999.0,
                "avg_lateral": 0.0,
            }

        length_m = 0.0
        turn_sum = 0.0
        previous_angle = None

        for i in range(1, len(points_world)):
            x0, y0 = points_world[i - 1]
            x1, y1 = points_world[i]
            length_m += math.hypot(x1 - x0, y1 - y0)
            angle = math.atan2(y1 - y0, x1 - x0)
            if previous_angle is not None:
                turn_sum += abs(normalize_angle(angle - previous_angle))
            previous_angle = angle

        obstacle_cost = 0.0
        history_cost = 0.0
        min_clearance = float("inf")
        collision_count = 0
        outside_count = 0

        for cell in cells:
            if not self.in_bounds(cell):
                outside_count += 1
                continue

            clearance = self.clearance_meter(cell)
            dynamic_clearance = self.dynamic_obstacle_clearance(cell)
            min_clearance = min(min_clearance, clearance)
            obstacle_cost += 1.0 / max(clearance, self.global_csa_min_clearance_floor_m)
            history_cost += penalty_grid.get(cell, 0.0)

            if (
                self.is_occupied_cell(cell)
                or clearance < self.path_required_clearance
                or dynamic_clearance < self.path_required_clearance
            ):
                collision_count += 1

        obstacle_cost /= max(1, len(cells))
        history_cost /= max(1, len(cells))

        tangent, normal, _ = self.line_basis_world(start_world, goal_world)
        del tangent
        lateral_values = [
            self.signed_lateral_distance(wp, start_world, normal)
            for wp in nest
        ]
        avg_lateral = sum(lateral_values) / max(1, len(lateral_values))

        if abs(side_target_m) < 1e-9:
            side_cost = 0.35 * self.global_csa_side_weight * abs(avg_lateral)
        else:
            side_cost = self.global_csa_side_weight * abs(avg_lateral - side_target_m)

        diversity_cost = 0.0
        for other_path in accepted_paths:
            similarity = self.path_similarity(cells, other_path)
            if similarity > self.path_similarity_threshold:
                diversity_cost += self.global_csa_similarity_penalty * similarity

        for blocked in self.blocked_path_history:
            similarity = self.path_similarity(cells, blocked)
            if similarity > self.blocked_history_similarity_threshold:
                diversity_cost += self.global_csa_similarity_penalty * similarity

        collision_cost = self.global_csa_collision_penalty * collision_count
        outside_cost = self.global_csa_outside_map_penalty * outside_count

        total_cost = (
            self.global_csa_length_weight * length_m
            + self.global_csa_smoothness_weight * turn_sum
            + self.global_csa_obstacle_weight * obstacle_cost
            + self.global_csa_history_weight * history_cost
            + side_cost
            + diversity_cost
            + collision_cost
            + outside_cost
        )

        metrics = {
            "length_m": length_m,
            "turn_sum": turn_sum,
            "min_clearance": 0.0 if min_clearance == float("inf") else min_clearance,
            "collision_count": float(collision_count),
            "outside_count": float(outside_count),
            "avg_lateral": avg_lateral,
            "side_cost": side_cost,
            "diversity_cost": diversity_cost,
        }
        return total_cost, cells, metrics

    def tournament_select_global(self, scored_population: List[Tuple[float, List[Point2D], List[GridCell], Dict[str, float]]]) -> List[Point2D]:
        if not scored_population:
            return []

        k = min(self.global_csa_tournament_size, len(scored_population))
        contestants = random.sample(scored_population, k=k)
        contestants.sort(key=lambda item: item[0])
        return [(x, y) for x, y in contestants[0][1]]

    def mutate_global_nest(
        self,
        parent: List[Point2D],
        best: List[Point2D],
        start_world: Point2D,
        goal_world: Point2D,
        side_target_m: float,
    ) -> List[Point2D]:
        tangent, normal, length = self.line_basis_world(start_world, goal_world)
        sx, sy = start_world
        tx, ty = tangent
        nx, ny = normal

        child: List[Point2D] = []
        for i, point in enumerate(parent):
            ratio = (i + 1) / float(self.global_csa_waypoints + 1)
            lateral_shape = math.sin(math.pi * ratio)
            base_x = sx + tx * length * ratio + nx * side_target_m * lateral_shape
            base_y = sy + ty * length * ratio + ny * side_target_m * lateral_shape

            bx, by = best[i]
            px, py = point

            # Tournament-selected parent is pulled toward the current best nest,
            # then perturbed by Levy flight. This keeps exploitation and exploration.
            levy = self.levy_step()
            levy_step = clamp(levy * self.global_csa_levy_step_m, -3.0 * self.global_csa_levy_step_m, 3.0 * self.global_csa_levy_step_m)
            gaussian_step = random.gauss(0.0, self.global_csa_gaussian_step_m)
            direction = random.uniform(-math.pi, math.pi)

            x = px + 0.35 * (bx - px) + 0.08 * (base_x - px) + math.cos(direction) * levy_step + nx * gaussian_step
            y = py + 0.35 * (by - py) + 0.08 * (base_y - py) + math.sin(direction) * levy_step + ny * gaussian_step
            child.append(self.repair_world_point_to_free((x, y)))

        return child

    def run_global_csa_for_profile(
        self,
        start_world: Point2D,
        goal_world: Point2D,
        profile_name: str,
        side_target_m: float,
        penalty_grid: Dict[GridCell, float],
        accepted_paths: List[List[GridCell]],
    ) -> Optional[Tuple[List[GridCell], float, Dict[str, float]]]:
        population: List[List[Point2D]] = []

        if self.global_csa_direct_seed_enabled:
            population.append(self.make_initial_csa_nest(start_world, goal_world, side_target_m, jitter_scale=0.0))

        seed_offsets = [0.50, 1.00, 1.50, -0.50, -1.00]
        for scale in seed_offsets:
            if len(population) >= self.global_csa_population:
                break
            if abs(side_target_m) < 1e-9:
                seed_side = scale * 0.35 * self.global_csa_side_offset_m
            else:
                seed_side = side_target_m * abs(scale)
            population.append(self.make_initial_csa_nest(start_world, goal_world, seed_side, jitter_scale=0.05))

        while len(population) < self.global_csa_population:
            jitter = random.uniform(0.10, max(0.15, self.global_csa_side_offset_m))
            population.append(self.make_initial_csa_nest(start_world, goal_world, side_target_m, jitter_scale=jitter))

        scored_population: List[Tuple[float, List[Point2D], List[GridCell], Dict[str, float]]] = []
        for nest in population:
            cost, cells, metrics = self.global_csa_path_metrics(nest, start_world, goal_world, side_target_m, penalty_grid, accepted_paths)
            scored_population.append((cost, nest, cells, metrics))

        for _ in range(self.global_csa_iterations):
            scored_population.sort(key=lambda item: item[0])
            best_nest = scored_population[0][1]

            next_population: List[List[Point2D]] = [[(x, y) for x, y in best_nest]]

            while len(next_population) < self.global_csa_population:
                parent = self.tournament_select_global(scored_population)
                child = self.mutate_global_nest(parent, best_nest, start_world, goal_world, side_target_m)
                next_population.append(child)

            scored_population = []
            for nest in next_population:
                cost, cells, metrics = self.global_csa_path_metrics(nest, start_world, goal_world, side_target_m, penalty_grid, accepted_paths)
                scored_population.append((cost, nest, cells, metrics))

            scored_population.sort(key=lambda item: item[0])
            abandon_count = int(self.global_csa_pa * self.global_csa_population)
            for i in range(max(0, self.global_csa_population - abandon_count), self.global_csa_population):
                jitter = random.uniform(0.15, max(0.20, 1.25 * self.global_csa_side_offset_m))
                new_nest = self.make_initial_csa_nest(start_world, goal_world, side_target_m, jitter_scale=jitter)
                cost, cells, metrics = self.global_csa_path_metrics(new_nest, start_world, goal_world, side_target_m, penalty_grid, accepted_paths)
                scored_population[i] = (cost, new_nest, cells, metrics)

        scored_population.sort(key=lambda item: item[0])
        best_cost, _, best_cells, best_metrics = scored_population[0]

        if not best_cells:
            self.log_status(
                f"GLOBAL_CSA_PROFILE_FAILED | profile={profile_name} | reason=empty_path",
                level="warn",
                force=True,
            )
            return None

        if best_metrics.get("collision_count", 0.0) > 0.0 or best_metrics.get("outside_count", 0.0) > 0.0:
            self.log_status(
                (
                    f"GLOBAL_CSA_PROFILE_REJECTED | profile={profile_name} | "
                    f"cost={best_cost:.2f} | collision={best_metrics.get('collision_count', 0.0):.0f} | "
                    f"outside={best_metrics.get('outside_count', 0.0):.0f}"
                ),
                level="warn",
                force=True,
            )
            return None

        self.log_status(
            (
                f"GLOBAL_CSA_PROFILE_OK | profile={profile_name} | cost={best_cost:.2f} | "
                f"length={best_metrics.get('length_m', 0.0):.2f}m | "
                f"min_clear={best_metrics.get('min_clearance', 0.0):.2f}m | "
                f"avg_lat={best_metrics.get('avg_lateral', 0.0):.2f}m"
            ),
            level="info",
            force=True,
        )

        return best_cells, best_cost, best_metrics

    def generate_csa_global_candidate_paths(
        self,
        start_cell: GridCell,
        goal_cell: GridCell,
        penalty_grid: Dict[GridCell, float],
    ) -> List[List[GridCell]]:
        start_world = self.grid_to_world(start_cell)
        goal_world = self.grid_to_world(goal_cell)
        candidate_paths: List[List[GridCell]] = []

        attempts_per_profile = max(1, self.path_generation_attempt_multiplier)

        for candidate_index in range(self.max_candidate_paths):
            profile_name, base_side_target = self.route_profile_for_candidate(candidate_index)
            accepted_for_this_profile = False

            for attempt in range(attempts_per_profile):
                side_target = base_side_target
                if candidate_index > 0:
                    side_target = base_side_target * (1.0 + 0.25 * attempt)

                result = self.run_global_csa_for_profile(
                    start_world,
                    goal_world,
                    profile_name,
                    side_target,
                    penalty_grid,
                    candidate_paths,
                )

                if result is None:
                    continue

                cells, _, _ = result
                if self.path_too_similar_to_existing(cells, candidate_paths):
                    self.log_status(
                        f"GLOBAL_CSA_PROFILE_TOO_SIMILAR | profile={profile_name} | attempt={attempt + 1}",
                        level="warn",
                        force=True,
                    )
                    continue

                if self.path_too_similar_to_blocked_history(cells):
                    self.log_status(
                        f"GLOBAL_CSA_PROFILE_MATCHES_BLOCKED_HISTORY | profile={profile_name} | attempt={attempt + 1}",
                        level="warn",
                        force=True,
                    )
                    continue

                candidate_paths.append(cells)
                accepted_for_this_profile = True
                break

            if not accepted_for_this_profile:
                self.log_status(
                    f"GLOBAL_CSA_PROFILE_NOT_ACCEPTED | profile={profile_name}",
                    level="warn",
                    force=True,
                )

        return candidate_paths

    def levy_step(self) -> float:
        num = math.gamma(1.0 + self.beta) * math.sin(math.pi * self.beta / 2.0)
        den = (
            math.gamma((1.0 + self.beta) / 2.0)
            * self.beta
            * (2.0 ** ((self.beta - 1.0) / 2.0))
        )
        sigma = (num / den) ** (1.0 / self.beta)

        u = random.gauss(0.0, sigma)
        v = random.gauss(0.0, 1.0)

        if abs(v) < 1e-9:
            v = 1e-9

        return u / (abs(v) ** (1.0 / self.beta))

    def apply_levy_flight_penalty(
        self,
        penalty_grid: Dict[GridCell, float],
        start: GridCell,
        goal: GridCell,
        count: int,
    ) -> None:
        sx, sy = start
        gx, gy = goal

        base_angle = math.atan2(gy - sy, gx - sx)

        for _ in range(count):
            ratio = random.uniform(self.levy_ratio_min, self.levy_ratio_max)
            along_x = sx + ratio * (gx - sx)
            along_y = sy + ratio * (gy - sy)

            side = random.choice([-1.0, 1.0])
            lateral_angle = base_angle + side * math.pi / 2.0

            step = self.levy_step()
            lateral_cells = clamp(abs(step) * self.levy_lateral_gain_cells, self.levy_lateral_min_cells, self.levy_lateral_max_cells)

            px = int(along_x + lateral_cells * math.cos(lateral_angle))
            py = int(along_y + lateral_cells * math.sin(lateral_angle))
            cell = (px, py)

            if not self.in_bounds(cell):
                continue

            self.add_penalty_around_cell(
                penalty_grid,
                cell,
                radius_cells=max(2, int(self.levy_penalty_radius_m / self.map_resolution)),
                penalty_value=random.uniform(self.levy_penalty_min, self.levy_penalty_max),
            )

    def reduce_path_corners(self, raw_path: List[GridCell]) -> List[GridCell]:
        if len(raw_path) <= 2:
            return raw_path

        reduced = [raw_path[0]]
        prev_dir = None

        for i in range(1, len(raw_path)):
            x0, y0 = raw_path[i - 1]
            x1, y1 = raw_path[i]

            direction = (x1 - x0, y1 - y0)

            if prev_dir is not None and direction != prev_dir:
                reduced.append(raw_path[i - 1])

            prev_dir = direction

        reduced.append(raw_path[-1])

        if len(reduced) < 3 and len(raw_path) >= 3:
            mid = raw_path[len(raw_path) // 2]
            reduced.insert(1, mid)

        return reduced

    def densify_world_points(self, points: List[Point2D], step: Optional[float] = None) -> List[Point2D]:
        if step is None:
            step = self.path_densify_step_m
        if len(points) < 2:
            return points

        dense = [points[0]]

        for i in range(1, len(points)):
            x0, y0 = points[i - 1]
            x1, y1 = points[i]

            dist = math.hypot(x1 - x0, y1 - y0)
            n = max(1, int(dist / step))

            for k in range(1, n + 1):
                t = k / float(n)
                x = x0 + t * (x1 - x0)
                y = y0 + t * (y1 - y0)
                dense.append((x, y))

        return dense

    def path_similarity(self, a: List[GridCell], b: List[GridCell]) -> float:
        if not a or not b:
            return 0.0

        set_a = set(a)
        set_b = set(b)

        overlap = len(set_a & set_b)
        base = max(1, min(len(set_a), len(set_b)))

        return overlap / float(base)

    def path_too_similar_to_existing(self, path: List[GridCell], existing: List[List[GridCell]]) -> bool:
        for other in existing:
            if self.path_similarity(path, other) > self.path_similarity_threshold:
                return True

        return False

    def path_too_similar_to_blocked_history(self, path: List[GridCell]) -> bool:
        for blocked in self.blocked_path_history:
            if self.path_similarity(path, blocked) > self.blocked_history_similarity_threshold:
                return True

        return False

    def score_path(self, path: List[GridCell]) -> float:
        if len(path) < 2:
            return float("inf")

        length_m = 0.0
        turn_sum = 0.0
        min_clearance = float("inf")
        avg_clearance = 0.0

        previous_angle = None

        for i in range(1, len(path)):
            x0, y0 = path[i - 1]
            x1, y1 = path[i]

            step_m = math.hypot(x1 - x0, y1 - y0) * self.map_resolution
            length_m += step_m

            angle = math.atan2(y1 - y0, x1 - x0)

            if previous_angle is not None:
                turn_sum += abs(normalize_angle(angle - previous_angle))

            previous_angle = angle

            clearance = self.clearance_meter(path[i])
            min_clearance = min(min_clearance, clearance)
            avg_clearance += clearance

        avg_clearance /= max(1, len(path) - 1)

        cost = (
            self.path_score_length_weight * length_m
            + self.path_score_turn_weight * turn_sum
            + self.path_score_min_clearance_weight / max(min_clearance, self.path_score_clearance_floor_m)
            + self.path_score_avg_clearance_weight / max(avg_clearance, self.path_score_clearance_floor_m)
        )

        return cost

    def goal_xy_in_map(self, goal_msg: PoseStamped) -> Optional[Point2D]:
        goal_frame = goal_msg.header.frame_id if goal_msg.header.frame_id else self.map_frame

        if goal_frame != self.map_frame:
            try:
                transform = self.lookup_transform(self.map_frame, goal_frame)
                gx, gy = self.transform_xy(
                    transform,
                    goal_msg.pose.position.x,
                    goal_msg.pose.position.y,
                )
                return gx, gy
            except TransformException:
                return None

        return goal_msg.pose.position.x, goal_msg.pose.position.y

    def generate_paths_to_goal(self, goal_msg: PoseStamped, reset_history: bool) -> None:
        if self.map_msg is None:
            self.log_status("WAIT_MAP_FOR_PATH | goal disimpan dulu.", level="warn", force=True)
            self.pending_goal = goal_msg
            return

        robot_pose = self.get_robot_pose_map()

        if robot_pose is None:
            self.log_status("WAIT_TF_FOR_PATH | set initial pose dulu.", level="warn", force=True)
            self.pending_goal = goal_msg
            return

        if reset_history:
            self.blocked_path_history = []
            self.blocked_obstacle_cells = []
            self.regeneration_count = 0

        rx, ry, _ = robot_pose
        goal_xy = self.goal_xy_in_map(goal_msg)

        if goal_xy is None:
            self.pending_goal = goal_msg
            return

        gx, gy = goal_xy

        start_cell = self.world_to_grid(rx, ry)
        goal_cell = self.world_to_grid(gx, gy)

        if start_cell is None:
            self.log_status(f"ROBOT_OUTSIDE_MAP | x={rx:.2f} | y={ry:.2f}", level="error", force=True)
            self.stop_robot()
            return

        if goal_cell is None:
            self.log_status(f"GOAL_OUTSIDE_MAP | x={gx:.2f} | y={gy:.2f}", level="error", force=True)
            self.stop_robot()
            return

        start_cell = self.nearest_free_cell(start_cell) or start_cell
        goal_cell = self.nearest_free_cell(goal_cell) or goal_cell

        penalty_grid = self.build_penalty_grid()

        if self.regeneration_count > 0:
            self.apply_levy_flight_penalty(
                penalty_grid,
                start_cell,
                goal_cell,
                count=self.levy_regen_count_base + self.levy_regen_count_gain * self.regeneration_count,
            )

        candidate_raw_paths = self.generate_csa_global_candidate_paths(
            start_cell,
            goal_cell,
            penalty_grid,
        )

        if not candidate_raw_paths:
            self.stop_robot()
            self.state = "NO_PATH"
            self.log_status(
                f"NO_PATH | tidak ada candidate path valid | regen={self.regeneration_count}",
                level="error",
                force=True,
            )
            return

        # Jangan sort ulang candidate_paths. Urutan topic harus tetap bermakna:
        # candidate_path_1 = DIRECT, candidate_path_2 = LEFT_DETOUR, candidate_path_3 = RIGHT_DETOUR.
        # Path yang dipakai tetap dipilih berdasarkan cost minimum.
        scored = [(self.score_path(path), path) for path in candidate_raw_paths]
        best_path_index = min(range(len(scored)), key=lambda idx: scored[idx][0])

        self.candidate_paths_cells = []
        self.candidate_paths_world = []
        self.candidate_costs = []

        for cost, raw_path in scored:
            reduced = self.reduce_path_corners(raw_path)
            world = [self.grid_to_world(cell) for cell in reduced]
            dense_world = self.densify_world_points(world)

            self.candidate_paths_cells.append(reduced)
            self.candidate_paths_world.append(dense_world)
            self.candidate_costs.append(cost)

        self.path_blocked = [False for _ in self.candidate_paths_cells]
        self.selected_path_index = best_path_index
        self.path_progress_index = self.find_nearest_index_on_path(
            robot_pose,
            self.candidate_paths_world[self.selected_path_index],
            global_search=True,
        )

        self.reverse_count_current_path = 0
        self.reverse_ticks = 0
        self.uturn_ticks = 0
        self.block_confirm_ticks = 0
        self.block_confirm_reason = "RESET_AFTER_PATH_GENERATION"
        self.trap_escape_ticks = 0
        self.trap_escape_reverse_ticks = 0
        self.trap_escape_reason = "RESET_AFTER_PATH_GENERATION"
        self.recovery_cooldown_ticks = 0

        self.last_goal_distance = None
        self.stall_start_time = None
        self.no_valid_start_time = None
        self.prev_v_cmd = 0.0
        self.prev_w_cmd = 0.0
        self.auto_switch_cooldown_ticks = 0

        # Putar balik dulu jika titik masuk JALUR terpilih ATAU goal berada di
        # belakang robot. Tujuannya: robot menghadap & mendekati jalur dulu,
        # baru mengikuti jalur menuju goal.
        path_behind = self.path_entry_is_behind_robot()
        goal_behind = self.goal_is_behind_robot(goal_msg)
        self.initial_turn_done = not (path_behind or goal_behind)

        if self.initial_turn_done:
            self.state = "FOLLOW_PATH"
        else:
            self.state = "INITIAL_GOAL_TURN"
            self.initial_turn_ticks = self.initial_turn_tick_limit
            turn_target = self.initial_turn_target_base()
            if turn_target is not None:
                self.initial_turn_direction = 1.0 if turn_target[1] >= 0.0 else -1.0
            else:
                self.initial_turn_direction = 1.0

        self.publish_candidate_paths()
        self.publish_selected_path()

        if self.regeneration_count == 0:
            self.log_status("PATH_SET_CREATED | memakai candidate path awal", level="warn", force=True)
        else:
            self.log_status(
                (
                    f"NEW_PATH_SET_CREATED_BY_LEVY | regen={self.regeneration_count} | "
                    f"blocked_history_paths={len(self.blocked_path_history)} | "
                    f"policy=avoid_previous_failed_paths"
                ),
                level="warn",
                force=True,
            )

        for i, cost in enumerate(self.candidate_costs):
            self.log_status(
                f"PATH_{i + 1}_READY | cost={cost:.3f} | points={len(self.candidate_paths_world[i])}",
                level="info",
                force=True,
            )

        if self.initial_turn_done:
            self.log_status(
                f"SELECT_PATH | selected={self.selected_path_index + 1}/{len(self.candidate_paths_world)} | reason=lowest_cost_csa | idx={self.path_progress_index}",
                level="warn",
                force=True,
            )
        else:
            self.log_status(
                f"INITIAL_GOAL_TURN_START | after_turn_selected={self.selected_path_index + 1}/{len(self.candidate_paths_world)}",
                level="warn",
                force=True,
            )

    def regenerate_paths_after_all_blocked(self, reason: str) -> None:
        if self.goal_pose is None:
            self.stop_robot()
            return

        if self.regeneration_count >= self.max_regeneration_count:
            self.stop_robot()
            self.state = "ALL_REGEN_FAILED"
            self.log_status(
                f"ALL_REGEN_FAILED | regen_count={self.regeneration_count} | reason={reason}",
                level="error",
                force=True,
            )
            return

        old_path_count = len(self.candidate_paths_cells)

        # Saat obstacle dinamis muncul, hanya path aktif yang dianggap gagal.
        # Candidate kiri/kanan lama tetap boleh menjadi basis jalur alternatif
        # karena bagian start dan goal memang pasti overlap.
        paths_to_remember = self.candidate_paths_cells
        if reason.startswith("dynamic_obstacle") and self.selected_path_index is not None:
            paths_to_remember = [self.candidate_paths_cells[self.selected_path_index]]

        for old_path in paths_to_remember:
            if old_path and old_path not in self.blocked_path_history:
                self.blocked_path_history.append(old_path)

        if len(self.blocked_path_history) > 60:
            self.blocked_path_history = self.blocked_path_history[-60:]

        self.regeneration_count += 1

        self.stop_robot()

        self.log_status(
            (
                f"STOP_AND_GENERATE_NEW_PATHS | count={self.max_candidate_paths} | reason={reason} | "
                f"old_paths_added_to_blocked_history={old_path_count} | "
                f"regen={self.regeneration_count}/{self.max_regeneration_count}"
            ),
            level="warn",
            force=True,
        )

        self.candidate_paths_cells = []
        self.candidate_paths_world = []
        self.candidate_costs = []
        self.path_blocked = []
        self.selected_path_index = None
        self.path_progress_index = 0

        self.generate_paths_to_goal(self.goal_pose, reset_history=False)

    # =============================================================
    # PATH PUBLISHING / ACTIVE PATH
    # =============================================================
    def cells_to_path_msg(self, cells: List[GridCell], world_path: Optional[List[Point2D]] = None) -> Path:
        msg = Path()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()

        points = world_path

        if points is None:
            raw_world = [self.grid_to_world(cell) for cell in cells]
            points = self.densify_world_points(raw_world)

        for wx, wy in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        return msg

    def publish_candidate_paths(self) -> None:
        for i, pub in enumerate(self.candidate_path_pubs):
            if i < len(self.candidate_paths_cells):
                pub.publish(
                    self.cells_to_path_msg(
                        self.candidate_paths_cells[i],
                        self.candidate_paths_world[i],
                    )
                )

    def publish_selected_path(self) -> None:
        if self.selected_path_index is None:
            return

        if self.selected_path_index >= len(self.candidate_paths_cells):
            return

        self.selected_path_pub.publish(
            self.cells_to_path_msg(
                self.candidate_paths_cells[self.selected_path_index],
                self.candidate_paths_world[self.selected_path_index],
            )
        )

    def active_path_world(self) -> Optional[List[Point2D]]:
        if self.selected_path_index is None:
            return None

        if self.selected_path_index >= len(self.candidate_paths_world):
            return None

        return self.candidate_paths_world[self.selected_path_index]

    def active_path_cells(self) -> Optional[List[GridCell]]:
        if self.selected_path_index is None:
            return None

        if self.selected_path_index >= len(self.candidate_paths_cells):
            return None

        return self.candidate_paths_cells[self.selected_path_index]

    def find_nearest_index_on_path(
        self,
        robot_pose_map: Tuple[float, float, float],
        path_world: List[Point2D],
        global_search: bool,
    ) -> int:
        if not path_world:
            return 0

        rx, ry, _ = robot_pose_map

        if global_search:
            start = 0
            end = len(path_world)
        else:
            start = max(0, self.path_progress_index - 3)
            end = min(len(path_world), self.path_progress_index + 30)

        best_idx = start
        best_dist = float("inf")

        for i in range(start, end):
            px, py = path_world[i]
            d = math.hypot(px - rx, py - ry)

            if d < best_dist:
                best_dist = d
                best_idx = i

        return best_idx

    def distance_to_world_path(
        self,
        robot_pose_map: Tuple[float, float, float],
        path_world: List[Point2D],
    ) -> Tuple[float, int]:
        if not path_world:
            return float("inf"), 0

        rx, ry, _ = robot_pose_map

        best_dist = float("inf")
        best_idx = 0

        for i, (px, py) in enumerate(path_world):
            d = math.hypot(px - rx, py - ry)

            if d < best_dist:
                best_dist = d
                best_idx = i

        return best_dist, best_idx

    def estimate_wheel_speed(self, v: float, w: float) -> Tuple[float, float, float, float]:
        left_linear = v - (w * self.wheel_separation_m / 2.0)
        right_linear = v + (w * self.wheel_separation_m / 2.0)

        circumference = 2.0 * math.pi * self.wheel_radius_m

        if circumference <= 1e-9:
            return left_linear, right_linear, 0.0, 0.0

        left_rpm = (left_linear / circumference) * 60.0
        right_rpm = (right_linear / circumference) * 60.0

        return left_linear, right_linear, left_rpm, right_rpm

    def wheel_motion_decision(self, v: float, w: float) -> str:
        if abs(v) < 0.03 and abs(w) < 0.05:
            return "STOP"

        if v < -0.03:
            return "REVERSE"

        if abs(v) < 0.03 and w > 0.05:
            return "SPIN_LEFT"

        if abs(v) < 0.03 and w < -0.05:
            return "SPIN_RIGHT"

        if w > 0.12:
            return "FORWARD_TURN_LEFT"

        if w < -0.12:
            return "FORWARD_TURN_RIGHT"

        return "FORWARD_STRAIGHT"

    def obstacle_decision_text(
        self,
        front_min: float,
        left_min: float,
        right_min: float,
        valid_count: int,
    ) -> str:
        front_obstacle = front_min < self.soft_obstacle_distance
        hard_front = front_min < self.hard_obstacle_distance
        full_block = self.is_full_blocked_geometry(front_min, left_min, right_min)

        if full_block:
            decision = "FULL_BLOCK_CANDIDATE"
        elif hard_front:
            decision = "HARD_AVOID"
        elif front_obstacle:
            decision = "SOFT_AVOID"
        else:
            decision = "CLEAR_FOLLOW_PATH"

        return (
            f"front_obstacle={'YES' if front_obstacle else 'NO'} | "
            f"avoid_decision={decision} | "
            f"front={front_min:.2f}m | left={left_min:.2f}m | right={right_min:.2f}m | "
            f"valid={valid_count}/{self.pop_size}"
        )

    def path_distance_report(
        self,
        robot_pose_map: Tuple[float, float, float],
    ) -> Tuple[str, int, float, float]:
        if not self.candidate_paths_world:
            return "near={}", -1, float("inf"), float("inf")

        distance_parts = []
        best_index = -1
        best_distance = float("inf")
        current_distance = float("inf")

        for i, candidate_path in enumerate(self.candidate_paths_world):
            d, _ = self.distance_to_world_path(robot_pose_map, candidate_path)
            distance_parts.append(f"P{i + 1}={d:.2f}m")

            if i == self.selected_path_index:
                current_distance = d

            if d < best_distance:
                best_distance = d
                best_index = i

        return "near={" + ", ".join(distance_parts) + "}", best_index, best_distance, current_distance

    def log_drive_diagnostics(
        self,
        robot_pose_map: Tuple[float, float, float],
        v_cmd: float,
        w_cmd: float,
        front_min: float,
        left_min: float,
        right_min: float,
        valid_count: int,
        goal_distance: float,
        path_world_len: int,
        fit: float,
    ) -> None:
        left_linear, right_linear, left_rpm, right_rpm = self.estimate_wheel_speed(v_cmd, w_cmd)
        wheel_decision = self.wheel_motion_decision(v_cmd, w_cmd)
        obstacle_text = self.obstacle_decision_text(front_min, left_min, right_min, valid_count)
        near_text, closest_idx, closest_dist, current_dist = self.path_distance_report(robot_pose_map)

        if closest_idx >= 0:
            approach_text = (
                f"robot_mendekati_jalur_ke={closest_idx + 1}/{len(self.candidate_paths_world)} | "
                f"closest_dist={closest_dist:.2f}m | selected_dist={current_dist:.2f}m"
            )
        else:
            approach_text = "robot_mendekati_jalur_ke=NONE"

        angular_deg = math.degrees(w_cmd)

        self.log_status(
            (
                f"DRIVE_DIAGNOSTIC | selected_path={self.current_path_number_text()} | "
                f"idx={self.path_progress_index}/{path_world_len} | goal_dist={goal_distance:.2f}m | "
                f"{approach_text} | {near_text} | "
                f"{obstacle_text} | "
                f"cmd_v={v_cmd:.2f}m/s | cmd_w={w_cmd:.2f}rad/s({angular_deg:.1f}deg/s) | "
                f"wheel_decision={wheel_decision} | "
                f"left_wheel={left_linear:.2f}m/s,{left_rpm:.1f}rpm | "
                f"right_wheel={right_linear:.2f}m/s,{right_rpm:.1f}rpm | "
                f"block_confirm={self.block_confirm_ticks}/{self.block_confirm_limit} | fit={fit:.3f}"
            ),
            level="warn" if front_min < self.soft_obstacle_distance else "info",
        )

    def maybe_auto_switch_path(self, robot_pose_map: Tuple[float, float, float]) -> bool:
        if not self.auto_path_switch_enabled:
            return False

        if self.selected_path_index is None:
            return False

        if not self.candidate_paths_world:
            return False

        if self.auto_switch_cooldown_ticks > 0:
            self.auto_switch_cooldown_ticks -= 1
            return False

        if self.selected_path_index >= len(self.candidate_paths_world):
            return False

        near_text, closest_index, closest_dist, current_dist = self.path_distance_report(robot_pose_map)

        if closest_index < 0:
            return False

        self.log_status(
            (
                f"APPROACHING_PATH | selected_path={self.current_path_number_text()} | "
                f"robot_mendekati_jalur_ke={closest_index + 1}/{len(self.candidate_paths_world)} | "
                f"closest_dist={closest_dist:.2f}m | selected_dist={current_dist:.2f}m | {near_text}"
            ),
            level="info",
        )

        if closest_index == self.selected_path_index:
            return False

        # Tidak balik ke jalur nomor lebih kecil.
        if closest_index < self.selected_path_index:
            self.log_status(
                (
                    f"AUTO_SWITCH_REJECTED | selected_path={self.current_path_number_text()} | "
                    f"candidate_path={closest_index + 1}/{len(self.candidate_paths_world)} | "
                    f"reason=lower_number_path_not_allowed"
                ),
                level="warn",
            )
            return False

        if closest_index < len(self.path_blocked) and self.path_blocked[closest_index]:
            self.log_status(
                (
                    f"AUTO_SWITCH_REJECTED | selected_path={self.current_path_number_text()} | "
                    f"candidate_path={closest_index + 1}/{len(self.candidate_paths_world)} | "
                    f"reason=candidate_path_already_blocked"
                ),
                level="warn",
            )
            return False

        should_switch = (
            current_dist > self.auto_switch_current_far_m
            and closest_dist < self.auto_switch_candidate_near_m
            and (current_dist - closest_dist) > self.auto_switch_margin_m
        )

        if not should_switch:
            self.log_status(
                (
                    f"AUTO_SWITCH_HOLD | selected_path={self.current_path_number_text()} | "
                    f"candidate_path={closest_index + 1}/{len(self.candidate_paths_world)} | "
                    f"reason=threshold_not_met | selected_dist={current_dist:.2f}m | "
                    f"candidate_dist={closest_dist:.2f}m"
                ),
                level="info",
            )
            return False

        old_index = self.selected_path_index
        _, best_idx_on_path = self.distance_to_world_path(
            robot_pose_map,
            self.candidate_paths_world[closest_index],
        )

        self.selected_path_index = closest_index
        self.path_progress_index = best_idx_on_path

        self.reverse_count_current_path = 0
        self.reverse_ticks = 0
        self.uturn_ticks = 0

        self.block_confirm_ticks = 0
        self.block_confirm_reason = "RESET_AFTER_AUTO_PATH_SWITCH"

        self.last_goal_distance = None
        self.stall_start_time = None
        self.no_valid_start_time = None

        self.prev_v_cmd = 0.0
        self.prev_w_cmd = 0.0

        self.auto_switch_cooldown_ticks = self.auto_switch_cooldown_limit

        self.publish_selected_path()

        self.log_status(
            (
                f"AUTO_SWITCH_PATH | old_path={old_index + 1}/{len(self.candidate_paths_world)} "
                f"-> selected_path={closest_index + 1}/{len(self.candidate_paths_world)} | "
                f"current_dist={current_dist:.2f}m | candidate_dist={closest_dist:.2f}m | "
                f"start_idx={self.path_progress_index} | reason=robot_closer_to_higher_candidate_path"
            ),
            level="warn",
            force=True,
        )

        return True

    def update_path_progress_index(self, robot_pose_map: Tuple[float, float, float]) -> None:
        path_world = self.active_path_world()

        if not path_world:
            self.path_progress_index = 0
            return

        nearest = self.find_nearest_index_on_path(
            robot_pose_map,
            path_world,
            global_search=False,
        )

        self.path_progress_index = max(self.path_progress_index, nearest)

    def active_path_base_points(self, start_index: int) -> Optional[List[Point2D]]:
        path_world = self.active_path_world()

        if not path_world or len(path_world) < 2:
            return None

        safe_start = int(clamp(start_index, 0, len(path_world) - 1))
        segment = path_world[safe_start:]

        if len(segment) < 2:
            segment = path_world[-2:]

        segment = segment[:100]

        try:
            transform = self.lookup_transform(self.active_base_frame, self.map_frame)
        except TransformException:
            return None

        base_points = []

        for wx, wy in segment:
            bx, by = self.transform_xy(transform, wx, wy)
            base_points.append((bx, by))

        return base_points

    @staticmethod
    def min_distance_to_path(x: float, y: float, path_points: List[Point2D]) -> float:
        if not path_points:
            return 999.0

        return min(math.hypot(px - x, py - y) for px, py in path_points)

    def select_lookahead(self, path_points_base: List[Point2D], goal_distance: float) -> Point2D:
        if not path_points_base:
            return self.lookahead_distance, 0.0

        closest_idx = 0
        closest_dist = float("inf")

        for i, (x, y) in enumerate(path_points_base):
            d = math.hypot(x, y)

            if d < closest_dist:
                closest_dist = d
                closest_idx = i

        lookahead_distance = self.lookahead_distance

        if goal_distance < self.near_goal_distance:
            lookahead_distance = self.near_goal_lookahead_distance

        cumulative = 0.0
        prev = path_points_base[closest_idx]

        for i in range(closest_idx + 1, len(path_points_base)):
            current = path_points_base[i]
            cumulative += math.hypot(current[0] - prev[0], current[1] - prev[1])
            prev = current

            if cumulative >= lookahead_distance and current[0] > self.forward_point_min_x:
                return current

        return path_points_base[-1]

    # =============================================================
    # SCAN / OBSTACLE
    # =============================================================
    def get_obstacle_points_base(self) -> List[Point2D]:
        if self.latest_scan is None:
            return []

        scan = self.latest_scan
        scan_frame = scan.header.frame_id if scan.header.frame_id else self.active_base_frame

        transform = None

        if scan_frame != self.active_base_frame:
            try:
                transform = self.lookup_transform(self.active_base_frame, scan_frame)
            except TransformException:
                transform = None

        points = []
        total = len(scan.ranges)

        if total == 0:
            return points

        stride = max(1, total // self.scan_stride_target_points)

        for i in range(0, total, stride):
            r = scan.ranges[i]

            if math.isnan(r) or math.isinf(r):
                continue

            if r < scan.range_min:
                continue

            if scan.range_max > 0.0 and r > scan.range_max:
                continue

            if r > self.obstacle_range:
                continue

            angle = scan.angle_min + i * scan.angle_increment
            sx = r * math.cos(angle)
            sy = r * math.sin(angle)

            if transform is not None:
                bx, by = self.transform_xy(transform, sx, sy)
            else:
                bx, by = sx, sy

            points.append((bx, by))

        return points

    def front_left_right_clearance(self, obstacle_points: List[Point2D]) -> Tuple[float, float, float]:
        front_min = self.obstacle_range
        left_min = self.obstacle_range
        right_min = self.obstacle_range

        for x, y in obstacle_points:
            if x < self.ignore_obstacle_behind_x:
                continue

            d = math.hypot(x, y)
            angle = math.atan2(y, x)

            if x > 0.0 and abs(angle) <= self.front_angle:
                front_min = min(front_min, d)

            if y >= 0.0:
                left_min = min(left_min, d)
            else:
                right_min = min(right_min, d)

        return front_min, left_min, right_min

    def dynamic_clearance(self, x: float, y: float, obstacle_points: List[Point2D]) -> float:
        if not obstacle_points:
            return self.obstacle_range

        min_d = self.obstacle_range

        for ox, oy in obstacle_points:
            d = math.hypot(ox - x, oy - y)
            min_d = min(min_d, d)

        return min_d - self.robot_radius

    def capture_blocking_obstacles(self, robot_pose_map: Tuple[float, float, float], obstacle_points: List[Point2D]) -> None:
        for bx, by in obstacle_points:
            if bx < self.ignore_obstacle_behind_x:
                continue

            d = math.hypot(bx, by)

            if d > self.capture_blocking_distance_m:
                continue

            angle = abs(math.atan2(by, bx))

            if angle > self.capture_blocking_angle:
                continue

            mx, my = self.base_to_map_point(robot_pose_map, bx, by)
            cell = self.world_to_grid(mx, my)

            if cell is not None:
                self.blocked_obstacle_cells.append(cell)

        if len(self.blocked_obstacle_cells) > self.max_blocked_obstacle_cells:
            self.blocked_obstacle_cells = self.blocked_obstacle_cells[-self.max_blocked_obstacle_cells:]

    # =============================================================
    # PURE PURSUIT + CSA
    # =============================================================
    def pure_pursuit_reference(
        self,
        lookahead: Point2D,
        front_min: float,
        left_min: float,
        right_min: float,
        goal_distance: float,
    ) -> Tuple[float, float]:
        lx, ly = lookahead

        if goal_distance <= self.final_stop_distance and lx < self.goal_forward_stop_x:
            return 0.0, 0.0

        dist = max(0.05, math.hypot(lx, ly))
        alpha = math.atan2(ly, lx)

        curvature_factor = clamp(1.0 - abs(alpha) / self.curvature_angle_scale, self.curvature_factor_min, 1.0)

        if front_min < self.hard_obstacle_distance:
            base_speed = self.v_hard_obstacle
        elif front_min < self.soft_obstacle_distance:
            base_speed = self.v_soft_obstacle
        else:
            base_speed = self.v_preferred

        # Obstacle yang muncul di tengah jalur harus memaksa robot mengurangi
        # kecepatan dan bias belok ke sisi yang lebih aman, bukan hanya fokus
        # pada path reference saja.
        if front_min < self.soft_obstacle_distance:
            obstacle_gain = clamp(
                (self.soft_obstacle_distance - front_min) / max(
                    self.soft_obstacle_distance - self.hard_obstacle_distance,
                    1e-6,
                ),
                0.0,
                1.0,
            )
            base_speed *= max(0.25, 1.0 - 0.75 * obstacle_gain)

            avoid_dir = 1.0 if left_min >= right_min else -1.0
            w_ref += avoid_dir * (0.55 + 1.10 * obstacle_gain)

        if goal_distance < self.near_goal_distance:
            base_speed = min(base_speed, self.v_near_goal_max)

        v_ref = base_speed * curvature_factor
        v_ref = clamp(v_ref, self.v_min, self.v_max)

        w_ref = 2.0 * v_ref * math.sin(alpha) / dist

        if front_min < self.hard_obstacle_distance:
            w_limit = self.w_max_obstacle
        else:
            w_limit = self.w_max_clear

        w_ref = clamp(w_ref, -w_limit, w_limit)

        return v_ref, w_ref

    def levy_sigma(self) -> float:
        num = math.gamma(1.0 + self.beta) * math.sin(math.pi * self.beta / 2.0)
        den = (
            math.gamma((1.0 + self.beta) / 2.0)
            * self.beta
            * (2.0 ** ((self.beta - 1.0) / 2.0))
        )
        return (num / den) ** (1.0 / self.beta)

    def obstacle_bias(self, front_min: float, left_min: float, right_min: float) -> float:
        if front_min >= self.soft_obstacle_distance:
            return 0.0

        denominator = self.soft_obstacle_distance - self.hard_obstacle_distance
        intensity = (self.soft_obstacle_distance - front_min) / max(denominator, 1e-6)
        intensity = clamp(intensity, 0.0, 1.0)

        direction = 1.0 if left_min >= right_min else -1.0

        return direction * (self.obstacle_bias_base + self.obstacle_bias_gain * intensity)

    def create_candidate_velocity(
        self,
        v_ref: float,
        w_ref: float,
        front_min: float,
    ) -> Tuple[float, float]:
        w_limit = self.w_max_obstacle if front_min < self.hard_obstacle_distance else self.w_max_clear

        if random.random() < self.candidate_gaussian_probability:
            v = random.gauss(v_ref, self.candidate_v_std)
            w = random.gauss(w_ref, self.candidate_w_std)
        else:
            v = random.uniform(self.candidate_random_v_min, self.v_max)
            w = random.uniform(-w_limit, w_limit)

        v = clamp(v, self.v_min, self.v_max)
        w = clamp(w, -w_limit, w_limit)

        return v, w

    def known_map_clearance_from_base_prediction(
        self,
        robot_pose_map: Tuple[float, float, float],
        bx: float,
        by: float,
    ) -> float:
        mx, my = self.base_to_map_point(robot_pose_map, bx, by)
        cell = self.world_to_grid(mx, my)

        if cell is None:
            return 0.0

        # STEP A: Gunakan local costmap untuk trajectory scoring jika tersedia.
        # Local costmap lebih real-time dan mencakup obstacle dinamis.
        if self.costmap_enabled and self.use_local_costmap_for_scoring and self.local_costmap_data is not None:
            if self.is_occupied_cell_costmap(cell, use_local=True):
                return 0.0

        return self.clearance_meter(cell)

    def trajectory_cost(
        self,
        v: float,
        w: float,
        v_ref: float,
        w_ref: float,
        path_points_base: List[Point2D],
        lookahead: Point2D,
        obstacle_points: List[Point2D],
        robot_pose_map: Tuple[float, float, float],
        front_min: float,
    ) -> Tuple[float, bool]:
        x = 0.0
        y = 0.0
        theta = 0.0

        min_dynamic_clearance = self.obstacle_range
        min_map_clearance = 999.0
        path_cost_sum = 0.0

        for _ in range(self.prediction_steps):
            x += v * math.cos(theta) * self.prediction_dt
            y += v * math.sin(theta) * self.prediction_dt
            theta = normalize_angle(theta + w * self.prediction_dt)

            dyn_clearance = self.dynamic_clearance(x, y, obstacle_points)
            min_dynamic_clearance = min(min_dynamic_clearance, dyn_clearance)

            if dyn_clearance < self.dynamic_safety_margin:
                return self.invalid_trajectory_cost, False

            map_clearance = self.known_map_clearance_from_base_prediction(
                robot_pose_map,
                x,
                y,
            )
            min_map_clearance = min(min_map_clearance, map_clearance)

            if map_clearance < self.trajectory_required_clearance:
                return self.invalid_trajectory_cost, False

            path_cost_sum += self.min_distance_to_path(x, y, path_points_base)

        lx, ly = lookahead
        start_to_target = math.hypot(lx, ly)
        end_to_target = math.hypot(lx - x, ly - y)
        progress = start_to_target - end_to_target

        desired_heading = math.atan2(ly - y, lx - x)
        heading_error = abs(normalize_angle(desired_heading - theta))

        endpoint_path_distance = self.min_distance_to_path(x, y, path_points_base)

        path_cost = endpoint_path_distance + self.traj_path_average_weight * (
            path_cost_sum / float(self.prediction_steps)
        )

        dynamic_obstacle_cost = 1.0 / ((min_dynamic_clearance + self.traj_obstacle_cost_floor) ** 2)
        map_obstacle_cost = 1.0 / ((min_map_clearance + self.traj_obstacle_cost_floor) ** 2)

        speed_cost = abs(v - v_ref) + self.traj_speed_w_weight * abs(w - w_ref)

        smooth_cost = (
            self.traj_smooth_v_weight * abs(v - self.prev_v_cmd)
            + self.traj_smooth_w_weight * abs(w - self.prev_w_cmd)
        )

        if progress < 0.0:
            progress_cost = self.traj_negative_progress_weight * abs(progress)
        else:
            progress_cost = self.traj_positive_progress_weight * progress

        if front_min < self.hard_obstacle_distance:
            path_weight = self.traj_hard_path_weight
            obstacle_weight = self.traj_hard_obstacle_weight
            heading_weight = self.traj_hard_heading_weight
        elif front_min < self.soft_obstacle_distance:
            path_weight = self.traj_soft_path_weight
            obstacle_weight = self.traj_soft_obstacle_weight
            heading_weight = self.traj_soft_heading_weight
        else:
            path_weight = self.traj_clear_path_weight
            obstacle_weight = self.traj_clear_obstacle_weight
            heading_weight = self.traj_clear_heading_weight

        total_cost = (
            path_weight * path_cost
            + heading_weight * heading_error
            + obstacle_weight * dynamic_obstacle_cost
            + self.traj_map_obstacle_weight * map_obstacle_cost
            + self.traj_speed_cost_weight * speed_cost
            + self.traj_smooth_cost_weight * smooth_cost
            + progress_cost
            + self.traj_angular_cost_weight * abs(w)
        )

        return total_cost, True

    def seed_candidates(
        self,
        v_ref: float,
        w_ref: float,
        front_min: float,
        left_min: float,
        right_min: float,
    ) -> List[Tuple[float, float]]:
        w_limit = self.w_max_obstacle if front_min < self.hard_obstacle_distance else self.w_max_clear
        bias = self.obstacle_bias(front_min, left_min, right_min)

        seeds = [
            (v_ref, w_ref),
            (v_ref * self.seed_v_ref_scale_1, w_ref * self.seed_w_ref_scale_1),
            (v_ref * self.seed_v_ref_scale_2, w_ref + self.seed_w_offset),
            (v_ref * self.seed_v_ref_scale_2, w_ref - self.seed_w_offset),
            (max(v_ref, self.seed_v_bias_min), w_ref + bias),
            (self.seed_v_mid, bias),
            (self.seed_v_low, bias + self.seed_w_bias_offset),
            (self.seed_v_low, bias - self.seed_w_bias_offset),
            (self.seed_v_turn, self.seed_w_turn),
            (self.seed_v_turn, -self.seed_w_turn),
        ]

        clean = []

        for v, w in seeds:
            clean.append(
                (
                    clamp(v, self.v_min, self.v_max),
                    clamp(w, -w_limit, w_limit),
                )
            )

        return clean

    def optimize_csa(
        self,
        v_ref: float,
        w_ref: float,
        path_points_base: List[Point2D],
        lookahead: Point2D,
        obstacle_points: List[Point2D],
        robot_pose_map: Tuple[float, float, float],
        front_min: float,
        left_min: float,
        right_min: float,
    ) -> Tuple[float, float, float, int]:
        population = []

        for v, w in self.seed_candidates(v_ref, w_ref, front_min, left_min, right_min):
            fit, valid = self.trajectory_cost(
                v,
                w,
                v_ref,
                w_ref,
                path_points_base,
                lookahead,
                obstacle_points,
                robot_pose_map,
                front_min,
            )
            population.append({"v": v, "w": w, "fit": fit, "valid": valid})

        while len(population) < self.pop_size:
            v, w = self.create_candidate_velocity(v_ref, w_ref, front_min)
            fit, valid = self.trajectory_cost(
                v,
                w,
                v_ref,
                w_ref,
                path_points_base,
                lookahead,
                obstacle_points,
                robot_pose_map,
                front_min,
            )
            population.append({"v": v, "w": w, "fit": fit, "valid": valid})

        best = min(population, key=lambda item: item["fit"])
        sigma = self.levy_sigma()

        w_limit = self.w_max_obstacle if front_min < self.hard_obstacle_distance else self.w_max_clear

        for _ in range(self.max_iter):
            for i in range(self.pop_size):
                current = population[i]

                u_v = random.gauss(0.0, sigma)
                u_w = random.gauss(0.0, sigma)

                g_v = random.gauss(0.0, 1.0)
                g_w = random.gauss(0.0, 1.0)

                if abs(g_v) < 1e-9:
                    g_v = 1e-9

                if abs(g_w) < 1e-9:
                    g_w = 1e-9

                step_v = u_v / (abs(g_v) ** (1.0 / self.beta))
                step_w = u_w / (abs(g_w) ** (1.0 / self.beta))

                new_v = current["v"] + self.levy_alpha * step_v * (
                    current["v"] - best["v"]
                )
                new_w = current["w"] + self.levy_alpha * step_w * (
                    current["w"] - best["w"]
                )

                new_v = clamp(new_v, self.v_min, self.v_max)
                new_w = clamp(new_w, -w_limit, w_limit)

                new_fit, new_valid = self.trajectory_cost(
                    new_v,
                    new_w,
                    v_ref,
                    w_ref,
                    path_points_base,
                    lookahead,
                    obstacle_points,
                    robot_pose_map,
                    front_min,
                )

                j = random.randint(0, self.pop_size - 1)

                if new_fit < population[j]["fit"]:
                    population[j] = {
                        "v": new_v,
                        "w": new_w,
                        "fit": new_fit,
                        "valid": new_valid,
                    }

            population.sort(key=lambda item: item["fit"])

            abandon_count = int(self.pa * self.pop_size)

            for i in range(self.pop_size - abandon_count, self.pop_size):
                v, w = self.create_candidate_velocity(v_ref, w_ref, front_min)
                fit, valid = self.trajectory_cost(
                    v,
                    w,
                    v_ref,
                    w_ref,
                    path_points_base,
                    lookahead,
                    obstacle_points,
                    robot_pose_map,
                    front_min,
                )
                population[i] = {"v": v, "w": w, "fit": fit, "valid": valid}

            best = min(population, key=lambda item: item["fit"])

        valid_count = sum(1 for item in population if item["valid"])

        return float(best["v"]), float(best["w"]), float(best["fit"]), valid_count

    # =============================================================
    # BLOCK / RECOVERY / SWITCH
    # =============================================================
    def goal_is_behind_robot(self, goal_msg: PoseStamped) -> bool:
        goal_base = self.goal_base_coordinates(goal_msg)

        if goal_base is None:
            return False

        gx, gy = goal_base
        angle = math.atan2(gy, gx)

        return gx < self.goal_behind_x or abs(angle) > math.radians(self.goal_behind_angle_deg)

    def path_entry_target_base(self) -> Optional[Point2D]:
        """
        Titik di jalur terpilih yang harus didekati robot lebih dulu, dinyatakan
        di base frame. Diambil dari titik terdekat + sedikit lookahead ke depan
        jalur, supaya robot menghadap ke arah masuk jalur, bukan ke titik di
        belakangnya.
        """
        path_world = self.active_path_world()
        if not path_world:
            return None

        robot_pose = self.get_robot_pose_map()
        if robot_pose is None:
            return None

        nearest_idx = self.find_nearest_index_on_path(robot_pose, path_world, global_search=True)
        target_idx = min(nearest_idx + 3, len(path_world) - 1)
        tx_world, ty_world = path_world[target_idx]

        try:
            transform = self.lookup_transform(self.active_base_frame, self.map_frame)
        except TransformException:
            return None

        return self.transform_xy(transform, tx_world, ty_world)

    def path_entry_is_behind_robot(self) -> bool:
        """
        True jika titik masuk jalur terpilih berada di belakang robot, sehingga
        robot perlu putar balik dulu untuk mendekati jalur sebelum mengikuti goal.
        """
        entry_base = self.path_entry_target_base()
        if entry_base is None:
            return False

        ex, ey = entry_base
        angle = math.atan2(ey, ex)
        return ex < self.goal_behind_x or abs(angle) > math.radians(self.goal_behind_angle_deg)

    def initial_turn_target_base(self) -> Optional[Point2D]:
        """
        Target arah untuk INITIAL_GOAL_TURN: utamakan titik masuk jalur (supaya
        robot mendekati jalur dulu), fallback ke goal jika jalur tidak tersedia.
        """
        entry = self.path_entry_target_base()
        if entry is not None:
            return entry
        if self.goal_pose is not None:
            return self.goal_base_coordinates(self.goal_pose)
        return None

    def is_full_blocked_geometry(self, front_min: float, left_min: float, right_min: float) -> bool:
        return (
            front_min < self.full_block_front_distance
            and left_min < self.full_block_side_clearance
            and right_min < self.full_block_side_clearance
        )

    def is_trap_escape_geometry(self, front_min: float, left_min: float, right_min: float) -> bool:
        return (
            self.trap_escape_enabled
            and front_min < self.trap_escape_front_distance
            and left_min < self.trap_escape_side_distance
            and right_min < self.trap_escape_side_distance
        )

    def update_trap_escape_confirmation(
        self,
        front_min: float,
        left_min: float,
        right_min: float,
    ) -> Tuple[bool, str]:
        if not self.trap_escape_enabled:
            self.trap_escape_ticks = 0
            self.trap_escape_reason = "TRAP_ESCAPE_DISABLED"
            return False, self.trap_escape_reason

        trapped = self.is_trap_escape_geometry(front_min, left_min, right_min)

        if trapped:
            self.trap_escape_ticks += 1
            self.trap_escape_reason = (
                f"TRAP_FRONT_LEFT_RIGHT_CLOSE | "
                f"front={front_min:.2f}<{self.trap_escape_front_distance:.2f} | "
                f"left={left_min:.2f}<{self.trap_escape_side_distance:.2f} | "
                f"right={right_min:.2f}<{self.trap_escape_side_distance:.2f}"
            )
        else:
            self.trap_escape_ticks = max(0, self.trap_escape_ticks - 2)
            self.trap_escape_reason = "TRAP_NOT_CONFIRMED"

        confirmed = self.trap_escape_ticks >= self.trap_escape_confirm_limit
        return confirmed, self.trap_escape_reason

    def start_trap_escape_reverse(
        self,
        reason: str,
        robot_pose_map: Tuple[float, float, float],
        obstacle_points: List[Point2D],
    ) -> None:
        self.stop_robot()

        self.capture_blocking_obstacles(robot_pose_map, obstacle_points)
        self.mark_current_path_blocked(reason)

        self.trap_escape_reverse_ticks = self.trap_escape_reverse_tick_limit
        self.state = "TRAP_ESCAPE_REVERSE"
        self.prev_v_cmd = 0.0
        self.prev_w_cmd = 0.0
        self.block_confirm_ticks = 0
        self.block_confirm_reason = "RESET_AFTER_TRAP_ESCAPE_START"

        self.log_status(
            (
                f"TRAP_ESCAPE_START | path={self.current_path_number_text()} | "
                f"reverse_ticks={self.trap_escape_reverse_ticks}/{self.trap_escape_reverse_tick_limit} | "
                f"after_reverse=switch_path | reason={reason}"
            ),
            level="warn",
            force=True,
        )

    def finish_trap_escape_reverse(self) -> None:
        self.stop_robot()
        self.trap_escape_reverse_ticks = 0
        self.trap_escape_ticks = 0

        switched = False
        if self.trap_escape_switch_after_reverse:
            switched = self.switch_to_next_available_path()

        if switched:
            self.log_status(
                f"TRAP_ESCAPE_DONE | selected_path={self.current_path_number_text()} | action=switched_to_existing_candidate",
                level="warn",
                force=True,
            )
            return

        if self.trap_escape_regen_if_no_path:
            self.log_status(
                "TRAP_ESCAPE_REGEN | no unblocked candidate path left | action=generate_new_candidate_paths",
                level="warn",
                force=True,
            )
            self.regenerate_paths_after_all_blocked("trap_escape_no_existing_path_left")
            return

        self.state = "FOLLOW_PATH"
        self.recovery_cooldown_ticks = self.recovery_cooldown_limit
        self.log_status(
            "TRAP_ESCAPE_DONE | no switch/regeneration enabled | back_to_follow_path",
            level="warn",
            force=True,
        )

    def update_blocked_confirmation(
        self,
        front_min: float,
        left_min: float,
        right_min: float,
        valid_count: int,
        no_valid: bool,
        stalled: bool,
    ) -> Tuple[bool, str]:
        if self.recovery_cooldown_ticks > 0:
            self.recovery_cooldown_ticks -= 1
            self.block_confirm_ticks = 0
            self.block_confirm_reason = "COOLDOWN_AFTER_RECOVERY"
            return False, self.block_confirm_reason

        full_blocked = self.is_full_blocked_geometry(front_min, left_min, right_min)

        hard_geometry = (
            front_min < self.hard_geometry_front_distance
            and (left_min < self.hard_geometry_side_distance or right_min < self.hard_geometry_side_distance)
        )

        very_low_valid = valid_count <= max(2, int(self.very_low_valid_ratio * self.pop_size))

        reason = "CLEAR"

        if front_min <= self.emergency_distance:
            reason = "EMERGENCY_FRONT_TOO_CLOSE"
            self.block_confirm_ticks += 3

        elif full_blocked:
            reason = "FULL_GEOMETRY_BLOCKED"
            self.block_confirm_ticks += 2

        elif no_valid and hard_geometry:
            reason = "NO_VALID_WITH_HARD_GEOMETRY"
            self.block_confirm_ticks += 1

        elif stalled and hard_geometry:
            reason = "STALLED_WITH_HARD_GEOMETRY"
            self.block_confirm_ticks += 1

        elif very_low_valid and hard_geometry:
            reason = "VERY_LOW_VALID_WITH_HARD_GEOMETRY"
            self.block_confirm_ticks += 1

        else:
            self.block_confirm_ticks = max(0, self.block_confirm_ticks - 2)
            self.block_confirm_reason = "NOT_BLOCKED_KEEP_CURRENT_PATH"
            return False, self.block_confirm_reason

        self.block_confirm_reason = reason
        confirmed = self.block_confirm_ticks >= self.block_confirm_limit

        return confirmed, reason

    def start_reverse_recovery(
        self,
        reason: str,
        robot_pose_map: Tuple[float, float, float],
        obstacle_points: List[Point2D],
        left_min: float,
        right_min: float,
    ) -> None:
        self.reverse_count_current_path += 1
        self.reverse_ticks = self.reverse_tick_limit
        self.state = "REVERSE_RECOVERY"

        self.capture_blocking_obstacles(robot_pose_map, obstacle_points)

        self.uturn_direction = 1.0 if left_min >= right_min else -1.0

        self.block_confirm_ticks = 0
        self.block_confirm_reason = "RESET_AFTER_REVERSE_START"

        self.log_status(
            (
                f"REVERSE_START | path={self.current_path_number_text()} | "
                f"reverse={self.reverse_count_current_path}/{self.reverse_required_before_switch} | "
                f"reason={reason}"
            ),
            level="warn",
            force=True,
        )

    def finish_reverse_recovery(self) -> None:
        self.recovery_cooldown_ticks = self.recovery_cooldown_limit

        if self.reverse_count_current_path >= self.reverse_required_before_switch:
            self.start_uturn()
        else:
            self.state = "FOLLOW_PATH"
            self.prev_v_cmd = 0.0
            self.prev_w_cmd = 0.0
            self.log_status(
                f"REVERSE_DONE | tetap coba path={self.current_path_number_text()}",
                level="warn",
                force=True,
            )

    def start_uturn(self) -> None:
        self.uturn_ticks = self.uturn_tick_limit
        self.state = "UTURN_SWITCH_PATH"

        self.log_status(
            (
                f"UTURN_START | current_path={self.current_path_number_text()} | "
                f"after_uturn=switch_to_next_existing_path"
            ),
            level="warn",
            force=True,
        )

    def finish_uturn_and_switch(self) -> None:
        self.uturn_ticks = 0

        self.mark_current_path_blocked("reverse 2x + uturn")

        switched = self.switch_to_next_available_path()

        if not switched:
            self.regenerate_paths_after_all_blocked("all_existing_candidate_paths_blocked")

    def mark_current_path_blocked(self, reason: str) -> None:
        if self.selected_path_index is None:
            return

        if self.selected_path_index >= len(self.path_blocked):
            return

        self.path_blocked[self.selected_path_index] = True

        cells = self.active_path_cells()

        if cells:
            self.blocked_path_history.append(cells)

        if len(self.blocked_path_history) > 20:
            self.blocked_path_history = self.blocked_path_history[-20:]

        self.log_status(
            f"PATH_BLOCKED | path={self.selected_path_index + 1}/{len(self.candidate_paths_world)} | reason={reason}",
            level="warn",
            force=True,
        )

    def switch_to_next_available_path(self) -> bool:
        if not self.candidate_paths_cells:
            return False

        old_index = self.selected_path_index

        if old_index is None:
            return False

        new_index = old_index + 1

        if new_index >= len(self.candidate_paths_world):
            self.log_status(
                (
                    f"NO_NEXT_PATH_LEFT | current_path={old_index + 1}/{len(self.candidate_paths_world)} | "
                    f"path_terakhir_gagal_generate_baru"
                ),
                level="warn",
                force=True,
            )
            return False

        if new_index < len(self.path_blocked) and self.path_blocked[new_index]:
            self.log_status(
                (
                    f"NEXT_PATH_ALREADY_BLOCKED | current_path={old_index + 1}/{len(self.candidate_paths_world)} | "
                    f"next_path={new_index + 1}/{len(self.candidate_paths_world)}"
                ),
                level="warn",
                force=True,
            )
            return False

        robot_pose = self.get_robot_pose_map()

        self.selected_path_index = new_index

        if robot_pose is not None:
            self.path_progress_index = self.find_nearest_index_on_path(
                robot_pose,
                self.candidate_paths_world[new_index],
                global_search=True,
            )
        else:
            self.path_progress_index = 0

        self.reverse_count_current_path = 0
        self.reverse_ticks = 0
        self.uturn_ticks = 0
        self.block_confirm_ticks = 0
        self.block_confirm_reason = "RESET_AFTER_SWITCH_TO_NEXT_PATH"
        self.trap_escape_ticks = 0
        self.trap_escape_reverse_ticks = 0
        self.trap_escape_reason = "RESET_AFTER_SWITCH_TO_NEXT_PATH"
        self.front_stuck_start_time = None
        self.front_stuck_uturn_ticks = 0
        self.front_stuck_reason = "RESET_AFTER_SWITCH_TO_NEXT_PATH"
        self.recovery_cooldown_ticks = self.recovery_cooldown_limit

        self.last_goal_distance = None
        self.stall_start_time = None
        self.no_valid_start_time = None
        self.prev_v_cmd = 0.0
        self.prev_w_cmd = 0.0
        self.auto_switch_cooldown_ticks = self.auto_switch_cooldown_limit
        self.state = "FOLLOW_PATH"

        self.publish_selected_path()

        self.log_status(
            (
                f"SWITCH_PATH | old_path={old_index + 1}/{len(self.candidate_paths_world)} "
                f"-> selected_path={new_index + 1}/{len(self.candidate_paths_world)} | "
                f"start_idx={self.path_progress_index} | source=next_path_plus_one"
            ),
            level="warn",
            force=True,
        )

        return True

    def current_path_number_text(self) -> str:
        if self.selected_path_index is None:
            return "NONE"

        return f"{self.selected_path_index + 1}/{len(self.candidate_paths_world)}"

    # =============================================================
    # SMOOTHING / MONITOR
    # =============================================================
    def smooth_cmd(self, v_cmd: float, w_cmd: float) -> Tuple[float, float]:
        v_smoothed = self.prev_v_cmd + clamp(
            v_cmd - self.prev_v_cmd,
            -self.max_delta_v,
            self.max_delta_v,
        )

        w_smoothed = self.prev_w_cmd + clamp(
            w_cmd - self.prev_w_cmd,
            -self.max_delta_w,
            self.max_delta_w,
        )

        if abs(w_smoothed) < self.angular_deadband:
            w_smoothed = 0.0

        self.prev_v_cmd = v_smoothed
        self.prev_w_cmd = w_smoothed

        return v_smoothed, w_smoothed

    def update_stall_monitor(self, goal_distance: float) -> bool:
        now = self.now_sec()

        if self.last_goal_distance is None:
            self.last_goal_distance = goal_distance
            self.stall_start_time = now
            return False

        progress = self.last_goal_distance - goal_distance

        if progress >= self.min_progress_meter:
            self.last_goal_distance = goal_distance
            self.stall_start_time = now
            return False

        if self.stall_start_time is None:
            self.stall_start_time = now
            return False

        return (now - self.stall_start_time) >= self.stall_timeout

    def update_no_valid_monitor(self, valid_count: int) -> bool:
        now = self.now_sec()

        if valid_count > 0:
            self.no_valid_start_time = None
            return False

        if self.no_valid_start_time is None:
            self.no_valid_start_time = now
            return False

        return (now - self.no_valid_start_time) >= self.no_valid_timeout

    # =============================================================
    # CONTROL LOOP
    # =============================================================
    def control_loop(self) -> None:
        if self.map_msg is None:
            self.stop_robot()
            self.log_status("WAIT_MAP", level="warn")
            return

        if self.goal_pose is None:
            self.stop_robot()
            self.log_status("WAIT_GOAL | kirim goal ke /goal_pose", level="warn")
            return

        robot_pose = self.get_robot_pose_map()

        if robot_pose is None:
            self.stop_robot()
            self.log_status("WAIT_TF | set initial pose dulu", level="warn")
            return

        # JOB 6 - rekam history jalur yang ditempuh robot.
        self.record_traveled_point(robot_pose)
        if self.state == "INITIAL_GOAL_TURN":
            # Arahkan ke titik masuk JALUR terpilih dulu (fallback ke goal).
            turn_target = self.initial_turn_target_base()

            if turn_target is None:
                self.publish_vel(0.0, self.initial_turn_direction * self.initial_turn_angular_speed_search)
                return

            gx_b, gy_b = turn_target
            bearing = math.atan2(gy_b, gx_b)

            self.initial_turn_ticks -= 1

            # Selesai kalau target jalur sudah ada di depan robot.
            if gx_b > self.initial_turn_done_x and abs(bearing) < math.radians(self.initial_turn_done_bearing_deg):
                self.initial_turn_done = True
                self.state = "FOLLOW_PATH"
                self.stop_robot()
                self.log_status(
                    f"INITIAL_TURN_DONE | menghadap_jalur | selected_path={self.current_path_number_text()}",
                    level="warn",
                    force=True,
                )
                return

            if self.initial_turn_ticks <= 0:
                self.initial_turn_done = True
                self.state = "FOLLOW_PATH"
                self.stop_robot()
                self.log_status(
                    f"INITIAL_TURN_TIMEOUT | continue selected_path={self.current_path_number_text()}",
                    level="warn",
                    force=True,
                )
                return

            turn_direction = 1.0 if bearing >= 0.0 else -1.0
            self.initial_turn_direction = turn_direction
            self.publish_vel(0.0, turn_direction * self.initial_turn_angular_speed_track)

            self.log_status(
                f"INITIAL_TURN_RUNNING | menghadap_jalur | bearing={math.degrees(bearing):.1f}deg | "
                f"selected_path={self.current_path_number_text()}",
                level="warn",
            )
            return

        # ---------------------------------------------------------
        # Reverse recovery
        # ---------------------------------------------------------
        if self.state == "REVERSE_RECOVERY":
            self.publish_vel(self.reverse_speed, 0.0)
            self.reverse_ticks -= 1

            self.log_status(
                f"REVERSING | path={self.current_path_number_text()} | reverse={self.reverse_count_current_path}/{self.reverse_required_before_switch}",
                level="warn",
            )

            if self.reverse_ticks <= 0:
                self.stop_robot()
                self.finish_reverse_recovery()

            return

        # ---------------------------------------------------------
        # U-turn then switch path
        # ---------------------------------------------------------
        if self.state == "UTURN_SWITCH_PATH":
            self.publish_vel(0.0, self.uturn_direction * self.uturn_angular_speed)
            self.uturn_ticks -= 1

            self.log_status(
                f"UTURN_RUNNING | current_path={self.current_path_number_text()}",
                level="warn",
            )

            if self.uturn_ticks <= 0:
                self.stop_robot()
                self.finish_uturn_and_switch()

            return

        # ---------------------------------------------------------
        # Trap escape: reverse first, then ignore current path and switch.
        # ---------------------------------------------------------
        if self.state == "TRAP_ESCAPE_REVERSE":
            self.publish_vel(self.reverse_speed, 0.0)
            self.trap_escape_reverse_ticks -= 1

            self.log_status(
                (
                    f"TRAP_ESCAPE_REVERSING | blocked_path={self.current_path_number_text()} | "
                    f"ticks_left={self.trap_escape_reverse_ticks}/{self.trap_escape_reverse_tick_limit}"
                ),
                level="warn",
            )

            if self.trap_escape_reverse_ticks <= 0:
                self.finish_trap_escape_reverse()

            return

        # ---------------------------------------------------------
        # JOB 6 - Front-stuck U-turn.
        # ---------------------------------------------------------
        if self.state == "FRONT_STUCK_UTURN":
            self.publish_vel(0.0, self.front_stuck_uturn_direction * self.front_stuck_uturn_speed)
            self.front_stuck_uturn_ticks -= 1
            self.log_status(
                f"FRONT_STUCK_UTURN_RUNNING | blocked_path={self.current_path_number_text()} | "
                f"ticks_left={self.front_stuck_uturn_ticks}/{self.front_stuck_uturn_tick_limit}",
                level="warn",
            )
            if self.front_stuck_uturn_ticks <= 0:
                self.stop_robot()
                self.finish_front_stuck_uturn()
            return

        if self.selected_path_index is None or not self.candidate_paths_world:
            self.stop_robot()
            self.log_status("WAIT_PATH | path belum tersedia", level="warn")
            return

        # Fitur baru minimal: jika posisi aktual robot lebih dekat ke
        # candidate path lain, selected_path dipindah otomatis.
        self.maybe_auto_switch_path(robot_pose)

        path_world = self.active_path_world()

        if path_world is None or len(path_world) < 2:
            self.stop_robot()
            self.log_status("WAIT_ACTIVE_PATH | path aktif tidak valid", level="warn")
            return

        goal_xy = self.goal_xy_in_map(self.goal_pose)

        if goal_xy is None:
            self.stop_robot()
            self.log_status("WAIT_GOAL_TF", level="warn")
            return

        gx, gy = goal_xy
        rx, ry, _ = robot_pose
        goal_distance = math.hypot(gx - rx, gy - ry)

        if goal_distance <= self.goal_tolerance:
            self.stop_robot()
            self.state = "GOAL_REACHED"
            self.log_status(
                f"GOAL_REACHED_STOP | selected_path={self.current_path_number_text()} | goal_dist={goal_distance:.2f}",
                level="info",
                force=True,
            )
            return

        self.update_path_progress_index(robot_pose)

        path_points_base = self.active_path_base_points(self.path_progress_index)

        if path_points_base is None or len(path_points_base) < 2:
            self.stop_robot()
            self.log_status("WAIT_PATH_TF", level="warn")
            return

        obstacle_points = self.get_obstacle_points_base()
        front_min, left_min, right_min = self.front_left_right_clearance(obstacle_points)

        # LaserScan is the last line of defense for obstacles that are absent
        # from the static map. Never allow a forward command when the measured
        # clearance is inside the robot's emergency envelope.
        if self.dynamic_replan_cooldown_ticks > 0:
            self.dynamic_replan_cooldown_ticks -= 1

        if front_min < self.dynamic_replan_distance and self.dynamic_replan_cooldown_ticks <= 0:
            self.stop_robot()
            self.capture_blocking_obstacles(robot_pose, obstacle_points)
            if front_min <= self.emergency_distance:
                self.mark_current_path_blocked("EMERGENCY_SCAN_OBSTACLE")
            self.dynamic_replan_cooldown_ticks = max(1, int(2.5 * self.control_frequency))
            self.regenerate_paths_after_all_blocked("dynamic_obstacle_detected_early_by_lidar")
            self.log_status(
                f"DYNAMIC_OBSTACLE_REPLAN | front={front_min:.2f}m | safe_trigger={self.dynamic_replan_distance:.2f}m | action=CSA_NEW_PATH",
                level="warn",
                force=True,
            )
            return

        trap_confirmed, trap_reason = self.update_trap_escape_confirmation(
            front_min=front_min,
            left_min=left_min,
            right_min=right_min,
        )

        if trap_confirmed:
            self.start_trap_escape_reverse(
                reason=trap_reason,
                robot_pose_map=robot_pose,
                obstacle_points=obstacle_points,
            )
            return

        # JOB 6 - Deteksi blokade depan + macet.
        front_stuck_confirmed, front_stuck_reason = self.update_front_stuck_confirmation(
            front_min=front_min,
            goal_distance=goal_distance,
        )
        if front_stuck_confirmed:
            self.start_front_stuck_uturn(
                reason=front_stuck_reason,
                robot_pose_map=robot_pose,
                obstacle_points=obstacle_points,
                left_min=left_min,
                right_min=right_min,
            )
            return

        lookahead = self.select_lookahead(path_points_base, goal_distance)

        if goal_distance <= self.final_stop_distance and lookahead[0] < self.goal_forward_stop_x:
            self.stop_robot()
            self.log_status(
                f"FINAL_APPROACH_STOP | selected_path={self.current_path_number_text()} | goal_dist={goal_distance:.2f}",
                level="info",
                force=True,
            )
            return

        v_ref, w_ref = self.pure_pursuit_reference(
            lookahead,
            front_min,
            left_min,
            right_min,
            goal_distance,
        )

        bias = self.obstacle_bias(front_min, left_min, right_min)
        w_ref = clamp(w_ref + bias, -self.w_max_obstacle, self.w_max_obstacle)

        v_raw, w_raw, fit, valid_count = self.optimize_csa(
            v_ref=v_ref,
            w_ref=w_ref,
            path_points_base=path_points_base,
            lookahead=lookahead,
            obstacle_points=obstacle_points,
            robot_pose_map=robot_pose,
            front_min=front_min,
            left_min=left_min,
            right_min=right_min,
        )

        no_valid = self.update_no_valid_monitor(valid_count)
        stalled = self.update_stall_monitor(goal_distance)

        confirmed_blocked, block_reason = self.update_blocked_confirmation(
            front_min=front_min,
            left_min=left_min,
            right_min=right_min,
            valid_count=valid_count,
            no_valid=no_valid,
            stalled=stalled,
        )

        if confirmed_blocked:
            # Jika candidate path terakhir sudah tidak bisa dilewati,
            # robot berhenti lalu generate path baru.
            if (
                self.selected_path_index is not None
                and self.selected_path_index >= len(self.candidate_paths_world) - 1
            ):
                self.stop_robot()
                self.capture_blocking_obstacles(robot_pose, obstacle_points)
                self.mark_current_path_blocked(f"path_terakhir_blocked | {block_reason}")
                self.regenerate_paths_after_all_blocked("last_candidate_path_blocked_cannot_pass")
                return

            self.start_reverse_recovery(
                reason=block_reason,
                robot_pose_map=robot_pose,
                obstacle_points=obstacle_points,
                left_min=left_min,
                right_min=right_min,
            )
            return

        if valid_count <= 0:
            self.stop_robot()
            self.log_status(
                f"HOLD_CURRENT_PATH | selected_path={self.current_path_number_text()} | reason=no_valid_but_not_confirmed_blocked | confirm={self.block_confirm_ticks}/{self.block_confirm_limit}",
                level="warn",
            )
            return

        v_cmd, w_cmd = self.smooth_cmd(v_raw, w_raw)
        self.publish_vel(v_cmd, w_cmd)

        self.log_drive_diagnostics(
            robot_pose_map=robot_pose,
            v_cmd=v_cmd,
            w_cmd=w_cmd,
            front_min=front_min,
            left_min=left_min,
            right_min=right_min,
            valid_count=valid_count,
            goal_distance=goal_distance,
            path_world_len=len(path_world),
            fit=fit,
        )

    # =============================================================
    # RESET / CMD
    # =============================================================
    def reset_runtime_for_new_goal(self, reset_history: bool) -> None:
        self.candidate_paths_cells = []
        self.candidate_paths_world = []
        self.candidate_costs = []
        self.path_blocked = []
        self.selected_path_index = None
        self.path_progress_index = 0

        if reset_history:
            self.blocked_path_history = []
            self.blocked_obstacle_cells = []
            self.regeneration_count = 0

        self.reverse_ticks = 0
        self.reverse_count_current_path = 0
        self.uturn_ticks = 0
        self.trap_escape_ticks = 0
        self.trap_escape_reverse_ticks = 0
        self.trap_escape_reason = "RESET_RUNTIME"

        # JOB 6 - reset front-stuck dan traveled path.
        self.front_stuck_start_time = None
        self.front_stuck_uturn_ticks = 0
        self.front_stuck_reason = "RESET_RUNTIME"
        self.reset_traveled_path()

        self.block_confirm_ticks = 0
        self.block_confirm_reason = "RESET_NEW_GOAL"
        self.recovery_cooldown_ticks = 0
        self.dynamic_replan_cooldown_ticks = 0

        self.initial_turn_done = True
        self.initial_turn_ticks = 0

        self.last_goal_distance = None
        self.stall_start_time = None
        self.no_valid_start_time = None

        self.prev_v_cmd = 0.0
        self.prev_w_cmd = 0.0
        self.auto_switch_cooldown_ticks = 0

        self.state = "WAIT_PATH"

    def publish_vel(self, v: float, w: float) -> None:
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_vel_pub.publish(msg)

        # Opsional untuk kompatibilitas sistem lama. Default mati agar tidak
        # mengirim dua sumber command secara tidak sengaja.
        if self.cmd_vel_mirror_pub is not None:
            self.cmd_vel_mirror_pub.publish(msg)

    def stop_robot(self) -> None:
        self.prev_v_cmd = 0.0
        self.prev_w_cmd = 0.0
        self.publish_vel(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = MultiPathCSALocalPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()