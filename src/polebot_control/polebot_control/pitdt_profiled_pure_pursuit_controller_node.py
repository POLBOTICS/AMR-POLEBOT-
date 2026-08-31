#!/usr/bin/env python3

import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray, Twist
from nav_msgs.msg import Path


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_pose(pose: Pose) -> float:
    q = pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PitdtProfiledPurePursuitController(Node):
    """
    PI(t)D(t)-Pure Pursuit controller with distance-based motion profiling.

    Input:
      - PoseArray from odom adapter, default: /model/ddr/pose
      - Reference path, default: /reference_path

    Output:
      - geometry_msgs/Twist, default: /cmd_vel

    Control structure:
      reference_path -> motion profile -> v_ff
                     -> pure pursuit   -> w_ff
                     -> PI(t)D(t)-like feedback correction
                     -> /cmd_vel

    Notes:
      - This node is designed for differential drive.
      - It only commands linear.x and angular.z.
      - It does not command lateral velocity.
    """

    def __init__(self):
        super().__init__("pitdt_profiled_pure_pursuit_controller_node")

        # Topics
        self.declare_parameter("pose_topic", "/model/ddr/pose")
        self.declare_parameter("reference_path_topic", "/reference_path")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        # Timing
        self.declare_parameter("control_rate", 40.0)

        # Pure pursuit geometry
        self.declare_parameter("lookahead_distance", 0.50)
        self.declare_parameter("stop_distance", 0.05)
        self.declare_parameter("finish_distance", 0.08)

        # Hard command limits
        self.declare_parameter("max_v", 0.30)
        self.declare_parameter("max_w", 0.60)

        # Motion profile parameters
        self.declare_parameter("profile_v_max", 0.25)
        self.declare_parameter("profile_a_max", 0.08)
        self.declare_parameter("profile_d_max", 0.10)
        self.declare_parameter("profile_min_v", 0.025)
        self.declare_parameter("profile_min_v_disable_distance", 0.20)
        self.declare_parameter("max_lateral_accel", 0.12)
        self.declare_parameter("profile_alpha_max", 0.70)

        # Linear correction parameters.
        # In this profiled controller, the linear part is used mostly as
        # speed reduction when cross-track error is large.
        self.declare_parameter("kp_lin", 0.35)
        self.declare_parameter("ki_lin", 0.00)
        self.declare_parameter("kd_lin", 0.02)
        self.declare_parameter("u0_lin", 0.00)
        self.declare_parameter("ramp_lin", 0.25)
        self.declare_parameter("v_fb_max", 0.08)
        self.declare_parameter("i_lin_limit", 1.0)

        # Angular correction parameters
        self.declare_parameter("kp_ang", 0.65)
        self.declare_parameter("ki_ang", 0.00)
        self.declare_parameter("kd_ang", 0.02)
        self.declare_parameter("u0_ang", 0.03)
        self.declare_parameter("ramp_ang", 0.30)
        self.declare_parameter("w_fb_max", 0.30)
        self.declare_parameter("i_ang_limit", 1.0)

        # Reacquire behavior
        self.declare_parameter("reacquire_distance", 0.25)
        self.declare_parameter("reacquire_heading_deg", 40.0)
        self.declare_parameter("reacquire_v_limit", 0.12)
        self.declare_parameter("reacquire_min_v", 0.04)
        self.declare_parameter("reacquire_w_limit", 0.60)

        # Debug
        self.declare_parameter("print_debug", False)

        self.pose_topic = self.get_parameter("pose_topic").value
        self.reference_path_topic = self.get_parameter("reference_path_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self.control_rate = float(self.get_parameter("control_rate").value)
        self.dt_nominal = 1.0 / max(self.control_rate, 1.0)

        self.pose_sub = self.create_subscription(
            PoseArray,
            self.pose_topic,
            self.pose_callback,
            10,
        )

        self.path_sub = self.create_subscription(
            Path,
            self.reference_path_topic,
            self.path_callback,
            10,
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.current_pose: Optional[Pose] = None
        self.path_xy: List[Tuple[float, float]] = []
        self.path_s: List[float] = []
        self.path_length = 0.0

        self.last_time = self.get_clock().now()
        self.start_time: Optional[float] = None

        self.last_v_cmd = 0.0
        self.last_w_cmd = 0.0

        self.i_lin = 0.0
        self.i_ang = 0.0
        self.prev_cte = 0.0
        self.prev_ang_error = 0.0

        self.finished = False
        self.last_nearest_index = 0

        self.timer = self.create_timer(self.dt_nominal, self.control_loop)

        self.get_logger().info("PI(t)D(t)-Pure Pursuit + motion profile controller started.")
        self.get_logger().info(f"pose_topic={self.pose_topic}")
        self.get_logger().info(f"reference_path_topic={self.reference_path_topic}")
        self.get_logger().info(f"cmd_vel_topic={self.cmd_vel_topic}")

    def pose_callback(self, msg: PoseArray) -> None:
        if not msg.poses:
            return
        self.current_pose = msg.poses[-1]

    def path_callback(self, msg: Path) -> None:
        points: List[Tuple[float, float]] = []
        for ps in msg.poses:
            points.append((ps.pose.position.x, ps.pose.position.y))

        if len(points) < 2:
            return

        s_values = [0.0]
        total = 0.0
        for i in range(1, len(points)):
            dx = points[i][0] - points[i - 1][0]
            dy = points[i][1] - points[i - 1][1]
            total += math.hypot(dx, dy)
            s_values.append(total)

        self.path_xy = points
        self.path_s = s_values
        self.path_length = total
        self.finished = False
        self.last_nearest_index = 0

        self.get_logger().info(
            f"Loaded reference path: {len(points)} points, length={total:.3f} m"
        )

    def get_param_float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def publish_zero(self) -> None:
        msg = Twist()
        self.cmd_pub.publish(msg)
        self.last_v_cmd = 0.0
        self.last_w_cmd = 0.0

    def nearest_path_index(self, x: float, y: float) -> int:
        if not self.path_xy:
            return 0

        # Search full path. Path is small enough, around hundreds of points.
        best_i = 0
        best_d2 = float("inf")

        for i, (px, py) in enumerate(self.path_xy):
            dx = px - x
            dy = py - y
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_i = i

        self.last_nearest_index = best_i
        return best_i

    def lookahead_index(self, nearest_i: int, lookahead_distance: float) -> int:
        if not self.path_s:
            return nearest_i

        target_s = self.path_s[nearest_i] + lookahead_distance
        for i in range(nearest_i, len(self.path_s)):
            if self.path_s[i] >= target_s:
                return i

        return len(self.path_s) - 1

    def compute_profile_velocity(
        self,
        progress_s: float,
        remaining_s: float,
        goal_distance: float,
        curvature: float,
    ) -> float:
        max_v = self.get_param_float("max_v")
        profile_v_max = self.get_param_float("profile_v_max")
        profile_a_max = max(self.get_param_float("profile_a_max"), 1.0e-6)
        profile_d_max = max(self.get_param_float("profile_d_max"), 1.0e-6)
        profile_min_v = self.get_param_float("profile_min_v")
        profile_min_v_disable_distance = self.get_param_float("profile_min_v_disable_distance")
        max_lateral_accel = max(self.get_param_float("max_lateral_accel"), 1.0e-6)

        v_accel = math.sqrt(max(0.0, 2.0 * profile_a_max * progress_s))
        v_decel = math.sqrt(max(0.0, 2.0 * profile_d_max * max(remaining_s, goal_distance)))

        v_cmd = min(max_v, profile_v_max, v_accel, v_decel)

        # Reduce velocity in sharp turns using lateral acceleration limit:
        # a_lat = v^2 * curvature
        if abs(curvature) > 1.0e-6:
            v_curve = math.sqrt(max_lateral_accel / abs(curvature))
            v_cmd = min(v_cmd, v_curve)

        # Avoid stalling far from the goal.
        if goal_distance > profile_min_v_disable_distance:
            v_cmd = max(v_cmd, profile_min_v)

        return clamp(v_cmd, 0.0, max_v)

    def rate_limit(self, target: float, previous: float, max_rate: float, dt: float) -> float:
        delta = target - previous
        max_delta = abs(max_rate) * dt
        delta = clamp(delta, -max_delta, max_delta)
        return previous + delta

    def control_loop(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1.0e-9
        self.last_time = now

        if dt <= 0.0 or dt > 1.0:
            dt = self.dt_nominal

        if self.current_pose is None or len(self.path_xy) < 2:
            self.publish_zero()
            return

        if self.start_time is None:
            self.start_time = now.nanoseconds * 1.0e-9

        x = self.current_pose.position.x
        y = self.current_pose.position.y
        yaw = yaw_from_pose(self.current_pose)

        goal_x, goal_y = self.path_xy[-1]
        goal_distance = math.hypot(goal_x - x, goal_y - y)

        stop_distance = self.get_param_float("stop_distance")
        finish_distance = self.get_param_float("finish_distance")

        if goal_distance <= stop_distance:
            if not self.finished:
                self.get_logger().info(f"Goal reached. goal_distance={goal_distance:.4f} m")
            self.finished = True
            self.publish_zero()
            return

        nearest_i = self.nearest_path_index(x, y)
        progress_s = self.path_s[nearest_i]
        remaining_s = max(0.0, self.path_length - progress_s)

        if nearest_i >= len(self.path_xy) - 2 and goal_distance <= finish_distance:
            if not self.finished:
                self.get_logger().info(
                    f"Goal region reached. nearest_i={nearest_i}, goal_distance={goal_distance:.4f} m"
                )
            self.finished = True
            self.publish_zero()
            return

        lookahead_distance = self.get_param_float("lookahead_distance")
        target_i = self.lookahead_index(nearest_i, lookahead_distance)
        target_x, target_y = self.path_xy[target_i]

        dx = target_x - x
        dy = target_y - y

        # Transform target point to robot frame.
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        x_r = cos_yaw * dx + sin_yaw * dy
        y_r = -sin_yaw * dx + cos_yaw * dy

        lookahead_actual = max(math.hypot(x_r, y_r), 1.0e-6)

        # Differential-drive pure pursuit curvature.
        curvature = 2.0 * y_r / (lookahead_actual * lookahead_actual)

        # Heading error to lookahead point.
        heading_error = normalize_angle(math.atan2(y_r, x_r))

        # Cross-track error approximation.
        cte = y_r

        # Feed-forward motion profile.
        v_ff = self.compute_profile_velocity(
            progress_s=progress_s,
            remaining_s=remaining_s,
            goal_distance=goal_distance,
            curvature=curvature,
        )
        w_ff = v_ff * curvature

        # Angular PI(t)D(t)-like feedback.
        kp_ang = self.get_param_float("kp_ang")
        ki_ang = self.get_param_float("ki_ang")
        kd_ang = self.get_param_float("kd_ang")
        u0_ang = self.get_param_float("u0_ang")
        w_fb_max = self.get_param_float("w_fb_max")
        i_ang_limit = self.get_param_float("i_ang_limit")

        # Combine target heading and crosstrack geometry into one angular error.
        cte_angle = math.atan2(2.0 * cte, max(lookahead_distance, 1.0e-3))
        ang_error = normalize_angle(heading_error + cte_angle)

        self.i_ang += ang_error * dt
        self.i_ang = clamp(self.i_ang, -i_ang_limit, i_ang_limit)

        d_ang = (ang_error - self.prev_ang_error) / dt
        self.prev_ang_error = ang_error

        w_fb = kp_ang * ang_error + ki_ang * self.i_ang + kd_ang * d_ang

        if abs(ang_error) > 0.01 and abs(u0_ang) > 0.0:
            w_fb += math.copysign(u0_ang, ang_error)

        w_fb = clamp(w_fb, -w_fb_max, w_fb_max)

        # Linear correction: reduce speed when cross-track error is large.
        kp_lin = self.get_param_float("kp_lin")
        ki_lin = self.get_param_float("ki_lin")
        kd_lin = self.get_param_float("kd_lin")
        v_fb_max = self.get_param_float("v_fb_max")
        i_lin_limit = self.get_param_float("i_lin_limit")

        abs_cte = abs(cte)
        self.i_lin += abs_cte * dt
        self.i_lin = clamp(self.i_lin, 0.0, i_lin_limit)

        d_cte = (abs_cte - abs(self.prev_cte)) / dt
        self.prev_cte = cte

        speed_penalty = kp_lin * abs_cte + ki_lin * self.i_lin + kd_lin * abs(d_cte)
        speed_penalty = clamp(speed_penalty, 0.0, v_fb_max)

        v_target = max(0.0, v_ff - speed_penalty)
        w_target = w_ff + w_fb

        max_w = self.get_param_float("max_w")
        w_target = clamp(w_target, -max_w, max_w)

        # Reacquire behavior: if robot is far from path or badly oriented,
        # slow linear motion and allow angular correction.
        reacquire_distance = self.get_param_float("reacquire_distance")
        reacquire_heading = math.radians(self.get_param_float("reacquire_heading_deg"))
        reacquire_v_limit = self.get_param_float("reacquire_v_limit")
        reacquire_min_v = self.get_param_float("reacquire_min_v")
        reacquire_w_limit = self.get_param_float("reacquire_w_limit")

        if abs_cte > reacquire_distance or abs(heading_error) > reacquire_heading:
            v_target = clamp(v_target, 0.0, reacquire_v_limit)
            if goal_distance > 0.30:
                v_target = max(v_target, reacquire_min_v)
            w_target = clamp(w_target, -reacquire_w_limit, reacquire_w_limit)

        # Rate limit commands.
        profile_a_max = self.get_param_float("profile_a_max")
        profile_d_max = self.get_param_float("profile_d_max")
        profile_alpha_max = self.get_param_float("profile_alpha_max")

        if v_target >= self.last_v_cmd:
            v_cmd = self.rate_limit(v_target, self.last_v_cmd, profile_a_max, dt)
        else:
            v_cmd = self.rate_limit(v_target, self.last_v_cmd, profile_d_max, dt)

        w_cmd = self.rate_limit(w_target, self.last_w_cmd, profile_alpha_max, dt)

        v_cmd = clamp(v_cmd, 0.0, self.get_param_float("max_v"))
        w_cmd = clamp(w_cmd, -max_w, max_w)

        self.last_v_cmd = v_cmd
        self.last_w_cmd = w_cmd

        msg = Twist()
        msg.linear.x = v_cmd
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = w_cmd
        self.cmd_pub.publish(msg)

        if bool(self.get_parameter("print_debug").value):
            self.get_logger().info(
                "profiled_ctrl "
                f"idx={nearest_i}/{len(self.path_xy)-1} "
                f"progress={progress_s:.3f} remain={remaining_s:.3f} "
                f"goal={goal_distance:.3f} cte={cte:.3f} "
                f"v_ff={v_ff:.3f} w_ff={w_ff:.3f} "
                f"v={v_cmd:.3f} w={w_cmd:.3f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = PitdtProfiledPurePursuitController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_zero()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
