#!/usr/bin/env python3
import math
from bisect import bisect_left
from typing import List, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class PathProfileNode(Node):
    def __init__(self) -> None:
        super().__init__('path_profile_node')

        # =========================
        # General parameters
        # =========================
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('repeat', False)
        self.declare_parameter('hold_time', 1.0)
        self.declare_parameter('samples_per_meter', 40)

        # Motion profile
        self.declare_parameter('v_max', 0.30)
        self.declare_parameter('a_max', 0.50)

        # Legacy compatibility
        self.declare_parameter('distance', 3.0)

        # Trajectory selection
        self.declare_parameter('trajectory_mode', 'complex_course')
        # modes:
        # straight, l_left, l_right, arc_left, arc_right, s_curve, complex_course

        # Geometry parameters
        self.declare_parameter('straight_1', 2.5)
        self.declare_parameter('straight_2', 2.0)
        self.declare_parameter('straight_3', 2.5)

        self.declare_parameter('turn_radius_1', 1.2)
        self.declare_parameter('turn_radius_2', 1.0)

        self.declare_parameter('turn_angle_1_deg', 90.0)
        self.declare_parameter('turn_angle_2_deg', 90.0)

        self.declare_parameter('turn_dir_1', 'right')   # left/right
        self.declare_parameter('turn_dir_2', 'left')    # left/right

        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.repeat = bool(self.get_parameter('repeat').value)
        self.hold_time = float(self.get_parameter('hold_time').value)
        self.samples_per_meter = int(self.get_parameter('samples_per_meter').value)

        self.v_max = float(self.get_parameter('v_max').value)
        self.a_max = float(self.get_parameter('a_max').value)

        self.distance = float(self.get_parameter('distance').value)

        self.trajectory_mode = str(self.get_parameter('trajectory_mode').value).lower()

        self.straight_1 = float(self.get_parameter('straight_1').value)
        self.straight_2 = float(self.get_parameter('straight_2').value)
        self.straight_3 = float(self.get_parameter('straight_3').value)

        self.turn_radius_1 = float(self.get_parameter('turn_radius_1').value)
        self.turn_radius_2 = float(self.get_parameter('turn_radius_2').value)

        self.turn_angle_1_deg = float(self.get_parameter('turn_angle_1_deg').value)
        self.turn_angle_2_deg = float(self.get_parameter('turn_angle_2_deg').value)

        self.turn_dir_1 = str(self.get_parameter('turn_dir_1').value).lower()
        self.turn_dir_2 = str(self.get_parameter('turn_dir_2').value).lower()

        if self.turn_dir_1 not in ['left', 'right']:
            self.turn_dir_1 = 'right'
        if self.turn_dir_2 not in ['left', 'right']:
            self.turn_dir_2 = 'left'

        # Publishers
        self.reference_path_pub = self.create_publisher(Path, '/reference_path', 10)
        self.planned_path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.reference_pose_pub = self.create_publisher(PoseStamped, '/reference_pose', 10)
        self.ff_twist_pub = self.create_publisher(TwistStamped, '/ff_twist', 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.profile_total_time_pub = self.create_publisher(Float32, '/profile_total_time', 10)

        # Build full trajectory
        self.points_xy = self.generate_path_points()
        self.path_s = self.compute_path_s(self.points_xy)
        self.total_length = self.path_s[-1] if len(self.path_s) > 0 else 0.0

        self.yaws = self.compute_yaws(self.points_xy)
        self.kappas = self.compute_kappas(self.path_s, self.yaws)

        self.path_msg = self.build_path_msg()
        self.goal_pose = self.make_pose_stamped(self.total_length)

        # Motion profile
        self.t_acc, self.t_flat, self.v_peak, self.total_time = self.compute_trapezoid_profile(
            self.total_length, self.v_max, self.a_max
        )

        self.start_time = None
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info(
            f'PathProfileNode started | mode={self.trajectory_mode} | '
            f'length={self.total_length:.2f} m | v_max={self.v_max:.2f} m/s | '
            f'a_max={self.a_max:.2f} m/s^2 | T={self.total_time:.2f} s'
        )

    # ============================================================
    # Path generation helpers
    # ============================================================
    def append_straight(
        self,
        points: List[Tuple[float, float]],
        heading: float,
        length: float,
    ) -> Tuple[List[Tuple[float, float]], float]:
        if length <= 1e-9:
            return points, heading

        step = 1.0 / max(self.samples_per_meter, 2)
        n = max(int(math.ceil(length / step)), 1)

        x0, y0 = points[-1]
        for i in range(1, n + 1):
            ds = length * i / n
            x = x0 + ds * math.cos(heading)
            y = y0 + ds * math.sin(heading)
            points.append((x, y))

        return points, heading

    def append_arc(
        self,
        points: List[Tuple[float, float]],
        heading: float,
        radius: float,
        angle_deg: float,
        direction: str,
    ) -> Tuple[List[Tuple[float, float]], float]:
        if radius <= 1e-9 or abs(angle_deg) <= 1e-9:
            return points, heading

        sign = 1.0 if direction == 'left' else -1.0
        angle_rad = math.radians(abs(angle_deg))
        arc_len = radius * angle_rad

        step = 1.0 / max(self.samples_per_meter, 2)
        n = max(int(math.ceil(arc_len / step)), 1)

        x0, y0 = points[-1]

        # center of arc
        cx = x0 - sign * radius * math.sin(heading)
        cy = y0 + sign * radius * math.cos(heading)

        # angle from center to current point
        phi0 = math.atan2(y0 - cy, x0 - cx)

        for i in range(1, n + 1):
            delta = sign * angle_rad * i / n
            phi = phi0 + delta
            x = cx + radius * math.cos(phi)
            y = cy + radius * math.sin(phi)
            points.append((x, y))

        heading = wrap_to_pi(heading + sign * angle_rad)
        return points, heading

    def generate_path_points(self) -> List[Tuple[float, float]]:
        points: List[Tuple[float, float]] = [(0.0, 0.0)]
        heading = 0.0

        mode = self.trajectory_mode

        if mode == 'straight':
            length = self.distance if self.distance > 0.0 else self.straight_1
            points, heading = self.append_straight(points, heading, length)

        elif mode == 'l_left':
            points, heading = self.append_straight(points, heading, self.straight_1)
            points, heading = self.append_arc(points, heading, self.turn_radius_1, self.turn_angle_1_deg, 'left')
            points, heading = self.append_straight(points, heading, self.straight_2)

        elif mode == 'l_right':
            points, heading = self.append_straight(points, heading, self.straight_1)
            points, heading = self.append_arc(points, heading, self.turn_radius_1, self.turn_angle_1_deg, 'right')
            points, heading = self.append_straight(points, heading, self.straight_2)

        elif mode == 'arc_left':
            points, heading = self.append_straight(points, heading, self.straight_1)
            points, heading = self.append_arc(points, heading, self.turn_radius_1, self.turn_angle_1_deg, 'left')
            points, heading = self.append_straight(points, heading, self.straight_2)

        elif mode == 'arc_right':
            points, heading = self.append_straight(points, heading, self.straight_1)
            points, heading = self.append_arc(points, heading, self.turn_radius_1, self.turn_angle_1_deg, 'right')
            points, heading = self.append_straight(points, heading, self.straight_2)

        elif mode == 's_curve':
            points, heading = self.append_straight(points, heading, self.straight_1)
            points, heading = self.append_arc(points, heading, self.turn_radius_1, self.turn_angle_1_deg, 'left')
            points, heading = self.append_arc(points, heading, self.turn_radius_2, self.turn_angle_2_deg, 'right')
            points, heading = self.append_straight(points, heading, self.straight_3)

        elif mode == 'u_curve':
            points, heading = self.append_straight(points, heading, self.straight_1)
            points, heading = self.append_arc(points, heading, self.turn_radius_1, 180.0, 'left')
            points, heading = self.append_straight(points, heading, self.straight_2)

        elif mode == 'figure_8':
            points, heading = self.append_straight(points, heading, 0.2)
            points, heading = self.append_arc(points, heading, self.turn_radius_1, 180.0, 'left')
            points, heading = self.append_straight(points, heading, self.straight_1) 
            points, heading = self.append_arc(points, heading, self.turn_radius_1, 180.0, 'left')
            points, heading = self.append_straight(points, heading, self.straight_1)
            points, heading = self.append_arc(points, heading, self.turn_radius_1, 180.0, 'right')
            points, heading = self.append_straight(points, heading, self.straight_1) 
            points, heading = self.append_arc(points, heading, self.turn_radius_1, 180.0, 'right')
            points, heading = self.append_straight(points, heading, self.straight_1)
            points, heading = self.append_straight(points, heading, 0.5)

        else:
            # complex_course (default)
            # lurus panjang -> belok kanan -> lurus -> belok kiri -> lurus
            points, heading = self.append_straight(points, heading, self.straight_1)
            points, heading = self.append_arc(points, heading, self.turn_radius_1, self.turn_angle_1_deg, self.turn_dir_1)
            points, heading = self.append_straight(points, heading, self.straight_2)
            points, heading = self.append_arc(points, heading, self.turn_radius_2, self.turn_angle_2_deg, self.turn_dir_2)
            points, heading = self.append_straight(points, heading, self.straight_3)

        return points

    # ============================================================
    # Path properties
    # ============================================================
    def compute_path_s(self, points: List[Tuple[float, float]]) -> List[float]:
        if not points:
            return [0.0]

        s = [0.0]
        total = 0.0
        for i in range(1, len(points)):
            dx = points[i][0] - points[i - 1][0]
            dy = points[i][1] - points[i - 1][1]
            total += math.hypot(dx, dy)
            s.append(total)
        return s

    def compute_yaws(self, points: List[Tuple[float, float]]) -> List[float]:
        n = len(points)
        if n == 0:
            return [0.0]

        yaws = [0.0] * n

        if n == 1:
            return yaws

        for i in range(n):
            if i == 0:
                dx = points[i + 1][0] - points[i][0]
                dy = points[i + 1][1] - points[i][1]
            elif i == n - 1:
                dx = points[i][0] - points[i - 1][0]
                dy = points[i][1] - points[i - 1][1]
            else:
                dx = points[i + 1][0] - points[i - 1][0]
                dy = points[i + 1][1] - points[i - 1][1]

            yaws[i] = math.atan2(dy, dx)

        return yaws

    def compute_kappas(self, s: List[float], yaws: List[float]) -> List[float]:
        n = len(s)
        if n == 0:
            return [0.0]

        kappas = [0.0] * n

        for i in range(n):
            if i == 0 or i == n - 1:
                kappas[i] = 0.0
            else:
                dyaw = wrap_to_pi(yaws[i + 1] - yaws[i - 1])
                ds = s[i + 1] - s[i - 1]
                kappas[i] = dyaw / ds if abs(ds) > 1e-9 else 0.0

        return kappas

    def interpolate_reference(self, s_query: float) -> Tuple[float, float, float, float]:
        if not self.path_s:
            return 0.0, 0.0, 0.0, 0.0

        s_query = clamp(s_query, 0.0, self.total_length)

        idx = bisect_left(self.path_s, s_query)

        if idx <= 0:
            x, y = self.points_xy[0]
            return x, y, self.yaws[0], self.kappas[0]

        if idx >= len(self.path_s):
            x, y = self.points_xy[-1]
            return x, y, self.yaws[-1], self.kappas[-1]

        s0 = self.path_s[idx - 1]
        s1 = self.path_s[idx]

        ratio = 0.0 if abs(s1 - s0) < 1e-9 else (s_query - s0) / (s1 - s0)

        x0, y0 = self.points_xy[idx - 1]
        x1, y1 = self.points_xy[idx]

        x = x0 + ratio * (x1 - x0)
        y = y0 + ratio * (y1 - y0)

        yaw0 = self.yaws[idx - 1]
        yaw1 = self.yaws[idx]
        dyaw = wrap_to_pi(yaw1 - yaw0)
        yaw = wrap_to_pi(yaw0 + ratio * dyaw)

        kappa = self.kappas[idx - 1] + ratio * (self.kappas[idx] - self.kappas[idx - 1])

        return x, y, yaw, kappa

    # ============================================================
    # Motion profile
    # ============================================================
    def compute_trapezoid_profile(self, distance: float, v_max: float, a_max: float):
        if distance <= 1e-9 or v_max <= 1e-9 or a_max <= 1e-9:
            return 0.0, 0.0, 0.0, 0.0

        t_acc = v_max / a_max
        d_acc = 0.5 * a_max * t_acc * t_acc

        # triangular
        if 2.0 * d_acc >= distance:
            t_acc = math.sqrt(distance / a_max)
            v_peak = a_max * t_acc
            t_flat = 0.0
            total_time = 2.0 * t_acc
            return t_acc, t_flat, v_peak, total_time

        # trapezoidal
        v_peak = v_max
        t_flat = (distance - 2.0 * d_acc) / v_peak
        total_time = 2.0 * t_acc + t_flat
        return t_acc, t_flat, v_peak, total_time

    def sample_profile(self, t: float) -> Tuple[float, float]:
        if self.total_time <= 1e-9:
            return 0.0, 0.0

        t1 = self.t_acc
        t2 = self.t_acc + self.t_flat
        t3 = self.total_time

        if t <= 0.0:
            return 0.0, 0.0

        # acceleration
        if t < t1:
            s = 0.5 * self.a_max * t * t
            v = self.a_max * t
            return min(s, self.total_length), min(v, self.v_peak)

        # constant velocity
        if t < t2:
            d_acc = 0.5 * self.a_max * t1 * t1
            s = d_acc + self.v_peak * (t - t1)
            v = self.v_peak
            return min(s, self.total_length), v

        # deceleration
        if t < t3:
            td = t - t2
            d_acc = 0.5 * self.a_max * t1 * t1
            d_flat = self.v_peak * self.t_flat
            s = d_acc + d_flat + self.v_peak * td - 0.5 * self.a_max * td * td
            v = max(self.v_peak - self.a_max * td, 0.0)
            return min(s, self.total_length), v

        return self.total_length, 0.0

    # ============================================================
    # ROS messages
    # ============================================================
    def make_pose_stamped(self, s_query: float) -> PoseStamped:
        x, y, yaw, _ = self.interpolate_reference(s_query)

        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation = yaw_to_quaternion(yaw)
        return pose

    def build_path_msg(self) -> Path:
        path = Path()
        path.header.frame_id = self.frame_id

        for i in range(len(self.points_xy)):
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = self.points_xy[i][0]
            pose.pose.position.y = self.points_xy[i][1]
            pose.pose.position.z = 0.0
            pose.pose.orientation = yaw_to_quaternion(self.yaws[i])
            path.poses.append(pose)

        return path

    # ============================================================
    # Main timer
    # ============================================================
    def timer_callback(self) -> None:
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now

        elapsed = (now - self.start_time).nanoseconds * 1e-9
        cycle_time = self.total_time + self.hold_time

        if self.repeat and cycle_time > 1e-9:
            elapsed = elapsed % cycle_time

        if elapsed <= self.total_time:
            s, v_ff = self.sample_profile(elapsed)
        else:
            s, v_ff = self.total_length, 0.0

        x, y, yaw, kappa = self.interpolate_reference(s)
        w_ff = kappa * v_ff

        pose_msg = PoseStamped()
        pose_msg.header.stamp = now.to_msg()
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation = yaw_to_quaternion(yaw)

        twist_msg = TwistStamped()
        twist_msg.header.stamp = now.to_msg()
        twist_msg.header.frame_id = self.frame_id
        twist_msg.twist.linear.x = v_ff
        twist_msg.twist.angular.z = w_ff

        self.path_msg.header.stamp = now.to_msg()

        goal_pose_msg = self.make_pose_stamped(self.total_length)
        goal_pose_msg.header.stamp = now.to_msg()

        total_time_msg = Float32()
        total_time_msg.data = float(self.total_time)

        self.reference_pose_pub.publish(pose_msg)
        self.ff_twist_pub.publish(twist_msg)
        self.reference_path_pub.publish(self.path_msg)
        self.planned_path_pub.publish(self.path_msg)
        self.goal_pose_pub.publish(goal_pose_msg)
        self.profile_total_time_pub.publish(total_time_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathProfileNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()