#!/usr/bin/env python3
import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, PoseStamped, TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def scale_small_start_error(value: float, floor_value: float) -> float:
    return max(value, floor_value)


class PITDTPurePursuitControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('pitdt_pure_pursuit_controller_node')

        self.declare_parameter('control_rate', 40.0)
        self.declare_parameter('lookahead_distance', 0.48)
        self.declare_parameter('max_v', 0.90)
        self.declare_parameter('max_w', 2.10)

        self.declare_parameter('kp_lin', 0.62)
        self.declare_parameter('ki_lin', 0.02)
        self.declare_parameter('kd_lin', 0.02)
        self.declare_parameter('u0_lin', 0.10)
        self.declare_parameter('ramp_lin', 0.40)
        self.declare_parameter('v_fb_max', 0.22)
        self.declare_parameter('i_lin_limit', 2.0)

        self.declare_parameter('kp_ang', 0.85)
        self.declare_parameter('ki_ang', 0.0)
        self.declare_parameter('kd_ang', 0.03)
        self.declare_parameter('u0_ang', 0.05)
        self.declare_parameter('ramp_ang', 0.50)
        self.declare_parameter('w_fb_max', 0.55)
        self.declare_parameter('i_ang_limit', 2.0)

        self.declare_parameter('stop_distance', 0.05)

        self.declare_parameter('reacquire_distance', 0.25)
        self.declare_parameter('reacquire_heading_deg', 40.0)
        self.declare_parameter('reacquire_v_limit', 0.22)
        self.declare_parameter('reacquire_min_v', 0.08)
        self.declare_parameter('reacquire_w_limit', 2.00)

        self.control_rate = float(self.get_parameter('control_rate').value)
        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.max_v = float(self.get_parameter('max_v').value)
        self.max_w = float(self.get_parameter('max_w').value)

        self.kp_lin = float(self.get_parameter('kp_lin').value)
        self.ki_lin = float(self.get_parameter('ki_lin').value)
        self.kd_lin = float(self.get_parameter('kd_lin').value)
        self.u0_lin = float(self.get_parameter('u0_lin').value)
        self.ramp_lin = float(self.get_parameter('ramp_lin').value)
        self.v_fb_max = float(self.get_parameter('v_fb_max').value)
        self.i_lin_limit = float(self.get_parameter('i_lin_limit').value)

        self.kp_ang = float(self.get_parameter('kp_ang').value)
        self.ki_ang = float(self.get_parameter('ki_ang').value)
        self.kd_ang = float(self.get_parameter('kd_ang').value)
        self.u0_ang = float(self.get_parameter('u0_ang').value)
        self.ramp_ang = float(self.get_parameter('ramp_ang').value)
        self.w_fb_max = float(self.get_parameter('w_fb_max').value)
        self.i_ang_limit = float(self.get_parameter('i_ang_limit').value)

        self.stop_distance = float(self.get_parameter('stop_distance').value)

        self.reacquire_distance = float(self.get_parameter('reacquire_distance').value)
        self.reacquire_heading = math.radians(float(self.get_parameter('reacquire_heading_deg').value))
        self.reacquire_v_limit = float(self.get_parameter('reacquire_v_limit').value)
        self.reacquire_min_v = float(self.get_parameter('reacquire_min_v').value)
        self.reacquire_w_limit = float(self.get_parameter('reacquire_w_limit').value)

        self.path_msg: Optional[Path] = None
        self.path_points: List[Tuple[float, float]] = []
        self.path_s: List[float] = []
        self.total_path_length = 0.0

        self.goal_pose: Optional[PoseStamped] = None
        self.ff_twist: Optional[TwistStamped] = None
        self.profile_total_time = 1.0

        self.current_x: Optional[float] = None
        self.current_y: Optional[float] = None
        self.current_yaw: Optional[float] = None
        self.current_z: float = 0.0
        self.pose_source = 'none'

        self.t0 = None
        self.last_control_time = None
        self.last_log_time = self.get_clock().now()

        self.lin_int = 0.0
        self.ang_int = 0.0
        self.prev_lin_e = 0.0
        self.prev_ang_e = 0.0
        self.lin_start_error = None
        self.ang_start_error = None

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.actual_path_pub = self.create_publisher(Path, '/actual_path', 10)

        self.path_sub = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.goal_pose_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.ff_twist_sub = self.create_subscription(TwistStamped, '/ff_twist', self.ff_twist_callback, 10)
        self.profile_total_time_sub = self.create_subscription(Float32, '/profile_total_time', self.profile_total_time_callback, 10)

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 20)
        self.model_pose_sub = self.create_subscription(PoseArray, '/model/ddr/pose', self.model_pose_callback, 20)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.actual_path = Path()
        self.actual_path.header.frame_id = 'odom'

        self.control_timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info('Pure pursuit controller started with /model/ddr/pose sync (moderate speed)')

    def path_callback(self, msg: Path) -> None:
        self.path_msg = msg
        self.path_points = []
        self.path_s = []

        total = 0.0
        last_pt = None
        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            self.path_points.append((x, y))

            if last_pt is None:
                self.path_s.append(0.0)
            else:
                ds = math.hypot(x - last_pt[0], y - last_pt[1])
                total += ds
                self.path_s.append(total)
            last_pt = (x, y)

        self.total_path_length = total

    def goal_pose_callback(self, msg: PoseStamped) -> None:
        self.goal_pose = msg

    def ff_twist_callback(self, msg: TwistStamped) -> None:
        self.ff_twist = msg
        if self.t0 is None:
            self.t0 = self.get_clock().now()

    def profile_total_time_callback(self, msg: Float32) -> None:
        self.profile_total_time = max(float(msg.data), 1e-3)

    def model_pose_callback(self, msg: PoseArray) -> None:
        if len(msg.poses) == 0:
            return

        pose = msg.poses[0]

        self.current_x = pose.position.x
        self.current_y = pose.position.y
        self.current_z = pose.position.z
        self.current_yaw = quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )
        self.pose_source = 'model_pose'

        now = self.get_clock().now()
        self.publish_tf_and_path(
            x=self.current_x,
            y=self.current_y,
            z=self.current_z,
            yaw=self.current_yaw,
            stamp=now,
        )

    def odom_callback(self, odom: Odometry) -> None:
        if self.pose_source == 'model_pose':
            return

        self.current_x = odom.pose.pose.position.x
        self.current_y = odom.pose.pose.position.y
        self.current_z = odom.pose.pose.position.z
        self.current_yaw = quaternion_to_yaw(
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w
        )
        self.pose_source = 'odom'

        now = self.get_clock().now()
        self.publish_tf_and_path(
            x=self.current_x,
            y=self.current_y,
            z=self.current_z,
            yaw=self.current_yaw,
            stamp=now,
        )

    def publish_tf_and_path(self, x: float, y: float, z: float, yaw: float, stamp) -> None:
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp.to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = z
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = math.sin(yaw / 2.0)
        tf_msg.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf_broadcaster.sendTransform(tf_msg)

        self.actual_path.header.stamp = stamp.to_msg()
        self.actual_path.header.frame_id = 'odom'

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = stamp.to_msg()
        pose_stamped.header.frame_id = 'odom'
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = math.sin(yaw / 2.0)
        pose_stamped.pose.orientation.w = math.cos(yaw / 2.0)

        self.actual_path.poses.append(pose_stamped)
        if len(self.actual_path.poses) > 5000:
            self.actual_path.poses.pop(0)
        self.actual_path_pub.publish(self.actual_path)

    def nearest_path_index(self, x: float, y: float) -> int:
        best_i = 0
        best_d2 = float('inf')
        for i, (px, py) in enumerate(self.path_points):
            d2 = (px - x) ** 2 + (py - y) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        return best_i

    def lookahead_point(self, x: float, y: float) -> Tuple[float, float, float]:
        nearest_i = self.nearest_path_index(x, y)
        s_nearest = self.path_s[nearest_i]
        target_s = min(s_nearest + self.lookahead_distance, self.total_path_length)

        if target_s <= self.path_s[0]:
            return self.path_points[0][0], self.path_points[0][1], 0.0

        for i in range(len(self.path_s) - 1):
            s0 = self.path_s[i]
            s1 = self.path_s[i + 1]
            if s0 <= target_s <= s1:
                ratio = 0.0 if abs(s1 - s0) < 1e-9 else (target_s - s0) / (s1 - s0)
                x0, y0 = self.path_points[i]
                x1, y1 = self.path_points[i + 1]
                lx = x0 + ratio * (x1 - x0)
                ly = y0 + ratio * (y1 - y0)
                return lx, ly, s_nearest

        lx, ly = self.path_points[-1]
        return lx, ly, s_nearest

    def pitdt_like_output(
        self,
        e_norm: float,
        e_int: float,
        e_dot: float,
        t_elapsed: float,
        T_total: float,
        kp: float,
        ki: float,
        kd: float,
        u0: float,
        ramp: float,
    ) -> float:
        tau = max(t_elapsed / max(T_total, 1e-3) + 1.0, 1.0)
        ramp_term = min(1.0, u0 + ramp * max(0.0, 1.0 - abs(e_norm)))

        u_p = kp * ramp_term * e_norm
        u_i = ki * math.copysign(math.sqrt(abs(e_int)), e_int) * tau
        u_d = kd * (e_dot / (tau ** 4))
        return u_p + u_i + u_d

    def control_loop(self) -> None:
        if (
            self.current_x is None
            or self.current_y is None
            or self.current_yaw is None
            or self.goal_pose is None
            or self.ff_twist is None
            or self.path_msg is None
            or len(self.path_points) < 2
        ):
            return

        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9

        if self.last_control_time is None:
            self.last_control_time = now_sec
            return

        dt = max(now_sec - self.last_control_time, 1e-6)
        self.last_control_time = now_sec

        if self.t0 is None:
            self.t0 = self.get_clock().now()
        t_elapsed = (self.get_clock().now() - self.t0).nanoseconds * 1e-9

        x = self.current_x
        y = self.current_y
        yaw = self.current_yaw

        goal_yaw = quaternion_to_yaw(
            self.goal_pose.pose.orientation.x,
            self.goal_pose.pose.orientation.y,
            self.goal_pose.pose.orientation.z,
            self.goal_pose.pose.orientation.w,
        )

        look_x, look_y, s_nearest = self.lookahead_point(x, y)

        dxl = look_x - x
        dyl = look_y - y

        x_r = math.cos(yaw) * dxl + math.sin(yaw) * dyl
        y_r = -math.sin(yaw) * dxl + math.cos(yaw) * dyl

        heading_error = wrap_to_pi(math.atan2(y_r, x_r))
        goal_heading_error = wrap_to_pi(goal_yaw - yaw)
        cross_track_error = y_r

        remaining_distance = max(self.total_path_length - s_nearest, 0.0)

        if self.lin_start_error is None:
            self.lin_start_error = scale_small_start_error(max(self.total_path_length, 1e-3), 0.20)

        if self.ang_start_error is None:
            self.ang_start_error = scale_small_start_error(abs(heading_error), 0.20)

        e_lin = remaining_distance / self.lin_start_error
        e_ang = heading_error / self.ang_start_error

        self.lin_int = clamp(self.lin_int + e_lin * dt, -self.i_lin_limit, self.i_lin_limit)
        self.ang_int = clamp(self.ang_int + e_ang * dt, -self.i_ang_limit, self.i_ang_limit)

        e_lin_dot = (e_lin - self.prev_lin_e) / dt
        e_ang_dot = (e_ang - self.prev_ang_e) / dt
        self.prev_lin_e = e_lin
        self.prev_ang_e = e_ang

        u_lin = self.pitdt_like_output(
            e_norm=e_lin,
            e_int=self.lin_int,
            e_dot=e_lin_dot,
            t_elapsed=t_elapsed,
            T_total=self.profile_total_time,
            kp=self.kp_lin,
            ki=self.ki_lin,
            kd=self.kd_lin,
            u0=self.u0_lin,
            ramp=self.ramp_lin,
        )

        u_ang = self.pitdt_like_output(
            e_norm=e_ang,
            e_int=self.ang_int,
            e_dot=e_ang_dot,
            t_elapsed=t_elapsed,
            T_total=self.profile_total_time,
            kp=self.kp_ang,
            ki=self.ki_ang,
            kd=self.kd_ang,
            u0=self.u0_ang,
            ramp=self.ramp_ang,
        )

        v_ff = float(self.ff_twist.twist.linear.x)
        w_ff = float(self.ff_twist.twist.angular.z)

        Ld = max(math.hypot(x_r, y_r), 1e-6)
        w_pp = v_ff * (2.0 * y_r / (Ld * Ld))

        base_v_cmd = v_ff + self.v_fb_max * u_lin
        base_w_cmd = w_ff + w_pp + self.w_fb_max * u_ang

        reacquire_mode = (
            abs(cross_track_error) > self.reacquire_distance
            or abs(heading_error) > self.reacquire_heading
        )

        if reacquire_mode:
            heading_scale = max(0.10, math.cos(heading_error))
            v_cmd = clamp(base_v_cmd * heading_scale, 0.0, self.reacquire_v_limit)

            if abs(cross_track_error) > self.reacquire_distance and v_cmd < self.reacquire_min_v:
                v_cmd = self.reacquire_min_v

            w_cmd = clamp(base_w_cmd, -self.reacquire_w_limit, self.reacquire_w_limit)
        else:
            heading_scale = max(0.30, math.cos(heading_error))
            v_cmd = base_v_cmd * heading_scale
            w_cmd = base_w_cmd

        if remaining_distance < self.stop_distance:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        v_cmd = clamp(v_cmd, -self.max_v, self.max_v)
        w_cmd = clamp(w_cmd, -self.max_w, self.max_w)

        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = w_cmd
        self.cmd_pub.publish(cmd)

        if (now - self.last_log_time).nanoseconds > 1_000_000_000:
            self.last_log_time = now
            self.get_logger().info(
                f'src={self.pose_source} | '
                f't={t_elapsed:.2f}/{self.profile_total_time:.2f} | '
                f'rem={remaining_distance:.3f} | '
                f'cte={cross_track_error:.3f} | '
                f'e_lin={e_lin:.3f} e_ang={e_ang:.3f} | '
                f'goal_h={goal_heading_error:.3f} | '
                f'reacq={reacquire_mode} | '
                f'ff=({v_ff:.3f},{w_ff:.3f}) | '
                f'cmd=({v_cmd:.3f},{w_cmd:.3f})'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PITDTPurePursuitControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()