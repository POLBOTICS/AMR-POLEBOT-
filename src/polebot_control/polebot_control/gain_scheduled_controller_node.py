#!/usr/bin/env python3
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
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


class GainScheduledControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('gain_scheduled_controller_node')

        # Gain scheduling operating points
        # P1: v = 0.15 m/s
        # P2: v = 0.35 m/s
        # P3: v = 0.55 m/s
        # Gain awal ini dibuat mengikuti ide:
        # 1) linearisasi lokal,
        # 2) desain gain lokal,
        # 3) interpolasi halus,
        # 4) uji di titik interpolasi.

        self.declare_parameter('speed_points', [0.15, 0.35, 0.55])
        self.declare_parameter('kx_points', [1.6, 2.0, 2.7])
        self.declare_parameter('ky_points', [9.2, 6.2, 5.1])
        self.declare_parameter('kth_points', [2.0, 2.5, 2.9])
        self.declare_parameter('max_v', 0.8)
        self.declare_parameter('max_w', 2.5)

        self.speed_points: List[float] = list(self.get_parameter('speed_points').value)
        self.kx_points: List[float] = list(self.get_parameter('kx_points').value)
        self.ky_points: List[float] = list(self.get_parameter('ky_points').value)
        self.kth_points: List[float] = list(self.get_parameter('kth_points').value)

        self.max_v = float(self.get_parameter('max_v').value)
        self.max_w = float(self.get_parameter('max_w').value)

        self.ref_pose = None
        self.ref_twist = None

        self.last_odom_time = None
        self.last_log_time = self.get_clock().now()

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.actual_path_pub = self.create_publisher(Path, '/actual_path', 10)

        self.ref_pose_sub = self.create_subscription(
            PoseStamped, '/reference_pose', self.reference_pose_callback, 10
        )
        self.ref_twist_sub = self.create_subscription(
            TwistStamped, '/reference_twist', self.reference_twist_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 20
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.actual_path = Path()
        self.actual_path.header.frame_id = 'odom'

        self.get_logger().info('Gain-scheduled controller started')

    def reference_pose_callback(self, msg: PoseStamped) -> None:
        self.ref_pose = msg

    def reference_twist_callback(self, msg: TwistStamped) -> None:
        self.ref_twist = msg

    def interpolate_gain(self, rho: float) -> Tuple[float, float, float]:
        sp = self.speed_points

        if rho <= sp[0]:
            return self.kx_points[0], self.ky_points[0], self.kth_points[0]

        if rho >= sp[-1]:
            return self.kx_points[-1], self.ky_points[-1], self.kth_points[-1]

        for i in range(len(sp) - 1):
            if sp[i] <= rho <= sp[i + 1]:
                beta = (rho - sp[i]) / (sp[i + 1] - sp[i])

                kx = (1.0 - beta) * self.kx_points[i] + beta * self.kx_points[i + 1]
                ky = (1.0 - beta) * self.ky_points[i] + beta * self.ky_points[i + 1]
                kth = (1.0 - beta) * self.kth_points[i] + beta * self.kth_points[i + 1]
                return kx, ky, kth

        return self.kx_points[-1], self.ky_points[-1], self.kth_points[-1]

    def publish_tf_from_odom(self, odom: Odometry) -> None:
        tf_msg = TransformStamped()
        tf_msg.header.stamp = odom.header.stamp
        tf_msg.header.frame_id = odom.header.frame_id if odom.header.frame_id else 'odom'
        tf_msg.child_frame_id = odom.child_frame_id if odom.child_frame_id else 'base_link'

        tf_msg.transform.translation.x = odom.pose.pose.position.x
        tf_msg.transform.translation.y = odom.pose.pose.position.y
        tf_msg.transform.translation.z = odom.pose.pose.position.z

        tf_msg.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)

    def odom_callback(self, odom: Odometry) -> None:
        self.publish_tf_from_odom(odom)

        self.actual_path.header.stamp = odom.header.stamp
        self.actual_path.header.frame_id = odom.header.frame_id if odom.header.frame_id else 'odom'

        pose_stamped = PoseStamped()
        pose_stamped.header = odom.header
        pose_stamped.pose = odom.pose.pose
        self.actual_path.poses.append(pose_stamped)
        if len(self.actual_path.poses) > 5000:
            self.actual_path.poses.pop(0)
        self.actual_path_pub.publish(self.actual_path)

        if self.ref_pose is None or self.ref_twist is None:
            return

        now_sec = odom.header.stamp.sec + odom.header.stamp.nanosec * 1e-9
        if self.last_odom_time is None:
            dt = 0.0
        else:
            dt = now_sec - self.last_odom_time
        self.last_odom_time = now_sec

        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y

        q = odom.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

        xr = self.ref_pose.pose.position.x
        yr = self.ref_pose.pose.position.y

        qr = self.ref_pose.pose.orientation
        yaw_r = quaternion_to_yaw(qr.x, qr.y, qr.z, qr.w)

        dx = xr - x
        dy = yr - y

        # Error dalam body frame
        ex = math.cos(yaw) * dx + math.sin(yaw) * dy
        ey = -math.sin(yaw) * dx + math.cos(yaw) * dy
        etheta = wrap_to_pi(yaw_r - yaw)

        v_ref = float(self.ref_twist.twist.linear.x)

        # Scheduling variable: kecepatan referensi
        rho = max(abs(v_ref), self.speed_points[0])
        kx, ky, kth = self.interpolate_gain(rho)

        # Motion control
        v_cmd = v_ref * math.cos(etheta) + kx * ex
        w_cmd = ky * ey + kth * etheta

        v_cmd = clamp(v_cmd, -self.max_v, self.max_v)
        w_cmd = clamp(w_cmd, -self.max_w, self.max_w)

        # Stop halus di akhir lintasan
        if abs(v_ref) < 1e-6 and abs(ex) < 0.02 and abs(ey) < 0.02 and abs(etheta) < 0.03:
            v_cmd = 0.0
            w_cmd = 0.0

        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = w_cmd
        self.cmd_pub.publish(cmd)

        # Logging periodik
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds > 1_000_000_000:
            self.last_log_time = now
            self.get_logger().info(
                f"dt={dt:.4f} | rho={rho:.2f} | "
                f"K=({kx:.2f}, {ky:.2f}, {kth:.2f}) | "
                f"e=({ex:.3f}, {ey:.3f}, {etheta:.3f}) | "
                f"u=({v_cmd:.3f}, {w_cmd:.3f})"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GainScheduledControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()