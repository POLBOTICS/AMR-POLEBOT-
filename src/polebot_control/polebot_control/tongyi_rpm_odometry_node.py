#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

try:
    from tf2_ros import TransformBroadcaster
except Exception:
    TransformBroadcaster = None


def yaw_to_quaternion(yaw: float):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return 0.0, 0.0, qz, qw


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class TongyiRpmOdometryNode(Node):
    """
    Hardware odometry node for Polebot AMR.

    Input:
      /tongyi_canopen_node/left/actual_rpm
      /tongyi_canopen_node/right/actual_rpm

    Output:
      /odom

    Calibration result from hardware test:
      actual_rpm_raw > 0 means wheel forward.
      Therefore default rpm_to_forward_sign is +1.0 for both wheels.

    Robot parameters:
      wheel radius     = 0.079 m
      wheel separation = 0.590 m
      gear ratio       = 32
    """

    def __init__(self):
        super().__init__("tongyi_rpm_odometry_node")

        self.declare_parameter("left_actual_rpm_topic", "/tongyi_canopen_node/left/actual_rpm")
        self.declare_parameter("right_actual_rpm_topic", "/tongyi_canopen_node/right/actual_rpm")
        self.declare_parameter("odom_topic", "/odom")

        self.declare_parameter("wheel_radius_m", 0.079)
        self.declare_parameter("wheel_separation_m", 0.600)
        self.declare_parameter("gear_ratio_left", 31.77)
        self.declare_parameter("gear_ratio_right", 31.77)

        self.declare_parameter("left_rpm_to_forward_sign", 1.0)
        self.declare_parameter("right_rpm_to_forward_sign", 1.0)

        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("publish_tf", True)

        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("feedback_timeout_s", 0.30)

        self.left_actual_rpm: Optional[float] = None
        self.right_actual_rpm: Optional[float] = None
        self.last_left_time = None
        self.last_right_time = None

        self.wheel_radius_m = float(self.get_parameter("wheel_radius_m").value)
        self.wheel_separation_m = float(self.get_parameter("wheel_separation_m").value)
        self.gear_ratio_left = float(self.get_parameter("gear_ratio_left").value)
        self.gear_ratio_right = float(self.get_parameter("gear_ratio_right").value)
        self.left_rpm_to_forward_sign = float(self.get_parameter("left_rpm_to_forward_sign").value)
        self.right_rpm_to_forward_sign = float(self.get_parameter("right_rpm_to_forward_sign").value)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.child_frame_id = str(self.get_parameter("child_frame_id").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.feedback_timeout_s = float(self.get_parameter("feedback_timeout_s").value)

        self.x_m = 0.0
        self.y_m = 0.0
        self.yaw_rad = 0.0

        self.last_update_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            10,
        )

        self.tf_broadcaster = None
        if self.publish_tf and TransformBroadcaster is not None:
            self.tf_broadcaster = TransformBroadcaster(self)
        elif self.publish_tf and TransformBroadcaster is None:
            self.get_logger().warn("tf2_ros is not available. TF will not be published.")

        self.create_subscription(
            Float64,
            str(self.get_parameter("left_actual_rpm_topic").value),
            self.left_rpm_callback,
            10,
        )

        self.create_subscription(
            Float64,
            str(self.get_parameter("right_actual_rpm_topic").value),
            self.right_rpm_callback,
            10,
        )

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / max(publish_rate_hz, 1.0), self.timer_callback)

        self.get_logger().info("tongyi_rpm_odometry_node started.")
        self.get_logger().info(f"left_actual_rpm_topic: {self.get_parameter('left_actual_rpm_topic').value}")
        self.get_logger().info(f"right_actual_rpm_topic: {self.get_parameter('right_actual_rpm_topic').value}")
        self.get_logger().info(f"odom_topic: {self.get_parameter('odom_topic').value}")
        self.get_logger().info(f"wheel_radius_m: {self.wheel_radius_m}")
        self.get_logger().info(f"wheel_separation_m: {self.wheel_separation_m}")
        self.get_logger().info(f"gear_ratio_left/right: {self.gear_ratio_left}, {self.gear_ratio_right}")
        self.get_logger().info(
            f"rpm_to_forward_sign left/right: "
            f"{self.left_rpm_to_forward_sign}, {self.right_rpm_to_forward_sign}"
        )

    def left_rpm_callback(self, msg: Float64) -> None:
        self.left_actual_rpm = float(msg.data)
        self.last_left_time = self.get_clock().now()

    def right_rpm_callback(self, msg: Float64) -> None:
        self.right_actual_rpm = float(msg.data)
        self.last_right_time = self.get_clock().now()

    def feedback_is_valid(self, now) -> bool:
        if self.left_actual_rpm is None or self.right_actual_rpm is None:
            return False

        if self.last_left_time is None or self.last_right_time is None:
            return False

        left_age_s = (now - self.last_left_time).nanoseconds * 1.0e-9
        right_age_s = (now - self.last_right_time).nanoseconds * 1.0e-9

        return left_age_s <= self.feedback_timeout_s and right_age_s <= self.feedback_timeout_s

    def compute_body_velocity(self, now) -> tuple[float, float, float, float]:
        if not self.feedback_is_valid(now):
            return 0.0, 0.0, 0.0, 0.0

        left_motor_rpm_forward = self.left_actual_rpm * self.left_rpm_to_forward_sign
        right_motor_rpm_forward = self.right_actual_rpm * self.right_rpm_to_forward_sign

        left_wheel_rpm = left_motor_rpm_forward / self.gear_ratio_left
        right_wheel_rpm = right_motor_rpm_forward / self.gear_ratio_right

        left_wheel_rad_s = left_wheel_rpm * 2.0 * math.pi / 60.0
        right_wheel_rad_s = right_wheel_rpm * 2.0 * math.pi / 60.0

        v_left_m_s = left_wheel_rad_s * self.wheel_radius_m
        v_right_m_s = right_wheel_rad_s * self.wheel_radius_m

        v_m_s = 0.5 * (v_right_m_s + v_left_m_s)
        w_rad_s = (v_right_m_s - v_left_m_s) / self.wheel_separation_m

        return v_m_s, w_rad_s, v_left_m_s, v_right_m_s

    def timer_callback(self) -> None:
        now = self.get_clock().now()
        dt_s = (now - self.last_update_time).nanoseconds * 1.0e-9
        self.last_update_time = now

        if dt_s <= 0.0 or dt_s > 0.5:
            dt_s = 0.0

        v_m_s, w_rad_s, v_left_m_s, v_right_m_s = self.compute_body_velocity(now)

        if abs(w_rad_s) < 1.0e-9:
            self.x_m += v_m_s * math.cos(self.yaw_rad) * dt_s
            self.y_m += v_m_s * math.sin(self.yaw_rad) * dt_s
        else:
            delta_yaw = w_rad_s * dt_s
            mid_yaw = self.yaw_rad + 0.5 * delta_yaw
            self.x_m += v_m_s * math.cos(mid_yaw) * dt_s
            self.y_m += v_m_s * math.sin(mid_yaw) * dt_s
            self.yaw_rad = normalize_angle(self.yaw_rad + delta_yaw)

        self.publish_odom(now, v_m_s, w_rad_s)

    def publish_odom(self, now, v_m_s: float, w_rad_s: float) -> None:
        qx, qy, qz, qw = yaw_to_quaternion(self.yaw_rad)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        odom.pose.pose.position.x = float(self.x_m)
        odom.pose.pose.position.y = float(self.y_m)
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = float(v_m_s)
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = float(w_rad_s)

        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.05

        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[7] = 0.02
        odom.twist.covariance[35] = 0.05

        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now.to_msg()
            tf_msg.header.frame_id = self.frame_id
            tf_msg.child_frame_id = self.child_frame_id
            tf_msg.transform.translation.x = float(self.x_m)
            tf_msg.transform.translation.y = float(self.y_m)
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TongyiRpmOdometryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
