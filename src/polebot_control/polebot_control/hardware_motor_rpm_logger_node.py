#!/usr/bin/env python3

import csv
import math
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class HardwareMotorRpmLogger(Node):
    """
    Hardware motor RPM validation logger for Polebot AMR.

    This node logs:
      - /cmd_vel
      - expected target motor RPM from differential-drive kinematics
      - feedback motor RPM from either Float64 topics or JointState
      - /odom

    Feedback modes:
      1. feedback_source = "rpm_topics"
         left/right feedback are std_msgs/Float64 in motor RPM.

      2. feedback_source = "joint_states"
         feedback is sensor_msgs/JointState velocity in rad/s at wheel joint.
         The node converts wheel rad/s to motor RPM using gear_ratio.
    """

    def __init__(self):
        super().__init__("hardware_motor_rpm_logger_node")

        self.declare_parameter("trial_name", "hardware_motor_rpm_validation_trial_001")
        self.declare_parameter("output_dir", str(Path.home() / "polebot_hardware_logs"))

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")

        self.declare_parameter("feedback_source", "rpm_topics")

        self.declare_parameter("feedback_left_rpm_topic", "/tongyi/left_motor_rpm")
        self.declare_parameter("feedback_right_rpm_topic", "/tongyi/right_motor_rpm")

        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("left_joint_name", "drivewhl_l_joint")
        self.declare_parameter("right_joint_name", "drivewhl_r_joint")

        self.declare_parameter("wheel_radius_m", 0.079)
        self.declare_parameter("wheel_separation_m", 0.590)
        self.declare_parameter("gear_ratio_left", 32.0)
        self.declare_parameter("gear_ratio_right", 32.0)

        # Sign convention for expected feedback RPM.
        # Based on your initial hardware note:
        # left raw -1000 => -100 rpm, right raw +997 => +99.7 rpm.
        # For forward motion, expected feedback magnitude is same,
        # but left may be negative and right may be positive.
        self.declare_parameter("left_feedback_sign", -1.0)
        self.declare_parameter("right_feedback_sign", 1.0)

        self.declare_parameter("log_rate_hz", 30.0)
        self.declare_parameter("flush_every_n_samples", 10)

        self.trial_name = str(self.get_parameter("trial_name").value)
        self.output_dir = Path(str(self.get_parameter("output_dir").value)).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.csv_path = self.output_dir / f"{self.trial_name}.csv"

        self.wheel_radius_m = float(self.get_parameter("wheel_radius_m").value)
        self.wheel_separation_m = float(self.get_parameter("wheel_separation_m").value)
        self.gear_ratio_left = float(self.get_parameter("gear_ratio_left").value)
        self.gear_ratio_right = float(self.get_parameter("gear_ratio_right").value)
        self.left_feedback_sign = float(self.get_parameter("left_feedback_sign").value)
        self.right_feedback_sign = float(self.get_parameter("right_feedback_sign").value)

        self.feedback_source = str(self.get_parameter("feedback_source").value)

        self.cmd_v_m_s = 0.0
        self.cmd_w_rad_s = 0.0

        self.feedback_rpm_left: Optional[float] = None
        self.feedback_rpm_right: Optional[float] = None

        self.odom_x_m: Optional[float] = None
        self.odom_y_m: Optional[float] = None
        self.odom_yaw_rad: Optional[float] = None
        self.odom_v_m_s: Optional[float] = None
        self.odom_w_rad_s: Optional[float] = None

        self.sample_count = 0
        self.start_time = self.get_clock().now()

        self.cmd_sub = self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self.cmd_callback,
            10,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.odom_callback,
            10,
        )

        if self.feedback_source == "rpm_topics":
            self.left_rpm_sub = self.create_subscription(
                Float64,
                str(self.get_parameter("feedback_left_rpm_topic").value),
                self.left_rpm_callback,
                10,
            )
            self.right_rpm_sub = self.create_subscription(
                Float64,
                str(self.get_parameter("feedback_right_rpm_topic").value),
                self.right_rpm_callback,
                10,
            )
        elif self.feedback_source == "joint_states":
            self.joint_sub = self.create_subscription(
                JointState,
                str(self.get_parameter("joint_states_topic").value),
                self.joint_state_callback,
                10,
            )
        else:
            raise ValueError(
                "feedback_source must be 'rpm_topics' or 'joint_states'"
            )

        self.csv_file = self.csv_path.open("w", newline="")
        self.writer = csv.DictWriter(
            self.csv_file,
            fieldnames=[
                "time_s",
                "trial_name",
                "cmd_v_m_s",
                "cmd_w_rad_s",
                "target_wheel_linear_left_m_s",
                "target_wheel_linear_right_m_s",
                "target_wheel_rpm_left",
                "target_wheel_rpm_right",
                "target_motor_rpm_left_expected_feedback_sign",
                "target_motor_rpm_right_expected_feedback_sign",
                "feedback_rpm_left",
                "feedback_rpm_right",
                "rpm_error_left",
                "rpm_error_right",
                "abs_rpm_error_left",
                "abs_rpm_error_right",
                "odom_x_m",
                "odom_y_m",
                "odom_yaw_rad",
                "odom_v_m_s",
                "odom_w_rad_s",
                "feedback_source",
            ],
        )
        self.writer.writeheader()

        log_rate_hz = float(self.get_parameter("log_rate_hz").value)
        self.timer = self.create_timer(1.0 / max(log_rate_hz, 1.0), self.log_sample)

        self.get_logger().info("Hardware motor RPM logger started.")
        self.get_logger().info(f"CSV output: {self.csv_path}")
        self.get_logger().info(f"feedback_source: {self.feedback_source}")
        self.get_logger().info(f"wheel_radius_m: {self.wheel_radius_m}")
        self.get_logger().info(f"wheel_separation_m: {self.wheel_separation_m}")
        self.get_logger().info(f"gear_ratio_left/right: {self.gear_ratio_left}, {self.gear_ratio_right}")
        self.get_logger().info(f"feedback signs left/right: {self.left_feedback_sign}, {self.right_feedback_sign}")

    def cmd_callback(self, msg: Twist) -> None:
        self.cmd_v_m_s = float(msg.linear.x)
        self.cmd_w_rad_s = float(msg.angular.z)

    def odom_callback(self, msg: Odometry) -> None:
        self.odom_x_m = float(msg.pose.pose.position.x)
        self.odom_y_m = float(msg.pose.pose.position.y)
        self.odom_yaw_rad = yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_v_m_s = float(msg.twist.twist.linear.x)
        self.odom_w_rad_s = float(msg.twist.twist.angular.z)

    def left_rpm_callback(self, msg: Float64) -> None:
        self.feedback_rpm_left = float(msg.data)

    def right_rpm_callback(self, msg: Float64) -> None:
        self.feedback_rpm_right = float(msg.data)

    def joint_state_callback(self, msg: JointState) -> None:
        left_name = str(self.get_parameter("left_joint_name").value)
        right_name = str(self.get_parameter("right_joint_name").value)

        name_to_index = {name: i for i, name in enumerate(msg.name)}

        if left_name in name_to_index:
            i = name_to_index[left_name]
            if i < len(msg.velocity):
                wheel_rad_s = float(msg.velocity[i])
                wheel_rpm = wheel_rad_s * 60.0 / (2.0 * math.pi)
                self.feedback_rpm_left = wheel_rpm * self.gear_ratio_left * self.left_feedback_sign

        if right_name in name_to_index:
            i = name_to_index[right_name]
            if i < len(msg.velocity):
                wheel_rad_s = float(msg.velocity[i])
                wheel_rpm = wheel_rad_s * 60.0 / (2.0 * math.pi)
                self.feedback_rpm_right = wheel_rpm * self.gear_ratio_right * self.right_feedback_sign

    def compute_target_rpm(self):
        v = self.cmd_v_m_s
        w = self.cmd_w_rad_s
        l = self.wheel_separation_m
        r = self.wheel_radius_m

        v_left = v - (w * l / 2.0)
        v_right = v + (w * l / 2.0)

        wheel_rpm_left = v_left / (2.0 * math.pi * r) * 60.0
        wheel_rpm_right = v_right / (2.0 * math.pi * r) * 60.0

        motor_rpm_left = wheel_rpm_left * self.gear_ratio_left * self.left_feedback_sign
        motor_rpm_right = wheel_rpm_right * self.gear_ratio_right * self.right_feedback_sign

        return v_left, v_right, wheel_rpm_left, wheel_rpm_right, motor_rpm_left, motor_rpm_right

    @staticmethod
    def safe_error(target: Optional[float], feedback: Optional[float]) -> Optional[float]:
        if target is None or feedback is None:
            return None
        return target - feedback

    def log_sample(self) -> None:
        now = self.get_clock().now()
        time_s = (now - self.start_time).nanoseconds * 1.0e-9

        (
            v_left,
            v_right,
            wheel_rpm_left,
            wheel_rpm_right,
            motor_rpm_left,
            motor_rpm_right,
        ) = self.compute_target_rpm()

        rpm_error_left = self.safe_error(motor_rpm_left, self.feedback_rpm_left)
        rpm_error_right = self.safe_error(motor_rpm_right, self.feedback_rpm_right)

        row = {
            "time_s": f"{time_s:.6f}",
            "trial_name": self.trial_name,
            "cmd_v_m_s": f"{self.cmd_v_m_s:.6f}",
            "cmd_w_rad_s": f"{self.cmd_w_rad_s:.6f}",
            "target_wheel_linear_left_m_s": f"{v_left:.6f}",
            "target_wheel_linear_right_m_s": f"{v_right:.6f}",
            "target_wheel_rpm_left": f"{wheel_rpm_left:.6f}",
            "target_wheel_rpm_right": f"{wheel_rpm_right:.6f}",
            "target_motor_rpm_left_expected_feedback_sign": f"{motor_rpm_left:.6f}",
            "target_motor_rpm_right_expected_feedback_sign": f"{motor_rpm_right:.6f}",
            "feedback_rpm_left": "" if self.feedback_rpm_left is None else f"{self.feedback_rpm_left:.6f}",
            "feedback_rpm_right": "" if self.feedback_rpm_right is None else f"{self.feedback_rpm_right:.6f}",
            "rpm_error_left": "" if rpm_error_left is None else f"{rpm_error_left:.6f}",
            "rpm_error_right": "" if rpm_error_right is None else f"{rpm_error_right:.6f}",
            "abs_rpm_error_left": "" if rpm_error_left is None else f"{abs(rpm_error_left):.6f}",
            "abs_rpm_error_right": "" if rpm_error_right is None else f"{abs(rpm_error_right):.6f}",
            "odom_x_m": "" if self.odom_x_m is None else f"{self.odom_x_m:.6f}",
            "odom_y_m": "" if self.odom_y_m is None else f"{self.odom_y_m:.6f}",
            "odom_yaw_rad": "" if self.odom_yaw_rad is None else f"{self.odom_yaw_rad:.6f}",
            "odom_v_m_s": "" if self.odom_v_m_s is None else f"{self.odom_v_m_s:.6f}",
            "odom_w_rad_s": "" if self.odom_w_rad_s is None else f"{self.odom_w_rad_s:.6f}",
            "feedback_source": self.feedback_source,
        }

        self.writer.writerow(row)
        self.sample_count += 1

        flush_every = int(self.get_parameter("flush_every_n_samples").value)
        if self.sample_count % max(flush_every, 1) == 0:
            self.csv_file.flush()

    def close(self) -> None:
        try:
            self.csv_file.flush()
            self.csv_file.close()
            self.get_logger().info(f"Saved CSV: {self.csv_path}")
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = HardwareMotorRpmLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
