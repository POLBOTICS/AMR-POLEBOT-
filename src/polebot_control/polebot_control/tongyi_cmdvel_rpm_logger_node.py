#!/usr/bin/env python3

import csv
import math
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String


class TongyiCmdvelRpmLoggerNode(Node):
    """
    Logger khusus validasi hardware TongYi.

    Log:
      - /cmd_vel
      - /tongyi_canopen_node/left/target_rpm
      - /tongyi_canopen_node/right/target_rpm
      - /tongyi_canopen_node/left/actual_rpm
      - /tongyi_canopen_node/right/actual_rpm
      - /tongyi_canopen_node/status

    Catatan hardware hasil uji:
      - RPM negatif = roda maju
      - RPM positif = roda mundur
      - actual_rpm dari driver terbaca berlawanan tanda terhadap target command

    Maka default:
      left_actual_sign_correction  = -1.0
      right_actual_sign_correction = -1.0
    """

    def __init__(self):
        super().__init__("tongyi_cmdvel_rpm_logger_node")

        self.declare_parameter("trial_name", "hw01c_tongyi_cmdvel_rpm_validation_001")
        self.declare_parameter("output_dir", str(Path.home() / "polebot_hardware_logs"))

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("left_target_rpm_topic", "/tongyi_canopen_node/left/target_rpm")
        self.declare_parameter("right_target_rpm_topic", "/tongyi_canopen_node/right/target_rpm")
        self.declare_parameter("left_actual_rpm_topic", "/tongyi_canopen_node/left/actual_rpm")
        self.declare_parameter("right_actual_rpm_topic", "/tongyi_canopen_node/right/actual_rpm")
        self.declare_parameter("status_topic", "/tongyi_canopen_node/status")

        self.declare_parameter("left_actual_sign_correction", -1.0)
        self.declare_parameter("right_actual_sign_correction", -1.0)

        self.declare_parameter("log_rate_hz", 30.0)
        self.declare_parameter("flush_every_n_samples", 10)

        self.trial_name = str(self.get_parameter("trial_name").value)
        self.output_dir = Path(str(self.get_parameter("output_dir").value)).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.output_dir / f"{self.trial_name}.csv"

        self.left_actual_sign_correction = float(
            self.get_parameter("left_actual_sign_correction").value
        )
        self.right_actual_sign_correction = float(
            self.get_parameter("right_actual_sign_correction").value
        )

        self.cmd_v_m_s = 0.0
        self.cmd_w_rad_s = 0.0

        self.left_target_rpm: Optional[float] = None
        self.right_target_rpm: Optional[float] = None
        self.left_actual_rpm_raw: Optional[float] = None
        self.right_actual_rpm_raw: Optional[float] = None
        self.status_text = ""

        self.start_time = self.get_clock().now()
        self.sample_count = 0

        self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self.cmd_vel_callback,
            10,
        )

        self.create_subscription(
            Float64,
            str(self.get_parameter("left_target_rpm_topic").value),
            self.left_target_callback,
            10,
        )

        self.create_subscription(
            Float64,
            str(self.get_parameter("right_target_rpm_topic").value),
            self.right_target_callback,
            10,
        )

        self.create_subscription(
            Float64,
            str(self.get_parameter("left_actual_rpm_topic").value),
            self.left_actual_callback,
            10,
        )

        self.create_subscription(
            Float64,
            str(self.get_parameter("right_actual_rpm_topic").value),
            self.right_actual_callback,
            10,
        )

        self.create_subscription(
            String,
            str(self.get_parameter("status_topic").value),
            self.status_callback,
            10,
        )

        self.csv_file = self.csv_path.open("w", newline="")
        self.writer = csv.DictWriter(
            self.csv_file,
            fieldnames=[
                "time_s",
                "trial_name",
                "cmd_v_m_s",
                "cmd_w_rad_s",
                "motion_state",
                "left_target_rpm",
                "right_target_rpm",
                "left_actual_rpm_raw",
                "right_actual_rpm_raw",
                "left_actual_rpm_corrected",
                "right_actual_rpm_corrected",
                "left_rpm_error",
                "right_rpm_error",
                "left_abs_rpm_error",
                "right_abs_rpm_error",
                "status_text",
            ],
        )
        self.writer.writeheader()

        log_rate_hz = float(self.get_parameter("log_rate_hz").value)
        self.timer = self.create_timer(1.0 / max(log_rate_hz, 1.0), self.log_sample)

        self.get_logger().info("TongYi cmd_vel RPM logger started.")
        self.get_logger().info(f"CSV output: {self.csv_path}")
        self.get_logger().info(
            f"actual sign correction L/R: "
            f"{self.left_actual_sign_correction}, {self.right_actual_sign_correction}"
        )

    def cmd_vel_callback(self, msg: Twist) -> None:
        self.cmd_v_m_s = float(msg.linear.x)
        self.cmd_w_rad_s = float(msg.angular.z)

    def left_target_callback(self, msg: Float64) -> None:
        self.left_target_rpm = float(msg.data)

    def right_target_callback(self, msg: Float64) -> None:
        self.right_target_rpm = float(msg.data)

    def left_actual_callback(self, msg: Float64) -> None:
        self.left_actual_rpm_raw = float(msg.data)

    def right_actual_callback(self, msg: Float64) -> None:
        self.right_actual_rpm_raw = float(msg.data)

    def status_callback(self, msg: String) -> None:
        self.status_text = str(msg.data)

    def infer_motion_state(self) -> str:
        eps = 1.0e-6

        if abs(self.cmd_v_m_s) <= eps and abs(self.cmd_w_rad_s) <= eps:
            return "stop"

        if abs(self.cmd_v_m_s) > eps and abs(self.cmd_w_rad_s) <= eps:
            if self.cmd_v_m_s > 0.0:
                return "forward"
            return "backward"

        if abs(self.cmd_v_m_s) <= eps and abs(self.cmd_w_rad_s) > eps:
            if self.cmd_w_rad_s > 0.0:
                return "rotate_left"
            return "rotate_right"

        return "combined"

    @staticmethod
    def safe_error(target: Optional[float], actual: Optional[float]) -> Optional[float]:
        if target is None or actual is None:
            return None
        return target - actual

    @staticmethod
    def fmt(value: Optional[float]) -> str:
        if value is None:
            return ""
        if not math.isfinite(value):
            return ""
        return f"{value:.6f}"

    def log_sample(self) -> None:
        now = self.get_clock().now()
        time_s = (now - self.start_time).nanoseconds * 1.0e-9

        left_actual_corrected = None
        right_actual_corrected = None

        if self.left_actual_rpm_raw is not None:
            left_actual_corrected = (
                self.left_actual_rpm_raw * self.left_actual_sign_correction
            )

        if self.right_actual_rpm_raw is not None:
            right_actual_corrected = (
                self.right_actual_rpm_raw * self.right_actual_sign_correction
            )

        left_error = self.safe_error(self.left_target_rpm, left_actual_corrected)
        right_error = self.safe_error(self.right_target_rpm, right_actual_corrected)

        row = {
            "time_s": f"{time_s:.6f}",
            "trial_name": self.trial_name,
            "cmd_v_m_s": f"{self.cmd_v_m_s:.6f}",
            "cmd_w_rad_s": f"{self.cmd_w_rad_s:.6f}",
            "motion_state": self.infer_motion_state(),
            "left_target_rpm": self.fmt(self.left_target_rpm),
            "right_target_rpm": self.fmt(self.right_target_rpm),
            "left_actual_rpm_raw": self.fmt(self.left_actual_rpm_raw),
            "right_actual_rpm_raw": self.fmt(self.right_actual_rpm_raw),
            "left_actual_rpm_corrected": self.fmt(left_actual_corrected),
            "right_actual_rpm_corrected": self.fmt(right_actual_corrected),
            "left_rpm_error": self.fmt(left_error),
            "right_rpm_error": self.fmt(right_error),
            "left_abs_rpm_error": self.fmt(None if left_error is None else abs(left_error)),
            "right_abs_rpm_error": self.fmt(None if right_error is None else abs(right_error)),
            "status_text": self.status_text,
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
    node = TongyiCmdvelRpmLoggerNode()

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
