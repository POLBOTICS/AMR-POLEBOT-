#!/usr/bin/env python3

import csv
import math
import os
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as NavPath


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class ExperimentLoggerNode(Node):
    """
    CSV logger for Polebot DDR motion-control experiments.

    Subscribes:
      /odom            nav_msgs/Odometry
      /cmd_vel         geometry_msgs/Twist
      /reference_path  nav_msgs/Path
      /planned_path    nav_msgs/Path

    Logs:
      robot pose, command velocity, nearest path point,
      signed cross-track error, heading error, and goal distance.
    """

    def __init__(self):
        super().__init__("experiment_logger_node")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("reference_path_topic", "/reference_path")
        self.declare_parameter("planned_path_topic", "/planned_path")
        self.declare_parameter("output_dir", "~/polebot_experiment_logs")
        self.declare_parameter("trial_name", "")
        self.declare_parameter("log_rate_hz", 30.0)
        self.declare_parameter("flush_every_n_rows", 30)

        self.odom_topic = self.get_parameter("odom_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.reference_path_topic = self.get_parameter("reference_path_topic").value
        self.planned_path_topic = self.get_parameter("planned_path_topic").value
        self.output_dir = Path(os.path.expanduser(self.get_parameter("output_dir").value))
        self.trial_name = self.get_parameter("trial_name").value
        self.log_rate_hz = float(self.get_parameter("log_rate_hz").value)
        self.flush_every_n_rows = int(self.get_parameter("flush_every_n_rows").value)

        if not self.trial_name:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.trial_name = f"polebot_trial_{timestamp}"

        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.output_dir / f"{self.trial_name}.csv"

        self.latest_odom: Optional[Odometry] = None
        self.latest_cmd: Twist = Twist()

        self.path_xy: List[Tuple[float, float]] = []
        self.path_yaw: List[float] = []
        self.path_s: List[float] = []
        self.last_path_signature = None

        self.start_time_s: Optional[float] = None
        self.row_count = 0

        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow([
            "time_s",
            "ros_time_s",
            "x_m",
            "y_m",
            "yaw_rad",
            "cmd_v_m_s",
            "cmd_w_rad_s",
            "nearest_path_index",
            "nearest_path_s_m",
            "nearest_path_x_m",
            "nearest_path_y_m",
            "reference_yaw_rad",
            "cross_track_error_m",
            "abs_cross_track_error_m",
            "heading_error_rad",
            "abs_heading_error_rad",
            "distance_to_goal_m",
            "path_available",
        ])
        self.csv_file.flush()

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_callback, 20)

        self.create_subscription(
            NavPath,
            self.reference_path_topic,
            self.reference_path_callback,
            10,
        )

        self.create_subscription(
            NavPath,
            self.planned_path_topic,
            self.planned_path_callback,
            10,
        )

        period = 1.0 / max(self.log_rate_hz, 1.0)
        self.create_timer(period, self.timer_callback)

        self.get_logger().info(f"Experiment logger started.")
        self.get_logger().info(f"CSV output: {self.csv_path}")
        self.get_logger().info(f"Listening odom: {self.odom_topic}")
        self.get_logger().info(f"Listening cmd_vel: {self.cmd_vel_topic}")
        self.get_logger().info(f"Listening reference path: {self.reference_path_topic}")
        self.get_logger().info(f"Listening planned path: {self.planned_path_topic}")

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def cmd_callback(self, msg: Twist):
        self.latest_cmd = msg

    def reference_path_callback(self, msg: NavPath):
        self.update_path(msg, source_name="reference_path")

    def planned_path_callback(self, msg: NavPath):
        if not self.path_xy:
            self.update_path(msg, source_name="planned_path")

    def update_path(self, msg: NavPath, source_name: str):
        if not msg.poses:
            return

        xy: List[Tuple[float, float]] = []
        yaws: List[float] = []

        for pose_stamped in msg.poses:
            p = pose_stamped.pose.position
            q = pose_stamped.pose.orientation
            xy.append((float(p.x), float(p.y)))
            yaws.append(yaw_from_quaternion(q))

        s_values = [0.0]
        for i in range(1, len(xy)):
            dx = xy[i][0] - xy[i - 1][0]
            dy = xy[i][1] - xy[i - 1][1]
            s_values.append(s_values[-1] + math.hypot(dx, dy))

        self.path_xy = xy
        self.path_yaw = yaws
        self.path_s = s_values

        signature = (source_name, len(self.path_xy), round(self.path_s[-1], 6))

        if signature != self.last_path_signature:
            self.get_logger().info(
                f"Loaded {source_name}: {len(self.path_xy)} points, length={self.path_s[-1]:.3f} m"
            )
            self.last_path_signature = signature

    def current_ros_time_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def nearest_path_metrics(self, x: float, y: float, yaw: float):
        if not self.path_xy:
            return {
                "idx": -1,
                "s": float("nan"),
                "px": float("nan"),
                "py": float("nan"),
                "ref_yaw": float("nan"),
                "cte": float("nan"),
                "abs_cte": float("nan"),
                "heading_error": float("nan"),
                "abs_heading_error": float("nan"),
                "distance_to_goal": float("nan"),
                "path_available": 0,
            }

        best_idx = 0
        best_dist_sq = float("inf")

        for i, (px, py) in enumerate(self.path_xy):
            dx = x - px
            dy = y - py
            dist_sq = dx * dx + dy * dy
            if dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_idx = i

        px, py = self.path_xy[best_idx]

        if len(self.path_xy) >= 2:
            if best_idx == 0:
                p0 = self.path_xy[0]
                p1 = self.path_xy[1]
            elif best_idx == len(self.path_xy) - 1:
                p0 = self.path_xy[-2]
                p1 = self.path_xy[-1]
            else:
                p0 = self.path_xy[best_idx - 1]
                p1 = self.path_xy[best_idx + 1]

            tx = p1[0] - p0[0]
            ty = p1[1] - p0[1]
            norm = math.hypot(tx, ty)

            if norm > 1e-9:
                tx /= norm
                ty /= norm
                ref_yaw = math.atan2(ty, tx)

                ex = x - px
                ey = y - py

                # Left-normal sign convention.
                nx = -ty
                ny = tx
                cte = ex * nx + ey * ny
            else:
                ref_yaw = self.path_yaw[best_idx]
                cte = math.sqrt(best_dist_sq)
        else:
            ref_yaw = self.path_yaw[best_idx]
            cte = math.sqrt(best_dist_sq)

        heading_error = normalize_angle(yaw - ref_yaw)

        gx, gy = self.path_xy[-1]
        distance_to_goal = math.hypot(gx - x, gy - y)

        return {
            "idx": best_idx,
            "s": self.path_s[best_idx],
            "px": px,
            "py": py,
            "ref_yaw": ref_yaw,
            "cte": cte,
            "abs_cte": abs(cte),
            "heading_error": heading_error,
            "abs_heading_error": abs(heading_error),
            "distance_to_goal": distance_to_goal,
            "path_available": 1,
        }

    def timer_callback(self):
        if self.latest_odom is None:
            return

        ros_time_s = self.current_ros_time_s()

        if self.start_time_s is None:
            self.start_time_s = ros_time_s

        time_s = ros_time_s - self.start_time_s

        pose = self.latest_odom.pose.pose
        x = float(pose.position.x)
        y = float(pose.position.y)
        yaw = yaw_from_quaternion(pose.orientation)

        cmd_v = float(self.latest_cmd.linear.x)
        cmd_w = float(self.latest_cmd.angular.z)

        metrics = self.nearest_path_metrics(x, y, yaw)

        self.csv_writer.writerow([
            f"{time_s:.6f}",
            f"{ros_time_s:.6f}",
            f"{x:.6f}",
            f"{y:.6f}",
            f"{yaw:.6f}",
            f"{cmd_v:.6f}",
            f"{cmd_w:.6f}",
            metrics["idx"],
            f"{metrics['s']:.6f}" if math.isfinite(metrics["s"]) else "nan",
            f"{metrics['px']:.6f}" if math.isfinite(metrics["px"]) else "nan",
            f"{metrics['py']:.6f}" if math.isfinite(metrics["py"]) else "nan",
            f"{metrics['ref_yaw']:.6f}" if math.isfinite(metrics["ref_yaw"]) else "nan",
            f"{metrics['cte']:.6f}" if math.isfinite(metrics["cte"]) else "nan",
            f"{metrics['abs_cte']:.6f}" if math.isfinite(metrics["abs_cte"]) else "nan",
            f"{metrics['heading_error']:.6f}" if math.isfinite(metrics["heading_error"]) else "nan",
            f"{metrics['abs_heading_error']:.6f}" if math.isfinite(metrics["abs_heading_error"]) else "nan",
            f"{metrics['distance_to_goal']:.6f}" if math.isfinite(metrics["distance_to_goal"]) else "nan",
            metrics["path_available"],
        ])

        self.row_count += 1

        if self.row_count % self.flush_every_n_rows == 0:
            self.csv_file.flush()

    def close_csv(self):
        if hasattr(self, "csv_file") and not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()
            print(f"[experiment_logger_node] CSV saved: {self.csv_path}", flush=True)

    def destroy_node(self):
        try:
            self.close_csv()
        finally:
            super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    node = ExperimentLoggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        # Sometimes rclpy throws this while shutting down after Ctrl+C.
        # The CSV has already been flushed/closed in finally.
        if "Unable to convert call argument" in str(exc):
            print(
                f"[experiment_logger_node] Ignoring shutdown-time rclpy conversion error: {exc}",
                flush=True,
            )
        else:
            raise
    finally:
        if node is not None:
            node.close_csv()

            if rclpy.ok():
                node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
