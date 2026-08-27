#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class CmdVelToTongyiRpmNode(Node):
    """
    Adapter node:
      /cmd_vel
        -> /tongyi_canopen_node/left/target_rpm
        -> /tongyi_canopen_node/right/target_rpm

    Robot convention:
      linear.x  > 0  = forward
      angular.z > 0  = rotate left / CCW

    Hardware result from direct RPM test:
      +RPM = wheel moves backward
      -RPM = wheel moves forward

    Therefore default command signs are:
      left_command_sign  = -1
      right_command_sign = -1
    """

    def __init__(self):
        super().__init__("cmd_vel_to_tongyi_rpm_node")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("left_target_rpm_topic", "/tongyi_canopen_node/left/target_rpm")
        self.declare_parameter("right_target_rpm_topic", "/tongyi_canopen_node/right/target_rpm")

        self.declare_parameter("wheel_radius_m", 0.079)
        self.declare_parameter("wheel_separation_m", 0.590)
        self.declare_parameter("gear_ratio_left", 32.0)
        self.declare_parameter("gear_ratio_right", 32.0)

        self.declare_parameter("left_command_sign", -1.0)
        self.declare_parameter("right_command_sign", -1.0)

        # Safety limits for first hardware test.
        # Increase only after motor direction and stop behavior are confirmed.
        self.declare_parameter("max_motor_rpm", 30.0)

        # RPM slew-rate limit. This reduces sudden RPM jumps.
        # Unit: motor RPM per second.
        self.declare_parameter("rpm_rate_limit", 120.0)

        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("cmd_timeout_s", 0.30)

        self.wheel_radius_m = float(self.get_parameter("wheel_radius_m").value)
        self.wheel_separation_m = float(self.get_parameter("wheel_separation_m").value)
        self.gear_ratio_left = float(self.get_parameter("gear_ratio_left").value)
        self.gear_ratio_right = float(self.get_parameter("gear_ratio_right").value)
        self.left_command_sign = float(self.get_parameter("left_command_sign").value)
        self.right_command_sign = float(self.get_parameter("right_command_sign").value)
        self.max_motor_rpm = abs(float(self.get_parameter("max_motor_rpm").value))
        self.rpm_rate_limit = abs(float(self.get_parameter("rpm_rate_limit").value))
        self.cmd_timeout_s = float(self.get_parameter("cmd_timeout_s").value)

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.dt = 1.0 / max(publish_rate_hz, 1.0)

        self.latest_cmd: Optional[Twist] = None
        self.last_cmd_time = self.get_clock().now()

        self.left_rpm_cmd = 0.0
        self.right_rpm_cmd = 0.0

        self.left_pub = self.create_publisher(
            Float64,
            str(self.get_parameter("left_target_rpm_topic").value),
            10,
        )
        self.right_pub = self.create_publisher(
            Float64,
            str(self.get_parameter("right_target_rpm_topic").value),
            10,
        )

        self.cmd_sub = self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self.cmd_callback,
            10,
        )

        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info("cmd_vel_to_tongyi_rpm_node started.")
        self.get_logger().info(f"cmd_vel_topic: {self.get_parameter('cmd_vel_topic').value}")
        self.get_logger().info(f"left_target_rpm_topic: {self.get_parameter('left_target_rpm_topic').value}")
        self.get_logger().info(f"right_target_rpm_topic: {self.get_parameter('right_target_rpm_topic').value}")
        self.get_logger().info(f"wheel_radius_m: {self.wheel_radius_m}")
        self.get_logger().info(f"wheel_separation_m: {self.wheel_separation_m}")
        self.get_logger().info(f"gear_ratio_left/right: {self.gear_ratio_left}, {self.gear_ratio_right}")
        self.get_logger().info(f"command signs left/right: {self.left_command_sign}, {self.right_command_sign}")
        self.get_logger().info(f"max_motor_rpm: {self.max_motor_rpm}")
        self.get_logger().info(f"rpm_rate_limit: {self.rpm_rate_limit}")

    def cmd_callback(self, msg: Twist) -> None:
        self.latest_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    @staticmethod
    def clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    @staticmethod
    def slew(current: float, target: float, max_delta: float) -> float:
        delta = target - current
        if delta > max_delta:
            return current + max_delta
        if delta < -max_delta:
            return current - max_delta
        return target

    def compute_target_motor_rpm(self) -> tuple[float, float]:
        now = self.get_clock().now()
        age_s = (now - self.last_cmd_time).nanoseconds * 1.0e-9

        if self.latest_cmd is None or age_s > self.cmd_timeout_s:
            return 0.0, 0.0

        v = float(self.latest_cmd.linear.x)
        w = float(self.latest_cmd.angular.z)

        v_left = v - (w * self.wheel_separation_m / 2.0)
        v_right = v + (w * self.wheel_separation_m / 2.0)

        wheel_rpm_left = v_left / (2.0 * math.pi * self.wheel_radius_m) * 60.0
        wheel_rpm_right = v_right / (2.0 * math.pi * self.wheel_radius_m) * 60.0

        motor_rpm_left = (
            wheel_rpm_left
            * self.gear_ratio_left
            * self.left_command_sign
        )
        motor_rpm_right = (
            wheel_rpm_right
            * self.gear_ratio_right
            * self.right_command_sign
        )

        motor_rpm_left = self.clamp(motor_rpm_left, self.max_motor_rpm)
        motor_rpm_right = self.clamp(motor_rpm_right, self.max_motor_rpm)

        return motor_rpm_left, motor_rpm_right

    def timer_callback(self) -> None:
        target_left, target_right = self.compute_target_motor_rpm()

        max_delta = self.rpm_rate_limit * self.dt

        self.left_rpm_cmd = self.slew(self.left_rpm_cmd, target_left, max_delta)
        self.right_rpm_cmd = self.slew(self.right_rpm_cmd, target_right, max_delta)

        left_msg = Float64()
        right_msg = Float64()

        left_msg.data = float(self.left_rpm_cmd)
        right_msg.data = float(self.right_rpm_cmd)

        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)


def main(args=None):
    rclpy.init(args=args)

    node = CmdVelToTongyiRpmNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        zero_left = Float64()
        zero_right = Float64()
        zero_left.data = 0.0
        zero_right.data = 0.0

        for _ in range(5):
            node.left_pub.publish(zero_left)
            node.right_pub.publish(zero_right)

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
