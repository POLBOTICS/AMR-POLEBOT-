#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion
from nav_msgs.msg import Path


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class StraightTrajectoryNode(Node):
    def __init__(self) -> None:
        super().__init__('straight_trajectory_node')

        self.declare_parameter('v_ref', 0.25)
        self.declare_parameter('distance', 3.0)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('frame_id', 'odom')

        self.v_ref = float(self.get_parameter('v_ref').value)
        self.distance = float(self.get_parameter('distance').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        self.ref_pose_pub = self.create_publisher(PoseStamped, '/reference_pose', 10)
        self.ref_twist_pub = self.create_publisher(TwistStamped, '/reference_twist', 10)
        self.ref_path_pub = self.create_publisher(Path, '/reference_path', 10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id

        self.start_time = None
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info(
            f"Straight trajectory started | v_ref={self.v_ref:.3f} m/s, distance={self.distance:.3f} m"
        )

    def timer_callback(self) -> None:
        now = self.get_clock().now()

        if self.start_time is None:
            self.start_time = now

        t = (now - self.start_time).nanoseconds * 1e-9
        yaw_ref = 0.0

        if self.v_ref <= 1e-6:
            x_ref = 0.0
            v_now = 0.0
        else:
            travel_time = self.distance / self.v_ref
            if t < travel_time:
                x_ref = self.v_ref * t
                v_now = self.v_ref
            else:
                x_ref = self.distance
                v_now = 0.0

        pose_msg = PoseStamped()
        pose_msg.header.stamp = now.to_msg()
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = x_ref
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation = yaw_to_quaternion(yaw_ref)

        twist_msg = TwistStamped()
        twist_msg.header.stamp = now.to_msg()
        twist_msg.header.frame_id = self.frame_id
        twist_msg.twist.linear.x = v_now
        twist_msg.twist.angular.z = 0.0

        self.ref_pose_pub.publish(pose_msg)
        self.ref_twist_pub.publish(twist_msg)

        self.path_msg.header.stamp = now.to_msg()
        self.path_msg.poses.append(pose_msg)
        if len(self.path_msg.poses) > 5000:
            self.path_msg.poses.pop(0)

        self.ref_path_pub.publish(self.path_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StraightTrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()