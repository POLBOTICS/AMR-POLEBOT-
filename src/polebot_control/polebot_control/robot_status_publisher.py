#!/usr/bin/env python3
"""
Robot Status Publisher
Aggregates system health and publishes polebot_interfaces/RobotStatus
AMR-POLEBOT | Polman Bandung
"""
import rclpy
from rclpy.node import Node
from polebot_interfaces.msg import RobotStatus, MotorStatus
from sensor_msgs.msg import BatteryState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
import math


class RobotStatusPublisher(Node):
    def __init__(self):
        super().__init__('robot_status_publisher')

        # Publishers
        self.status_pub = self.create_publisher(RobotStatus, '/polebot/status', 10)

        # Subscriptions
        self.create_subscription(BatteryState, '/battery_state', self.battery_cb, 10)
        self.create_subscription(MotorStatus,  '/motor_status',  self.motor_cb,  10)
        self.create_subscription(Odometry,     '/odom',          self.odom_cb,   10)

        # State
        self.battery_pct = 100.0
        self.battery_v   = 0.0
        self.is_charging = False
        self.motor_ok    = True
        self.pose_2d     = Pose2D()
        self.lidar_ok    = True
        self.imu_ok      = True
        self.camera_ok   = True

        # Timer — publish at 2 Hz
        self.create_timer(0.5, self.publish_status)

        self.get_logger().info('RobotStatusPublisher started')

    def battery_cb(self, msg: BatteryState):
        self.battery_v   = msg.voltage
        self.battery_pct = msg.percentage * 100.0
        self.is_charging = (msg.power_supply_status ==
                            BatteryState.POWER_SUPPLY_STATUS_CHARGING)

    def motor_cb(self, msg: MotorStatus):
        self.motor_ok = not (msg.left_fault or msg.right_fault)

    def odom_cb(self, msg: Odometry):
        self.pose_2d.x = msg.pose.pose.position.x
        self.pose_2d.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.pose_2d.theta = math.atan2(siny, cosy)

    def publish_status(self):
        msg = RobotStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.mode             = RobotStatus.MODE_MANUAL
        msg.battery_voltage  = self.battery_v
        msg.battery_percentage = self.battery_pct
        msg.is_charging      = self.is_charging
        msg.pose_2d          = self.pose_2d
        msg.lidar_ok         = self.lidar_ok
        msg.imu_ok           = self.imu_ok
        msg.motor_ok         = self.motor_ok
        msg.camera_ok        = self.camera_ok

        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotStatusPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
