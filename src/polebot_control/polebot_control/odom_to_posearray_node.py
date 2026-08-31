#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry

class OdomToPoseArrayNode(Node):
    """
    Konverter Odometry ke PoseArray (Untuk Otak/Kontroler)

    TF odom->base_link TIDAK di-broadcast di sini lagi. Plugin
    gz-sim-diff-drive-system di Gazebo sudah publish TF itu sendiri
    lewat topic native "/tf" (dijembatani oleh ros_gz_bridge). Kalau
    node ini ikut broadcast TF yang sama, akan terjadi dua publisher
    untuk transform yang sama -> bentrok/jitter di RViz (TF_REPEATED_DATA).
    """
    def __init__(self):
        super().__init__("odom_to_posearray_node")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("posearray_topic", "/model/ddr/pose")
        self.declare_parameter("frame_id", "odom")

        odom_topic = self.get_parameter("odom_topic").value
        posearray_topic = self.get_parameter("posearray_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        self.publisher = self.create_publisher(PoseArray, posearray_topic, 10)
        self.subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10,
        )

        self.get_logger().info(f"Odom to PoseArray Started!")

    def odom_callback(self, odom_msg: Odometry):
        pose_array = PoseArray()
        pose_array.header.stamp = odom_msg.header.stamp
        pose_array.header.frame_id = self.frame_id if self.frame_id else odom_msg.header.frame_id
        pose_array.poses.append(odom_msg.pose.pose)
        self.publisher.publish(pose_array)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToPoseArrayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()