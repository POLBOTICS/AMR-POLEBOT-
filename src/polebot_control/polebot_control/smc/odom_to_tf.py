import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, '/odom', self.callback, 10)

    def callback(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        # Elevasi base_link agar roda menapak pas di atas lantai (wheel_radius 0.079 + wheel_zoff 0.231 = 0.310m)
        t.transform.translation.z = msg.pose.pose.position.z if abs(msg.pose.pose.position.z) > 0.01 else 0.310
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = OdomToTF()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
