import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

# Physical constants — must match polebot_amr_description.sdf
WHEEL_RADIUS = 0.079   # m
WHEEL_BASE   = 0.600   # m  (|2 × wheel_yoff|)

# Naming convention: drivewhl_l sits at y = +0.300 (left side) and drivewhl_r sits at y = -0.300 (right side).
LEFT_JOINT  = 'drivewhl_l_joint'
RIGHT_JOINT = 'drivewhl_r_joint'


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class WheelOdomPublisher(Node):
    """Compute and publish /odom with support for Gazebo Ground-Truth and encoder fallback.

    When running in Gazebo, subscribes to the true physics pose (/model/polebot_amr/odometry)
    and relativizes it against the spawn pose. This eliminates tire slip, drift, and understeer
    in localization, ensuring RViz displays the exact physical pose of the robot in Gazebo.
    If ground-truth is not available, falls back to joint_states encoder integration.
    """

    def __init__(self):
        super().__init__('wheel_odom_publisher')

        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        self.declare_parameter('spawn_yaw', 0.0)
        self.declare_parameter('gazebo_odom_topic', '/model/polebot_amr/odometry')

        self.declare_parameter('use_ground_truth', False)
        self._use_ground_truth = bool(self.get_parameter('use_ground_truth').value)

        self._spawn_x = float(self.get_parameter('spawn_x').value)
        self._spawn_y = float(self.get_parameter('spawn_y').value)
        self._spawn_yaw = float(self.get_parameter('spawn_yaw').value)
        gz_topic = self.get_parameter('gazebo_odom_topic').value

        self._pub = self.create_publisher(Odometry, '/odom', 10)

        # Fallback encoder subscription
        self._sub_joints = self.create_subscription(
            JointState, '/joint_states', self._cb_joints, qos_profile_sensor_data)

        # Gazebo Ground-Truth subscription
        self._sub_gz_odom = self.create_subscription(
            Odometry, gz_topic, self._cb_gz_odom, qos_profile_sensor_data)

        self._use_gz_gt = False
        self._last_gz_stamp: rclpy.time.Time | None = None

        self._x     = 0.0
        self._y     = 0.0
        self._theta = 0.0
        self._last_stamp: rclpy.time.Time | None = None

        self.get_logger().info(
            f'WheelOdomPublisher started (use_ground_truth={self._use_ground_truth}, '
            f'spawn=[{self._spawn_x:.2f}, {self._spawn_y:.2f}, {self._spawn_yaw:.2f}], '
            f'gt_topic={gz_topic})'
        )

    def _cb_gz_odom(self, msg: Odometry):
        """Handle Gazebo Ground-Truth Odometry (hanya aktif jika use_ground_truth=True)."""
        if not self._use_ground_truth:
            return

        self._use_gz_gt = True
        self._last_gz_stamp = self.get_clock().now()

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        dx = p.x - self._spawn_x
        dy = p.y - self._spawn_y
        c = math.cos(self._spawn_yaw)
        s = math.sin(self._spawn_yaw)

        # Rotated to local odom frame
        self._x = c * dx + s * dy
        self._y = -s * dx + c * dy
        self._theta = _wrap(yaw - self._spawn_yaw)

        odom = Odometry()
        odom.header.stamp = msg.header.stamp if msg.header.stamp.sec > 0 else self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = p.z if abs(p.z) > 0.01 else 0.310
        odom.pose.pose.orientation.z = math.sin(self._theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._theta / 2.0)

        # True velocity from physics engine
        odom.twist = msg.twist

        self._pub.publish(odom)

    def _cb_joints(self, msg: JointState):
        """Hitung odometri roda dari kecepatan enkoder sendi roda (fisika diff-drive riil)."""
        if self._use_ground_truth and self._use_gz_gt and self._last_gz_stamp is not None:
            now = self.get_clock().now()
            dt_since_gt = (now - self._last_gz_stamp).nanoseconds / 1e9
            if dt_since_gt < 0.5:
                return

        if LEFT_JOINT not in msg.name or RIGHT_JOINT not in msg.name:
            return
        if not msg.velocity:
            return

        il = msg.name.index(LEFT_JOINT)
        ir = msg.name.index(RIGHT_JOINT)

        now = self.get_clock().now()
        if self._last_stamp is None:
            self._last_stamp = now
            return

        dt = (now - self._last_stamp).nanoseconds / 1e9
        self._last_stamp = now

        if dt <= 0.0 or dt > 0.5:
            return

        omega_l = msg.velocity[il]
        omega_r = msg.velocity[ir]

        # Kinematika — standard diff-drive (drivewhl_l = left, drivewhl_r = right)
        v     = WHEEL_RADIUS / 2.0 * (omega_l + omega_r)
        omega = WHEEL_RADIUS / WHEEL_BASE * (omega_r - omega_l)

        # Integrasi posisi pada frame odom lokal
        self._x     += v * math.cos(self._theta) * dt
        self._y     += v * math.sin(self._theta) * dt
        self._theta += omega * dt
        self._theta  = _wrap(self._theta)

        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'

        odom.pose.pose.position.x    = self._x
        odom.pose.pose.position.y    = self._y
        odom.pose.pose.position.z    = 0.310  # Tinggi base_link agar roda menapak pas di atas lantai
        odom.pose.pose.orientation.z = math.sin(self._theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._theta / 2.0)

        # Matriks kovariansi diagonal untuk fusi sensor AMCL / SLAM / EKF
        odom.pose.covariance[0]  = 0.001
        odom.pose.covariance[7]  = 0.001
        odom.pose.covariance[14] = 0.001
        odom.pose.covariance[35] = 0.001

        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = omega
        odom.twist.covariance[0]   = 0.001
        odom.twist.covariance[35]  = 0.001

        self._pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
