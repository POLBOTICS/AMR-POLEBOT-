import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Int32, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PolygonStamped, Point32

SOLO, FIXED, PIVOT = 0, 1, 2

# Footprint polygon per mode, derived from SDF dimensions (CLAUDE.md §4.5)
FOOTPRINTS = {
    SOLO:  [(+0.586, +0.335), (+0.586, -0.335), (-0.586, -0.335), (-0.586, +0.335)],
    FIXED: [(+0.586, +0.500), (+0.586, -0.500), (-2.180, -0.500), (-2.180, +0.500)],
    PIVOT: [(+0.586, +0.550), (+0.586, -0.550), (-2.180, -0.550), (-2.180, +0.550)],
}

MODE_NAMES = {SOLO: 'Solo', FIXED: 'Fixed Joint', PIVOT: 'Pivot Joint'}
HYSTERESIS_COUNT = 3


class TrajModeSelectorNode(Node):
    def __init__(self):
        super().__init__('trajectory_mode_selector')
        self.declare_parameter('joint_type', 'auto')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )

        self._sub_joints = self.create_subscription(
            JointState, '/joint_states', self._cb_joints, sensor_qos)
        self._sub_dock = self.create_subscription(
            String, '/dock_status', self._cb_dock, 10)

        self._pub_mode = self.create_publisher(Int32, '/towing_mode', 10)
        self._pub_status = self.create_publisher(String, '/towing_status', 10)
        self._pub_footprint = self.create_publisher(
            PolygonStamped, '/local_costmap/footprint', 10)

        init_mode = self._compute_candidate()
        self._docked = False
        self._has_hitch = False
        self._current_mode = init_mode
        self._candidate = init_mode
        self._hysteresis_count = HYSTERESIS_COUNT

        self.create_timer(0.1, self._cb_timer)  # 10 Hz
        self._publish_footprint()

    def _cb_dock(self, msg):
        self._docked = msg.data.lower() == 'docked'

    def _cb_joints(self, msg):
        self._has_hitch = 'trolley_hitch_joint' in msg.name

    def _compute_candidate(self):
        jt = self.get_parameter('joint_type').get_parameter_value().string_value
        if jt in ('solo', 'none'):
            return SOLO
        if jt == 'fixed':
            return FIXED
        if jt == 'pivot':
            return PIVOT
        # auto mode: detect from dock_status + joint_states
        if not self._docked:
            return SOLO
        return PIVOT if self._has_hitch else FIXED

    def _cb_timer(self):
        candidate = self._compute_candidate()

        if candidate == self._candidate:
            self._hysteresis_count += 1
        else:
            self._candidate = candidate
            self._hysteresis_count = 1

        if self._hysteresis_count >= HYSTERESIS_COUNT and candidate != self._current_mode:
            self._current_mode = candidate
            self.get_logger().info(f'Mode changed to: {MODE_NAMES[self._current_mode]}')
            self._publish_footprint()

        mode_msg = Int32()
        mode_msg.data = self._current_mode
        self._pub_mode.publish(mode_msg)

        status_msg = String()
        status_msg.data = f'{MODE_NAMES[self._current_mode]} Mode'
        self._pub_status.publish(status_msg)

    def _publish_footprint(self):
        msg = PolygonStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        for x, y in FOOTPRINTS[self._current_mode]:
            pt = Point32()
            pt.x = float(x)
            pt.y = float(y)
            msg.polygon.points.append(pt)
        self._pub_footprint.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrajModeSelectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
