#!/usr/bin/env python3
"""Simulator Kinematis Mandiri untuk Polebot AMR + Trailer di RViz.

Node ini mensimulasikan pergerakan kinematis traktor AMR roda diferensial serta
gandengan trailer (Fixed Joint maupun Pivot Joint) secara mandiri tanpa memerlukan
Gazebo physics engine.

Fitur:
1. Menerima /cmd_vel (Twist) dari pengontrol PID atau teleop keyboard.
2. Menerima /joint_group_effort_controller/commands (Float64MultiArray) dari pengontrol SMC.
3. Mendukung auto_follow: otomatis melacak /planned_path secara mulus jika diaktifkan.
4. Mendukung reposisi interaktif via RViz '2D Pose Estimate' (/initialpose).
5. Menerbitkan TF (odom -> base_link, map -> odom), /odom, dan /joint_states
   (termasuk posisi sudut sendi revolute trolley_hitch_joint untuk artikulasi visual trailer).
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, TransformStamped, Point
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Float64MultiArray, Int32
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros


class KinematicSimulatorNode(Node):
    def __init__(self):
        super().__init__('kinematic_simulator')

        # Parameter inisialisasi
        self.declare_parameter('spawn_x', -7.0)
        self.declare_parameter('spawn_y', -5.5)
        self.declare_parameter('spawn_yaw', 0.0)
        self.declare_parameter('default_mode', 2)       # 0: Solo, 1: Fixed, 2: Pivot
        self.declare_parameter('auto_follow', False)    # Direct kinematic follower on /planned_path
        self.declare_parameter('target_speed', 0.35)    # m/s saat auto_follow
        self.declare_parameter('control_rate', 50.0)    # Hz

        # Dimensi fisik (sesuai SDF)
        self.declare_parameter('d_hitch', 0.786)        # base_link ke hitch pin (m)
        self.declare_parameter('L_trailer', 1.145)      # hitch pin ke as roda trailer (m)
        self.declare_parameter('L_rear', 1.300)         # hitch pin ke bumper belakang trailer (m)
        self.declare_parameter('wheel_radius', 0.079)   # m
        self.declare_parameter('wheel_base', 0.600)     # m
        self.declare_parameter('m_eff', 117.09)         # kg (inersia efektif translasi)
        self.declare_parameter('i_eff', 15.40)          # kg*m^2 (inersia efektif rotasi)

        # Baca parameter
        self._x = float(self.get_parameter('spawn_x').value)
        self._y = float(self.get_parameter('spawn_y').value)
        self._theta = float(self.get_parameter('spawn_yaw').value)
        self._mode = int(self.get_parameter('default_mode').value)
        self._auto_follow = bool(self.get_parameter('auto_follow').value)
        self._target_speed = float(self.get_parameter('target_speed').value)
        self._rate = float(self.get_parameter('control_rate').value)

        self._d_hitch = float(self.get_parameter('d_hitch').value)
        self._L_trailer = float(self.get_parameter('L_trailer').value)
        self._L_rear = float(self.get_parameter('L_rear').value)
        self._r_wheel = float(self.get_parameter('wheel_radius').value)
        self._d_track = float(self.get_parameter('wheel_base').value) / 2.0
        self._m_eff = float(self.get_parameter('m_eff').value)
        self._i_eff = float(self.get_parameter('i_eff').value)

        # Kecepatan traktor
        self._v = 0.0
        self._omega = 0.0

        # Keadaan trailer
        self._theta_t = self._theta
        self._gamma = 0.0  # Sudut artikulasi hitch (theta - theta_t)

        # Posisi roda (rotasi)
        self._wheel_l_pos = 0.0
        self._wheel_r_pos = 0.0
        self._trailer_wl_pos = 0.0
        self._trailer_wr_pos = 0.0

        # Status perintah
        self._last_cmd_time = 0.0
        self._cmd_type = 'none'  # 'vel', 'effort', 'auto'
        self._cmd_v = 0.0
        self._cmd_omega = 0.0
        self._tau_l = 0.0
        self._tau_r = 0.0

        # Path tracking
        self._path_points = []
        self._path_idx = 0

        # TF Broadcaster
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self._publish_static_transforms()

        # Publishers
        self._pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self._pub_joints = self.create_publisher(JointState, '/joint_states', 10)
        self._pub_actual = self.create_publisher(Path, '/actual_path', 10)
        self._pub_scan = self.create_publisher(LaserScan, '/scan', 10)
        self._pub_markers = self.create_publisher(MarkerArray, '/kinematic_sim/footprint', 10)

        # Subscriptions
        self._sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self._cb_cmd_vel, 10)
        self._sub_effort = self.create_subscription(
            Float64MultiArray, '/joint_group_effort_controller/commands', self._cb_effort, 10)
        self._sub_init_pose = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self._cb_initial_pose, 10)
        self._sub_mode = self.create_subscription(Int32, '/towing_mode', self._cb_mode, 10)
        self._sub_path = self.create_subscription(Path, '/planned_path', self._cb_path, 10)

        # Jejak lintasan aktual untuk RViz
        self._actual_path_msg = Path()
        self._actual_path_msg.header.frame_id = 'map'
        self._last_actual_pub_time = 0.0

        # Main simulation timer (50 Hz)
        self._dt = 1.0 / self._rate
        self._sim_timer = self.create_timer(self._dt, self._update_simulation)

        mode_name = {0: 'Solo', 1: 'Fixed Joint', 2: 'Pivot Joint'}.get(self._mode, 'Unknown')
        self.get_logger().info(
            f'Kinematic Simulator aktif di RViz: Pose=({self._x:.2f}, {self._y:.2f}, {math.degrees(self._theta):.1f}°), '
            f'Mode={mode_name}, AutoFollow={self._auto_follow}'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_cmd_vel(self, msg: Twist):
        self._cmd_v = msg.linear.x
        self._cmd_omega = msg.angular.z
        self._cmd_type = 'vel'
        self._last_cmd_time = self._clock_seconds()

    def _cb_effort(self, msg: Float64MultiArray):
        if len(msg.data) >= 2:
            self._tau_l = msg.data[0]
            self._tau_r = msg.data[1]
            self._cmd_type = 'effort'
            self._last_cmd_time = self._clock_seconds()

    def _cb_initial_pose(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self._x = p.x
        self._y = p.y
        self._theta = yaw
        self._theta_t = yaw
        self._gamma = 0.0
        self._v = 0.0
        self._omega = 0.0
        self._actual_path_msg.poses.clear()
        self.get_logger().info(f'Robot reset ke pose: ({self._x:.2f}, {self._y:.2f}, {math.degrees(yaw):.1f}°)')

    def _cb_mode(self, msg: Int32):
        if msg.data != self._mode:
            self._mode = msg.data
            mode_name = {0: 'Solo', 1: 'Fixed Joint', 2: 'Pivot Joint'}.get(self._mode, '?')
            self.get_logger().info(f'Towing mode diubah ke: {mode_name} ({self._mode})')

    def _cb_path(self, msg: Path):
        self._path_points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._path_idx = 0
        if len(self._path_points) > 1:
            self.get_logger().info(f'Menerima lintasan baru: {len(self._path_points)} titik')

    # ── Simulation Update (50 Hz) ─────────────────────────────────────────────

    def _clock_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _update_simulation(self):
        now_sec = self._clock_seconds()
        dt = self._dt

        # 1. Tentukan sumber kendali kecepatan (v, omega)
        is_cmd_recent = (now_sec - self._last_cmd_time) < 0.25

        if is_cmd_recent and self._cmd_type == 'vel':
            # Kontrol kecepatan langsung (dari PID atau teleop)
            self._v = self._cmd_v
            self._omega = self._cmd_omega

        elif is_cmd_recent and self._cmd_type == 'effort':
            # Integrasi dinamika torsi SMC:
            # v_dot = (tau_l + tau_r) / (R * M_eff)
            # omega_dot = (tau_r - tau_l) * d / (R * I_eff)
            v_dot = (self._tau_l + self._tau_r) / (self._r_wheel * self._m_eff)
            omega_dot = ((self._tau_r - self._tau_l) * self._d_track) / (self._r_wheel * self._i_eff)

            # Redaman gesekan mekanis
            v_dot -= 0.5 * self._v
            omega_dot -= 1.0 * self._omega

            self._v += v_dot * dt
            self._omega += omega_dot * dt

            # Forward-only guard untuk AMR ber-trailer
            if self._mode != 0:
                self._v = max(0.0, self._v)

        elif self._auto_follow or (not is_cmd_recent and len(self._path_points) > 1):
            # Built-in pure pursuit path follower (kinematik mulus)
            self._execute_auto_follow()

        else:
            # Tidak ada perintah: deselerasi hingga berhenti
            self._v *= 0.85
            self._omega *= 0.85
            if abs(self._v) < 0.01:
                self._v = 0.0
            if abs(self._omega) < 0.01:
                self._omega = 0.0

        # 2. Integrasi kinematika traktor AMR
        self._theta += self._omega * dt
        self._theta = math.atan2(math.sin(self._theta), math.cos(self._theta))
        self._x += self._v * math.cos(self._theta) * dt
        self._y += self._v * math.sin(self._theta) * dt

        # 3. Integrasi kinematika trailer
        if self._mode == 0:
            # Solo: tidak ada trailer
            self._theta_t = self._theta
            self._gamma = 0.0
        elif self._mode == 1:
            # Fixed Joint: trailer kaku bersatu dengan bodi AMR
            self._theta_t = self._theta
            self._gamma = 0.0
        else:
            # Pivot Joint: gandengan artikulasi pasif 1-trailer
            # Kecepatan titik pin pengait hitch:
            vh_x = self._v * math.cos(self._theta) + self._d_hitch * self._omega * math.sin(self._theta)
            vh_y = self._v * math.sin(self._theta) - self._d_hitch * self._omega * math.cos(self._theta)
            vh_mag = math.hypot(vh_x, vh_y)
            phi_h = math.atan2(vh_y, vh_x)

            if vh_mag > 1e-4:
                # Dinamika sudut trailer pasif
                d_theta_t = (vh_mag / self._L_trailer) * math.sin(phi_h - self._theta_t) * dt
                self._theta_t += d_theta_t
                self._theta_t = math.atan2(math.sin(self._theta_t), math.cos(self._theta_t))

            # Sudut artikulasi hitch (gamma = theta_robot - theta_trailer)
            self._gamma = math.atan2(math.sin(self._theta - self._theta_t), math.cos(self._theta - self._theta_t))
            # Batas mekanis sendi SDF (+/- 90 deg / 1.57 rad)
            self._gamma = max(-1.57, min(1.57, self._gamma))

        # 4. Integrasi posisi roda (efek putaran roda di RViz)
        v_l = self._v - self._omega * self._d_track
        v_r = self._v + self._omega * self._d_track
        self._wheel_l_pos += (v_l / self._r_wheel) * dt
        self._wheel_r_pos += (v_r / self._r_wheel) * dt
        self._trailer_wl_pos += (self._v / self._r_wheel) * dt
        self._trailer_wr_pos += (self._v / self._r_wheel) * dt

        # 5. Publikasikan data simulator
        stamp = self.get_clock().now().to_msg()
        self._publish_tf(stamp)
        self._publish_odometry(stamp)
        self._publish_joint_states(stamp)
        self._publish_laser_scan(stamp)
        self._publish_actual_path(stamp, now_sec)
        self._publish_footprint_markers(stamp)

    def _execute_auto_follow(self):
        """Pelacak jalur murni kinematik (pure-pursuit) bawaan simulator."""
        if not self._path_points or self._path_idx >= len(self._path_points):
            self._v = 0.0
            self._omega = 0.0
            return

        lookahead = 0.50  # m
        # Cari titik lookahead terdekat pada sisa jalur
        target = self._path_points[-1]
        for i in range(self._path_idx, len(self._path_points)):
            dist = math.hypot(self._path_points[i][0] - self._x, self._path_points[i][1] - self._y)
            if dist >= lookahead:
                target = self._path_points[i]
                self._path_idx = max(self._path_idx, i - 1)
                break

        dist_to_goal = math.hypot(self._path_points[-1][0] - self._x, self._path_points[-1][1] - self._y)
        if dist_to_goal < 0.15:
            # Sampai di tujuan
            self._v = 0.0
            self._omega = 0.0
            return

        dx = target[0] - self._x
        dy = target[1] - self._y
        alpha = math.atan2(math.sin(math.atan2(dy, dx) - self._theta), math.cos(math.atan2(dy, dx) - self._theta))

        # Kontrol proporsional kecepatan belok
        kp_omega = 2.0
        self._v = self._target_speed if abs(alpha) < math.radians(45) else self._target_speed * 0.5
        self._omega = max(-1.0, min(1.0, kp_omega * alpha))

    # ── Publishers ────────────────────────────────────────────────────────────

    def _publish_static_transforms(self):
        """Menerbitkan transform statis map -> odom."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self._tf_static_broadcaster.sendTransform(t)

    def _publish_tf(self, stamp):
        """Menerbitkan transform dinamis odom -> base_link."""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.translation.z = 0.0

        qz = math.sin(self._theta / 2.0)
        qw = math.cos(self._theta / 2.0)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self._tf_broadcaster.sendTransform(t)

    def _publish_odometry(self, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(self._theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(self._theta / 2.0)

        msg.twist.twist.linear.x = self._v
        msg.twist.twist.angular.z = self._omega

        self._pub_odom.publish(msg)

    def _publish_joint_states(self, stamp):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = [
            'drivewhl_l_joint', 'drivewhl_r_joint',
            'casterwhl_fl_joint', 'casterwhl_fr_joint',
            'casterwhl_rl_joint', 'casterwhl_rr_joint',
            'trolley_hitch_joint',
            'trolley_wheel_l_joint', 'trolley_wheel_r_joint',
            'trolley_caster_l_joint', 'trolley_caster_r_joint',
        ]
        msg.position = [
            self._wheel_l_pos, self._wheel_r_pos,
            0.0, 0.0,
            0.0, 0.0,
            self._gamma,  # Sudut artikulasi putar trailer di RViz!
            self._trailer_wl_pos, self._trailer_wr_pos,
            0.0, 0.0,
        ]
        msg.velocity = [
            (self._v - self._omega * self._d_track) / self._r_wheel,
            (self._v + self._omega * self._d_track) / self._r_wheel,
            0.0, 0.0,
            0.0, 0.0,
            (self._omega if self._mode == 2 else 0.0),
            self._v / self._r_wheel,
            self._v / self._r_wheel,
            0.0, 0.0,
        ]
        self._pub_joints.publish(msg)

    def _publish_laser_scan(self, stamp):
        """Menerbitkan LaserScan sintetis kosong agar pengontrol SMC tidak terblokir."""
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'lidar_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.radians(1.0)
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 20.0
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        scan.ranges = [20.0] * num_readings
        self._pub_scan.publish(scan)

    def _publish_actual_path(self, stamp, now_sec):
        """Menerbitkan jejak lintasan aktual berwarna merah di RViz."""
        if now_sec - self._last_actual_pub_time > 0.10:
            self._last_actual_pub_time = now_sec
            p = PoseStamped()
            p.header.stamp = stamp
            p.header.frame_id = 'map'
            p.pose.position.x = self._x
            p.pose.position.y = self._y
            p.pose.position.z = 0.02
            p.pose.orientation.z = math.sin(self._theta / 2.0)
            p.pose.orientation.w = math.cos(self._theta / 2.0)
            self._actual_path_msg.poses.append(p)
            if len(self._actual_path_msg.poses) > 2000:
                self._actual_path_msg.poses.pop(0)
            self._pub_actual.publish(self._actual_path_msg)

    def _publish_footprint_markers(self, stamp):
        """Visualisasi amplop 3D AMR dan trailer di RViz."""
        ma = MarkerArray()

        # 1. Bounding box AMR traktor
        m_amr = Marker()
        m_amr.header.stamp = stamp
        m_amr.header.frame_id = 'base_link'
        m_amr.ns = 'amr_footprint'
        m_amr.id = 0
        m_amr.type = Marker.CUBE
        m_amr.action = Marker.ADD
        m_amr.pose.position.z = 0.20
        m_amr.scale.x = 1.172
        m_amr.scale.y = 0.670
        m_amr.scale.z = 0.350
        m_amr.color.r = 0.1
        m_amr.color.g = 0.5
        m_amr.color.b = 0.9
        m_amr.color.a = 0.4
        ma.markers.append(m_amr)

        # 2. Bounding box Trailer (jika ada trailer)
        if self._mode != 0:
            m_tr = Marker()
            m_tr.header.stamp = stamp
            m_tr.header.frame_id = 'map'
            m_tr.ns = 'trailer_footprint'
            m_tr.id = 1
            m_tr.type = Marker.CUBE
            m_tr.action = Marker.ADD

            # Hitung posisi pusat bodi trailer di peta
            hx = self._x - self._d_hitch * math.cos(self._theta)
            hy = self._y - self._d_hitch * math.sin(self._theta)
            cx = hx - (self._L_rear / 2.0) * math.cos(self._theta_t)
            cy = hy - (self._L_rear / 2.0) * math.sin(self._theta_t)

            m_tr.pose.position.x = cx
            m_tr.pose.position.y = cy
            m_tr.pose.position.z = 0.20
            m_tr.pose.orientation.z = math.sin(self._theta_t / 2.0)
            m_tr.pose.orientation.w = math.cos(self._theta_t / 2.0)

            m_tr.scale.x = self._L_rear
            m_tr.scale.y = 1.000
            m_tr.scale.z = 0.300
            m_tr.color.r = 0.9
            m_tr.color.g = 0.4
            m_tr.color.b = 0.1
            m_tr.color.a = 0.45
            ma.markers.append(m_tr)

        self._pub_markers.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicSimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
