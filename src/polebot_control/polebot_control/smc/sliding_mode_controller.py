"""Sliding Mode Controller — Dynamic formulation (Alipour et al. 2019).

Control hierarchy
-----------------
  Outer layer : SMC on polar errors (rho, phi) → desired accelerations (v_dot_d, omega_dot_d)
  Inner layer : Differential-drive dynamic inversion → wheel torques (tau_l, tau_r)
  Actuator    : gz_ros2_control JointGroupEffortController
                topic: /joint_group_effort_controller/commands  [Float64MultiArray]
                order: [drivewhl_l_joint, drivewhl_r_joint]

Notation follows Alipour et al., Mech. Mach. Theory 138 (2019) 16–37.
  rho  = distance from robot CoM to lookahead point
  phi  = heading error  = atan2(dy,dx) − theta_robot   (analog of E_phi, Eq. 62)
  S1   = rho_dot + lambda1 * rho   (analog of Eq. 65)
  S2   = phi_dot + lambda2 * phi   (analog of Eq. 65)
  u_d  = u_eq + u_sw  with  u_sw = K * sat(S/phi_bound)   (analog of Eq. 70)
  tau  = M_tilde * [v_dot_d, omega_dot_d]^T + V_tilde     (Eq. 33 / 40 analog)

Physical parameters are derived from polebot_amr_description.sdf (CLAUDE.md §4).
"""

import math

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Float64MultiArray, Int32, String

# ── Physical constants from SDF ───────────────────────────────────────────────
WHEEL_RADIUS = 0.079    # m — wheel_radius (SDF line 21)
WHEEL_BASE   = 0.600    # m — |2 × wheel_yoff|, SDF line 24
HITCH_LENGTH = 0.586    # m — base_length/2, SDF line 10
TRAILER_LEN  = 1.145    # m — trolley_half_length + wheel_xoff

_d = WHEEL_BASE / 2.0   # m — half wheel separation = 0.300

# ── Dynamic model parameters (derived from SDF mass/inertia) ─────────────────
# Component masses (kg)
_M_BASE    = 84.837   # base_footprint
_M_MONITOR = 16.008
_M_WHEEL   =  3.610   # each drive wheel
_M_CASTER  =  1.355   # each (×4)
M_TOTAL    = _M_BASE + _M_MONITOR + 2 * _M_WHEEL + 4 * _M_CASTER   # 113.485 kg

# Drive wheel spin-axis inertia: cylinder Izz = m*r²/2
_I_WHEEL = _M_WHEEL * WHEEL_RADIUS ** 2 / 2.0    # 0.01127 kg·m²

# Robot body moment of inertia about vertical axis (approximate, from SDF box_inertia):
#   I_z_base ≈ m_base/12 * (w² + l²) = 84.837/12*(0.670²+1.172²) ≈ 12.88 kg·m²
# Plus monitor, caster, and wheel-body contributions:
_I_Z = 15.40    # kg·m²  (see implementation notes above)

# Effective inertia parameters for (v, omega) dynamics (diff-drive Lagrangian):
#   m_eff * v_dot   = (tau_l + tau_r) / R
#   I_eff * omega_dot = (tau_l - tau_r) * d / R
M_EFF = M_TOTAL + 2.0 * _I_WHEEL / WHEEL_RADIUS ** 2    # ≈ 117.09 kg
I_EFF = _I_Z   + 2.0 * _I_WHEEL * (_d / WHEEL_RADIUS) ** 2  # ≈ 15.73 kg·m²

# Effective rotational inertia accounting for trailer mass & moment arm per mode
I_EFF_MAP = {
    0: 28.0,     # Solo: diadaptasi dari Trolley Pivot (28.0) agar torsi rotasi (yaw authority) kuat membelokkan robot
    1: 30.0,     # Fixed tow: robot + rigid trailer inertia
    2: 28.0,     # Pivot tow: robot + articulated trailer hitch yaw resistance
}

TAU_MAX = 40.0    # N·m per wheel — safe within actuator limit (50.0 N·m in SDF)

# ── SMC gain sets per towing mode ────────────────────────────────────────────
# Units:  lambda1 [s⁻¹], lambda2 [s⁻¹]
#         K1 [m/s²], K2 [rad/s²]   (multiply sat()→dimensionless → desired accel)
#         phi1 [m/s], phi2 [rad/s] (boundary-layer width, same units as S1/S2)
#         v_max [m/s], omega_max [rad/s]
SMC_PARAMS = {
    0: {  # Solo — diadaptasi penuh dari konfigurasi Trolley Pivot yang sukses belok
        'lambda1': 0.5, 'lambda2': 1.5,
        'K1': 2.0,  'K2': 10.0,
        'phi1': 0.35, 'phi2': 0.25,
        'v_max': 0.30, 'omega_max': 0.60,
    },
    1: {  # Fixed tow — rigid hitch
        'lambda1': 0.5, 'lambda2': 1.0,
        'K1': 2.0,  'K2': 8.0,
        'phi1': 0.35, 'phi2': 0.3,
        'v_max': 0.30, 'omega_max': 0.50,
    },
    2: {  # Pivot tow — rotating hitch
        'lambda1': 0.5, 'lambda2': 1.5,
        'K1': 2.0,  'K2': 10.0,
        'phi1': 0.35, 'phi2': 0.25,
        'v_max': 0.30, 'omega_max': 0.50,
    },
}

LOOKAHEAD_DIST  = 0.50    # m — responsive tracking around curves
GOAL_TOLERANCE  = 0.15    # m — kompromi: lebih akurat tapi berhenti sebelum
                          # zona singular atan2(phi) yg dominan di bawah ~0.10m
RHO_MIN         = 0.05    # m — minimum rho before phi_dot blows up
COS_PHI_MIN     = 0.707   # cos(45°) — below this: switch to alignment mode to avoid driving sideways into obstacles
OBS_STOP_DIST   = 0.30    # m
OBS_SLOW_DIST   = 0.50    # m
JACKKNIFE_LIMIT = math.radians(55)  # 55 deg (physical joint limit in SDF is 90 deg / 1.57 rad)
JACKKNIFE_WARN  = math.radians(40)  # 40 deg — soft limit threshold
CONTROL_FREQ    = 50.0    # Hz

LEFT_JOINT  = 'drivewhl_l_joint'
RIGHT_JOINT = 'drivewhl_r_joint'

MODE_NAMES = {0: 'Solo', 1: 'Fixed', 2: 'Pivot'}


def _normalize(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _sat(x: float) -> float:
    return max(-1.0, min(1.0, x))


class SlidingModeControllerNode(Node):
    def __init__(self):
        super().__init__('sliding_mode_controller')

        self._sub_path   = self.create_subscription(Path, '/planned_path', self._cb_path, 10)
        self._sub_joints = self.create_subscription(
            JointState, '/joint_states', self._cb_joints, qos_profile_sensor_data)
        self._sub_mode   = self.create_subscription(Int32, '/towing_mode', self._cb_mode, 10)
        self._sub_scan   = self.create_subscription(
            LaserScan, '/scan', self._cb_scan, qos_profile_sensor_data)
        self._sub_odom   = self.create_subscription(
            Odometry, '/odom', self._cb_odom, qos_profile_sensor_data)

        # Effort command publisher — order matches ros2_controllers.yaml: [left, right]
        self._pub_effort  = self.create_publisher(
            Float64MultiArray, '/joint_group_effort_controller/commands', 10)
        self._pub_cmdvel  = self.create_publisher(Twist, '/cmd_vel', 10)
        self._pub_debug   = self.create_publisher(String, '/smc_debug', 10)
        self._pub_actual  = self.create_publisher(Path, '/actual_path', 10)

        # TF: get robot pose in map frame (path is in map frame)
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.declare_parameter('default_mode', 2)  # 0=Solo, 1=Fixed, 2=Pivot
        self.declare_parameter('enable_obstacle_stop', False)  # Default: False (agar tracking simulasi di RViz terus berjalan lancar)

        self._path: list[tuple[float, float]] = []
        self._path_idx: int = 0
        self._path_frame: str = 'map'
        self._actual_path = Path()
        self._actual_path.header.frame_id = 'map'
        self._last_actual_pub_time = 0.0
        self._rx = self._ry = self._rtheta = 0.0
        self._v  = self._omega = 0.0         # actual velocities from joint states or odom
        self._mode: int = self.get_parameter('default_mode').value
        self._alpha: float = 0.0             # hitch articulation (Pivot mode)
        self._scan_ranges: list[float] = []
        self._have_pose: bool = False
        self._have_joints: bool = False
        self._have_odom: bool = False
        self._finished: bool = False
        self._dt: float = 1.0 / CONTROL_FREQ

        self.create_timer(self._dt, self._control_loop)

    # ── Subscribers ──────────────────────────────────────────────────────────

    def _cb_path(self, msg: Path):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(pts) < 2:
            return

        # Prevent resetting tracking progress if identical path is published periodically
        if len(self._path) == len(pts) and len(self._path) > 0:
            p0_same = math.hypot(pts[0][0] - self._path[0][0], pts[0][1] - self._path[0][1]) < 0.01
            pn_same = math.hypot(pts[-1][0] - self._path[-1][0], pts[-1][1] - self._path[-1][1]) < 0.01
            if p0_same and pn_same:
                return

        self._path     = pts
        self._path_idx = 0
        self._finished = False
        self._path_frame = msg.header.frame_id if msg.header.frame_id else 'map'
        self._actual_path = Path()
        self._actual_path.header.frame_id = self._path_frame
        self.get_logger().info(f'New path: {len(self._path)} pts (frame={self._path_frame})')

    def _cb_odom(self, msg: Odometry):
        self._v = msg.twist.twist.linear.x
        self._omega = msg.twist.twist.angular.z
        self._have_odom = True

    def _cb_joints(self, msg: JointState):
        if LEFT_JOINT not in msg.name or RIGHT_JOINT not in msg.name:
            return
        if not msg.velocity:
            return
        il = msg.name.index(LEFT_JOINT)
        ir = msg.name.index(RIGHT_JOINT)
        omega_l = msg.velocity[il]
        omega_r = msg.velocity[ir]
        # Only use wheel kinematics if odom twist is not active
        if not self._have_odom:
            self._v     = WHEEL_RADIUS / 2.0 * (omega_l + omega_r)
            self._omega = WHEEL_RADIUS / WHEEL_BASE * (omega_r - omega_l)
        # Extract hitch articulation angle for Pivot-mode jackknife protection
        if 'trolley_hitch_joint' in msg.name:
            self._alpha = msg.position[msg.name.index('trolley_hitch_joint')]
        self._have_joints = True

    def _cb_mode(self, msg: Int32):
        self._mode = msg.data

    def _cb_scan(self, msg: LaserScan):
        self._scan_ranges = list(msg.ranges)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _get_pose(self) -> bool:
        """Update (rx, ry, rtheta) from TF path_frame→base_link. Returns success."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._path_frame, 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
            self._rx = tf.transform.translation.x
            self._ry = tf.transform.translation.y
            q = tf.transform.rotation
            self._rtheta = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self._have_pose = True
            return True
        except Exception:
            return self._have_pose   # use last good pose if TF temporarily unavailable

    def _obstacle_factor(self) -> tuple[float, bool]:
        if not self._scan_ranges:
            return 1.0, False
        valid = [r for r in self._scan_ranges if math.isfinite(r) and r > 0.0]
        if not valid:
            return 1.0, False
        d_min = min(valid)
        if d_min < OBS_STOP_DIST:
            return 0.0, True
        if d_min < OBS_SLOW_DIST:
            return (d_min - OBS_STOP_DIST) / (OBS_SLOW_DIST - OBS_STOP_DIST), False
        return 1.0, False

    def _find_lookahead(self) -> tuple[tuple[float, float] | None, int]:
        n = len(self._path)
        if n == 0:
            return None, 0

        # Global search on startup, forward window search once moving
        if self._path_idx <= 0:
            search_start = 0
            search_end = min(50, n)
        else:
            search_start = max(0, self._path_idx - 20)
            search_end = min(self._path_idx + 100, n)

        best_idx = self._path_idx
        best_d = math.hypot(self._path[best_idx][0] - self._rx,
                            self._path[best_idx][1] - self._ry)
        for i in range(search_start, search_end):
            d = math.hypot(self._path[i][0] - self._rx, self._path[i][1] - self._ry)
            if d < best_d:
                best_d, best_idx = d, i
        self._path_idx = best_idx

        for i in range(self._path_idx, n):
            if math.hypot(self._path[i][0] - self._rx,
                          self._path[i][1] - self._ry) >= LOOKAHEAD_DIST:
                return self._path[i], i
        return self._path[-1], n - 1

    def _effort_zero(self):
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0]
        self._pub_effort.publish(msg)
        self._pub_cmdvel.publish(Twist())

    # ── Control loop ─────────────────────────────────────────────────────────

    def _control_loop(self):
        if self._finished:
            self._effort_zero()
            return

        if not self._get_pose() or not self._have_joints or not self._path:
            self._effort_zero()
            return

        # Record actual trajectory periodically for RViz visualization
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self._last_actual_pub_time >= 0.1:  # 10 Hz
            self._last_actual_pub_time = now_sec
            ps = PoseStamped()
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.header.frame_id = 'map'
            ps.pose.position.x = self._rx
            ps.pose.position.y = self._ry
            ps.pose.orientation.z = math.sin(self._rtheta / 2.0)
            ps.pose.orientation.w = math.cos(self._rtheta / 2.0)
            self._actual_path.header.stamp = ps.header.stamp
            self._actual_path.poses.append(ps)
            if len(self._actual_path.poses) > 3000:
                self._actual_path.poses.pop(0)
            self._pub_actual.publish(self._actual_path)

        p = SMC_PARAMS.get(self._mode, SMC_PARAMS[0])

        # ── [1] Goal check ────────────────────────────────────────────────────
        goal = self._path[-1]
        dist_goal = math.hypot(goal[0] - self._rx, goal[1] - self._ry)
        at_end = (self._path_idx >= len(self._path) * 0.85) or (len(self._path) - self._path_idx <= 25)
        if at_end and dist_goal < GOAL_TOLERANCE:
            self._finished = True
            self._effort_zero()
            self.get_logger().info(f'Goal reached. dist={dist_goal:.4f} m')
            return

        lookahead, _ = self._find_lookahead()
        if lookahead is None:
            self._effort_zero()
            return

        # ── [2] Polar error model (Alipour 2019 §3.1, Eq. 62 analog) ────────
        # rho  = distance to lookahead
        # phi  = heading error (robot heading vs direction to lookahead)
        dx  = lookahead[0] - self._rx
        dy  = lookahead[1] - self._ry
        rho = max(math.hypot(dx, dy), RHO_MIN)
        phi = _normalize(math.atan2(dy, dx) - self._rtheta)

        # ── [3] Kinematic rates (Eq. 54) ───────────────────────────────────────────────
        # rho_dot  = -v * cos(phi)    (distance decreasing when phi ≈ 0)
        # phi_dot  = v*sin(phi)/rho - omega
        v     = self._v
        omega = self._omega
        rho_dot = -v * math.cos(phi)
        phi_dot =  v * math.sin(phi) / rho - omega

        # ── [4] Sliding surfaces (Alipour 2019 §3.2, Eq. 65 & 72) ────────────────
        # S1 = rho_dot + lambda1 * rho   →  drives rho → 0
        # S2 = phi_dot + lambda2 * phi   →  drives phi → 0
        S1 = rho_dot + p['lambda1'] * rho
        S2 = phi_dot + p['lambda2'] * phi

        sat_S1 = _sat(S1 / p['phi1'])
        sat_S2 = _sat(S2 / p['phi2'])

        # ── [5] Desired accelerations via SMC law (Alipour 2019 §3.2, Eq. 70) ─
        # From dS1/dt = 0 condition:
        #   v_dot_d = [K1*sat(S1/phi1) + v*phi_dot*sin(phi) - lambda1*v*cos(phi)] / cos(phi)
        #
        # From dS2/dt = 0 condition:
        #   omega_dot_d = K2*sat(S2/phi2)
        #               + [v_dot_d*sin(phi) + v*phi_dot*cos(phi)] / rho
        #               + v²*sin(phi)*cos(phi) / rho²
        #               + lambda2*phi_dot
        #
        # Singularity guard: when cos(phi) < COS_PHI_MIN (heading error > ~81 deg),
        # switch to alignment mode — rotate in-place to align heading first.

        cos_phi = math.cos(phi)
        sin_phi = math.sin(phi)

        alignment_mode = cos_phi < COS_PHI_MIN
        if alignment_mode:
            # Focus on aligning heading before driving forward
            v_dot_d     = -v / (0.2 + self._dt)   # decelerate to stop
            omega_dot_d = p['K2'] * sat_S2 + p['lambda2'] * phi_dot
            # Diadaptasi dari Trolley Pivot: pertahankan minimum forward creep saat belokan tajam
            # agar robot bergerak membentuk kurva dan tidak macet berputar di tempat
            if dist_goal > LOOKAHEAD_DIST:
                V_CREEP = 0.08   # m/s — minimum forward speed during alignment
                if v < V_CREEP:
                    v_dot_d = max(v_dot_d, (V_CREEP - v) / (0.3 + self._dt))
        else:
            # Full Alipour dynamic SMC
            feedforward_v = v * phi_dot * sin_phi - p['lambda1'] * v * cos_phi
            v_dot_d = (p['K1'] * sat_S1 + feedforward_v) / cos_phi

            feedforward_omega = ((v_dot_d * sin_phi + v * phi_dot * cos_phi) / rho
                                 + v * v * sin_phi * cos_phi / (rho * rho)
                                 + p['lambda2'] * phi_dot)
            omega_dot_d = p['K2'] * sat_S2 + feedforward_omega

        # ── [6] Acceleration clamping & Velocity limits ───────────────────────
        v_max     = p['v_max']
        omega_max = p['omega_max']

        # Speed scaling in curves: reduce forward speed when heading error phi is large
        # to prevent tire slip and wheel skidding
        speed_scale = max(0.40, cos_phi)
        effective_v_max = v_max * speed_scale

        if v     >= effective_v_max and v_dot_d     > 0.0:
            v_dot_d     = 0.0
        if v     <= 0.0             and v_dot_d     < 0.0:
            v_dot_d     = 0.0
        if abs(omega) >= omega_max  and omega_dot_d * math.copysign(1, omega) > 0:
            omega_dot_d = 0.0

        v_dot_d     = max(-2.0, min(2.0, v_dot_d))
        omega_dot_d = max(-4.0, min(4.0, omega_dot_d))

        # Obstacle avoidance (dinonaktifkan secara default agar simulasi di RViz terus berjalan mengikuti path)
        if self.get_parameter('enable_obstacle_stop').value:
            obs_factor, emergency = self._obstacle_factor()
            if emergency:
                self._effort_zero()
                return
            if obs_factor < 1.0:
                v_target = p['v_max'] * obs_factor
                if v > v_target:
                    v_dot_d = min(v_dot_d, (v_target - v) / (0.2 + self._dt))

        # Jackknife protection (Pivot mode only)
        # NOTE: NEVER reverse to fix a jackknife! Reversing bends the trailer further.
        # To straighten an articulated trailer, pull forward gently and cease turning into the bend.
        if self._mode == 2 and abs(self._alpha) > JACKKNIFE_LIMIT:
            # Stop turning into the bend (only if turning worsens alpha)
            if omega_dot_d * self._alpha > 0.0:
                omega_dot_d = 0.0
            # Pull forward gently to straighten the trailer
            if v < 0.08:
                v_dot_d = max(v_dot_d, (0.08 - v) / (0.3 + self._dt))
        elif self._mode == 2 and abs(self._alpha) > JACKKNIFE_WARN:
            # Soft limit: proportionally reduce omega that worsens alpha
            if omega_dot_d * self._alpha > 0.0:
                factor = 1.0 - (abs(self._alpha) - JACKKNIFE_WARN) / (JACKKNIFE_LIMIT - JACKKNIFE_WARN)
                omega_dot_d *= max(0.0, factor)

        # ── Strict forward-only guard ─────────────────────────────────────────
        # The AMR Polebot with trailer is designed for forward navigation only.
        # Prevent any negative linear acceleration from driving the robot backwards.
        if v <= 0.0 and v_dot_d < 0.0:
            v_dot_d = 0.0
        elif v > 0.0 and v_dot_d < 0.0:
            v_dot_d = max(v_dot_d, -v / (0.1 + self._dt))

        # ── [7] Dynamic inversion — diff-drive dynamics (Alipour 2019 §2, Eq. 33) ─
        # M_eff * v_dot     = (tau_l + tau_r) / R
        # I_eff * omega_dot = (tau_l - tau_r) * d / R
        #
        # Inverse (Eq. 63):
        #   tau_l = R/2 * (M_eff*v_dot - I_eff/d * omega_dot)
        #   tau_r = R/2 * (M_eff*v_dot + I_eff/d * omega_dot)
        #
        # Standard convention: drivewhl_l (left), drivewhl_r (right).
        # Positive omega_dot (turn left) requires right wheel torque > left wheel torque.
        half_R = WHEEL_RADIUS / 2.0
        i_eff = I_EFF_MAP.get(self._mode, I_EFF)
        tau_l = half_R * (M_EFF * v_dot_d - (i_eff / _d) * omega_dot_d)
        tau_r = half_R * (M_EFF * v_dot_d + (i_eff / _d) * omega_dot_d)

        tau_l = max(-TAU_MAX, min(TAU_MAX, tau_l))
        tau_r = max(-TAU_MAX, min(TAU_MAX, tau_r))

        # ── [8] Publish effort commands ────────────────────────────────────────
        cmd = Float64MultiArray()
        cmd.data = [tau_l, tau_r]   # order: [drivewhl_l, drivewhl_r] — matches yaml
        self._pub_effort.publish(cmd)

        # ── [9] Debug output (all intermediate variables) ─────────────────────
        debug = String()
        debug.data = (
            f'mode={MODE_NAMES[self._mode]} dist={dist_goal:.3f} | '
            f'rho={rho:.3f} phi={math.degrees(phi):.2f}deg | '
            f'rho_dot={rho_dot:.4f} phi_dot={phi_dot:.4f} | '
            f'S1={S1:.4f} S2={S2:.4f} sat_S1={sat_S1:.4f} sat_S2={sat_S2:.4f} | '
            f'v_dot_d={v_dot_d:.4f} omega_dot_d={omega_dot_d:.4f} | '
            f'tau_l={tau_l:.4f} tau_r={tau_r:.4f} | '
            f'v={v:.4f} omega={omega:.4f} align={int(alignment_mode)} | '
            f'alpha={math.degrees(self._alpha):.2f}deg'
        )
        self._pub_debug.publish(debug)


def main(args=None):
    rclpy.init(args=args)
    node = SlidingModeControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
