"""Hybrid controller (Solusi 2) — dinamika jurnal + outer-loop robust.

Mempertahankan dari Alipour et al. 2019:
  • Model dinamika H(q̄), C̃ (Lapis 1) — journal_dynamics, tervalidasi
  • Feedback linearization τ = H(q̄)·û + C̃ (Lapis 2, Eq.63)

Mengganti (Lapis 3) hukum kontrol polar 3-surface yg rapuh dengan
PURE-PURSUIT robust: traktor mengikuti path oval, error selalu kecil →
sinyal kontrol tak mentok → tak spinning. Inner-loop H eksak memberi torsi
belok yang BENAR (yaw efektif ~55, bukan aproksimasi 15.7) → mampu menikung.

Outer loop:  e = heading error ke titik lookahead pada path
             v_des = V_REF (ramp + perlambatan dekat jackknife/goal)
             ω0_des = v·κ ,  κ = 2·sin(e)/Ld          (pure pursuit)
Inner loop:  v̇_d = Kv·(v_des−v) ,  ω̇0_d = Kw·(ω0_des−ω0)
             τ = H(θ,θ0)·[v̇_d, ω̇0_d] + C̃(θ,θ0,v,ω0)
"""
import math
import numpy as np

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

try:
    from . import journal_params as P
    from . import journal_dynamics as DYN
except ImportError:
    import journal_params as P
    import journal_dynamics as DYN

TAU_MAX = 30.0
CONTROL_FREQ = 50.0

# Outer-loop (pure pursuit) & inner-loop gains
V_REF    = 0.20       # m/s kecepatan jelajah (diturunkan: kurangi over-bend trailer & lag)
V_RAMP   = 2.0        # s ramp awal
LD       = 2.00       # m jarak lookahead (tractor-following: cukup utk tracking mulus oval r=6)
KV       = 4.0        # gain P loop kecepatan linear → v̇_d
KW       = 12.0       # gain P loop kecepatan sudut → ω̇0_d
KIV      = 2.5        # gain I kecepatan linear (lawan gesekan, capai v_des)
KIW      = 5.0        # gain I kecepatan sudut (lawan stiksi yaw — diturunkan: kurang windup)
K_ALPHA  = 1.2        # gain backstepping: laju traktor membawa hitch → sudut hitch diinginkan
KAPPA_FA = 0.12       # low-pass filter kurvatur path (redam lonjakan FF → tak osilasi)
ALPHA_MAX_SIN = 0.70  # batas sin(hitch diinginkan) ≈ ±44° (jauh dari jackknife 60°)
IV_MAX   = 0.6        # batas integral linear (anti-windup)
IW_MAX   = 0.35       # batas integral sudut (anti-windup)
GOAL_TOL = 0.35       # m — toleransi berhenti: traktor berhenti tepat di goal
SLOW_DIST = 2.50      # m — jarak mulai "final approach" (arahkan traktor langsung ke goal)
JACK_LIM = math.radians(40)   # bila |α| lewat ini: KURANGI belok (bukan perlambat)

LEFT_JOINT, RIGHT_JOINT = 'drivewhl_l_joint', 'drivewhl_r_joint'


def _wrap(a): return math.atan2(math.sin(a), math.cos(a))


class HybridSMC(Node):
    def __init__(self):
        super().__init__('hybrid_smc_controller')
        _latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        self.create_subscription(Path, '/planned_path', self._cb_path, _latched)
        # DIUBAH: sebelumnya '/ground_truth_odom' -- topic itu TIDAK PERNAH ada
        # di arsitektur kita (tidak dibridge dari Gazebo; wheel_odom_publisher
        # yang kita pakai publish ke '/odom' biasa). Tanpa fix ini, node ini
        # diam total selamanya (self._have_gt tidak pernah True).
        self.create_subscription(Odometry, '/odom', self._cb_gt, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self._cb_joint, qos_profile_sensor_data)
        self._pub_eff = self.create_publisher(Float64MultiArray, '/joint_group_effort_controller/commands', 10)
        self._pub_dbg = self.create_publisher(String, '/hybrid_debug', 10)

        self._path = []; self._cum = []; self._plen = 0.0
        self._path_idx = 0           # progres path monoton (cegah lompat di persilangan)
        self._x0 = self._y0 = self._th0 = 0.0
        self._v = self._omega0 = self._alpha = 0.0
        self._iv = self._iw = 0.0    # state integral (lawan gesekan/stiksi)
        self._wdes_prev = 0.0        # state rate-limit w_des
        self._gt0 = None             # pose awal (origin frame map = spawn robot)
        self._kappa_f = 0.0          # state low-pass filter kurvatur
        self._min_dgoal = 1e9        # jarak terdekat ke goal (deteksi sudah melewati)
        # ABLASI (Eksperimen B): use_dynamics=False → H konstan nominal tanpa C̃
        self._use_dyn = self.declare_parameter('use_dynamics', True).value
        self._H_nom = DYN.H(0.0, 0.0)   # H pada konfigurasi lurus (nominal, konstan)
        self.get_logger().info(f'use_dynamics = {self._use_dyn} '
                               f'({"H(q̄)+C̃ penuh jurnal" if self._use_dyn else "ABLASI: H konstan, tanpa C̃"})')
        # SUMBER POSE: 'ground_truth' (empty-room oval, origin=spawn) atau
        # 'tf_map' (depot: pakai TF map→base_link dari AMCL — frame peta sebenarnya)
        self._pose_src = self.declare_parameter('pose_source', 'ground_truth').value
        self._base_frame = self.declare_parameter('base_frame', 'base_link').value
        if self._pose_src == 'tf_map':
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self.get_logger().info(f'pose_source = {self._pose_src}')
        self._have_gt = self._have_joint = False
        self._t0 = None
        self._dt = 1.0 / CONTROL_FREQ
        self.create_timer(self._dt, self._loop)
        self.get_logger().info('Hybrid controller siap — pure-pursuit + dinamika H/C eksak jurnal.')

    # ── Subscribers ───────────────────────────────────────────────────────────
    def _cb_path(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(pts) < 2: return
        self._path = pts
        c = [0.0]
        for i in range(1, len(pts)):
            c.append(c[-1] + math.hypot(pts[i][0]-pts[i-1][0], pts[i][1]-pts[i-1][1]))
        self._cum = c; self._plen = c[-1]; self._t0 = None; self._path_idx = 0
        self._iv = self._iw = 0.0; self._wdes_prev = 0.0; self._min_dgoal = 1e9
        self.get_logger().info(f'Path: {len(pts)} titik, {self._plen:.2f} m')

    def _cb_gt(self, msg):
        if self._pose_src != 'ground_truth':   # mode tf_map: pose dari TF, abaikan gt
            return
        p = msg.pose.pose.position; q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        # origin frame map = pose spawn robot (gt frame world offset dari map).
        # Nyatakan posisi RELATIF thd spawn → sejajar dgn frame path 'map'.
        if self._gt0 is None:
            self._gt0 = (p.x, p.y, yaw)
        dx, dy = p.x - self._gt0[0], p.y - self._gt0[1]
        c, s = math.cos(self._gt0[2]), math.sin(self._gt0[2])
        self._x0 =  c*dx + s*dy        # rotasi balik ke frame spawn
        self._y0 = -s*dx + c*dy
        self._th0 = _wrap(yaw - self._gt0[2])
        self._have_gt = True

    def _update_pose_from_tf(self):
        """Mode tf_map (depot): pose robot di frame map dari TF map→base_link (AMCL)."""
        try:
            tr = self._tf_buffer.lookup_transform('map', self._base_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            return
        t = tr.transform.translation; r = tr.transform.rotation
        self._x0, self._y0 = t.x, t.y
        self._th0 = math.atan2(2*(r.w*r.z+r.x*r.y), 1-2*(r.y*r.y+r.z*r.z))
        self._have_gt = True

    def _cb_joint(self, msg):
        if LEFT_JOINT in msg.name and RIGHT_JOINT in msg.name and msg.velocity:
            wl = msg.velocity[msg.name.index(LEFT_JOINT)]
            wr = msg.velocity[msg.name.index(RIGHT_JOINT)]
            self._v      = P.R0 / 2.0 * (wl + wr)
            self._omega0 = P.R0 / (2*P.B0) * (wl - wr)
        if 'trolley_hitch_joint' in msg.name:
            self._alpha = msg.position[msg.name.index('trolley_hitch_joint')]
        self._have_joint = True

    # ── Pure pursuit: titik lookahead, PROGRES MONOTON ─────────────────────────
    # Cari terdekat HANYA di depan path_idx (window) → tak lompat ke ekor keluar
    # saat path menyilang di bawah oval. Inilah yg bikin robot ikut oval, bukan lurus.
    def _lookahead(self, px, py):
        n = len(self._path)
        win = min(self._path_idx + 60, n)
        ci, cd = self._path_idx, 1e18
        for k in range(self._path_idx, win):
            dd = (self._path[k][0]-px)**2 + (self._path[k][1]-py)**2
            if dd < cd: cd, ci = dd, k
        self._path_idx = ci          # hanya maju
        s_target = self._cum[ci] + LD
        j = ci
        while j < n-1 and self._cum[j] < s_target:
            j += 1
        return self._path[j], ci, j

    # ── Control loop ───────────────────────────────────────────────────────────
    def _loop(self):
        if self._pose_src == 'tf_map':
            self._update_pose_from_tf()       # depot: pose dari TF map→base_link
        if not (self._path and self._have_gt and self._have_joint): return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._t0 is None: self._t0 = now
        t = now - self._t0

        # selesai bila: dekat goal ATAU path tertelusuri ATAU SUDAH MELEWATI goal
        # (mencapai titik terdekat lalu mulai menjauh = mengorbit → berhenti)
        ex, ey = self._path[-1]
        d_goal = math.hypot(ex-self._x0, ey-self._y0)
        self._min_dgoal = min(self._min_dgoal, d_goal)
        # berhenti hanya saat TRAKTOR benar2 di goal (<GOAL_TOL); backup anti-orbit:
        # jika sudah sangat dekat (<0.7m) lalu mulai menjauh = tak bisa lebih dekat → stop
        backup = d_goal < 0.7 and d_goal > self._min_dgoal + 0.20
        if t > V_RAMP and (d_goal < GOAL_TOL or backup):
            self._pub_eff.publish(Float64MultiArray(data=[0.0, 0.0])); return

        # ── TRACTOR-FOLLOWING: traktor ikuti oval; dgn roda trailer mencengkeram +
        # hitch bebas, trailer mengikuti natural di belakang (hitch stabil ~16°).
        la, ci, jla = self._lookahead(self._x0, self._y0)
        e = _wrap(math.atan2(la[1]-self._y0, la[0]-self._x0) - self._th0)
        # kurvatur path (feedforward — traktor ikut lengkungan kontinu tanpa telat)
        n = len(self._path)
        ja = max(jla-8, 0); jb = min(jla+8, n-1)
        tha = math.atan2(self._path[jla][1]-self._path[ja][1], self._path[jla][0]-self._path[ja][0])
        thb = math.atan2(self._path[jb][1]-self._path[jla][1], self._path[jb][0]-self._path[jla][0])
        dss = self._cum[jb] - self._cum[ja]
        kappa_raw = _wrap(thb - tha) / dss if dss > 1e-3 else 0.0
        self._kappa_f += KAPPA_FA * (kappa_raw - self._kappa_f)   # low-pass
        kappa_path = self._kappa_f

        v_des = V_REF * min(1.0, t / V_RAMP)
        if d_goal < SLOW_DIST:
            # FINAL APPROACH: arahkan traktor LANGSUNG ke goal, KECEPATAN PENUH (tak coasting).
            # Robot melaju lurus ke goal & berhenti saat tercapai (bukan meluncur/berhenti dini).
            e_goal = _wrap(math.atan2(ey-self._y0, ex-self._x0) - self._th0)
            w_des = max(-0.5, min(0.5, 1.6 * e_goal))     # arahkan ke goal (clamp anti-jackknife)
        else:
            w_des = v_des * kappa_path + v_des * 2.0 * math.sin(e) / LD   # pure pursuit + FF
        alpha_ss = self._alpha          # debug

        # error kecepatan
        ev = v_des - self._v
        ew = w_des - self._omega0
        # integral (anti-windup: bekukan bila torsi sebelumnya jenuh)
        if not getattr(self, '_sat', False):
            self._iv = max(-IV_MAX, min(IV_MAX, self._iv + ev*self._dt))
            self._iw = max(-IW_MAX, min(IW_MAX, self._iw + ew*self._dt))

        # outer→inner: PI kecepatan → percepatan desired (P+I lawan gesekan/stiksi)
        vdot_d = KV*ev + KIV*self._iv
        wdot_d = KW*ew + KIW*self._iw

        # inner loop: τ = H(θ,θ0)·[v̇,ω̇0] + C̃ (penuh jurnal) ATAU ablasi (H konstan)
        uvec = np.array([vdot_d, wdot_d])
        if self._use_dyn:
            th = self._th0 - self._alpha
            tau = DYN.H(th, self._th0) @ uvec + DYN.Cterm(th, self._th0, self._v, self._omega0)
        else:
            tau = self._H_nom @ uvec        # ABLASI: tanpa H(q̄) state-dependent & tanpa C̃
        traw_l, traw_r = float(tau[0]), float(tau[1])
        tau_l = max(-TAU_MAX, min(TAU_MAX, traw_l))
        tau_r = max(-TAU_MAX, min(TAU_MAX, traw_r))
        self._sat = (tau_l != traw_l) or (tau_r != traw_r)
        self._pub_eff.publish(Float64MultiArray(data=[tau_l, tau_r]))

        self._pub_dbg.publish(String(data=(
            f't={t:.1f} | e={math.degrees(e):.1f} v={self._v:.2f}/{v_des:.2f} '
            f'w={self._omega0:.3f}/{w_des:.3f} alpha={math.degrees(self._alpha):.1f}/{math.degrees(alpha_ss):.1f} | '
            f'vdot_d={vdot_d:.2f} wdot_d={wdot_d:.2f} | tau=({tau_l:.1f},{tau_r:.1f})')))


def main(args=None):
    rclpy.init(args=args)
    node = HybridSMC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():           # cegah "rcl_shutdown already called" di Jazzy
            rclpy.shutdown()


if __name__ == '__main__':
    main()
