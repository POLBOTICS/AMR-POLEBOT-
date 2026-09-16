"""Journal SMC — replikasi PENUH controller Alipour et al. 2019 untuk robot kita.

Komponen (sesuai jurnal):
  • 3 sliding surface (Eq.65/72/83) + virtual input û3 (Eq.79)
  • Feedback-linearization dgn H(q̄) & C̃ eksak (Eq.63) → journal_dynamics
  • Frame-changing 3-frame (Sec.3.2, Fig.8) utk singularitas cos(θ−φ)=0
  • Kontrol TRAILER (titik P) dlm koordinat polar melacak referensi oval

Feedback:
  θ0 (ground-truth traktor), α (hitch /joint_states) → θ=θ0−α, P (forward kin)
Referensi q̄_r(t): dari /planned_path (oval BFS), parametrisasi v_r konstan.
Parameter: journal_params (Tabel 3 + geometri robot kita).
"""
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
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

HITCH_LENGTH = 0.586
D_PRIME      = P.D_PRIME
TAU_MAX      = 30.0
CONTROL_FREQ = 50.0

G1, G2, G3 = P.GAMMA            # γ1,γ2,γ3
Q1, Q2, Q3 = P.Q_SMC           # Q1,Q2,Q3
P1, P2     = P.P_UNC           # P1,P2
V_REF      = P.V_REF
V_RAMP     = 3.0               # detik ramp v_r

PHI_BL  = 0.10                 # boundary-layer sat() anti-chatter
V_FLOOR = 0.08
RHO_MIN = 0.10
COS_SW  = 0.30                 # ambang ganti frame: pilih |cos(θ−φ)| terbesar

# Batas fisik & filter (hardening Solusi 1) — cegah ledakan numerik
U3DOT_MAX = 2.0                # rad/s  — clamp turunan virtual input û̇3
U1_MAX    = 2.0                # m/s²   — clamp percepatan linear desired
U2_MAX    = 3.0                # rad/s² — clamp percepatan sudut desired
U3DOT_FA  = 0.25               # koef low-pass filter û̇3 (0..1; kecil=halus)

LEFT_JOINT, RIGHT_JOINT = 'drivewhl_l_joint', 'drivewhl_r_joint'


def _wrap(a): return math.atan2(math.sin(a), math.cos(a))
def _sat(x, b=PHI_BL): return max(-1.0, min(1.0, x / b))


class JournalSMC(Node):
    def __init__(self):
        super().__init__('journal_smc_controller')
        self.create_subscription(Path, '/planned_path', self._cb_path, 10)
        # DIUBAH: sebelumnya '/ground_truth_odom' -- topic itu TIDAK PERNAH ada
        # di arsitektur kita (tidak dibridge dari Gazebo; wheel_odom_publisher
        # yang kita pakai publish ke '/odom' biasa). Tanpa fix ini, node ini
        # diam total selamanya (self._have_gt tidak pernah True).
        self.create_subscription(Odometry, '/odom', self._cb_gt, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self._cb_joint, qos_profile_sensor_data)
        self._pub_eff = self.create_publisher(Float64MultiArray, '/joint_group_effort_controller/commands', 10)
        self._pub_dbg = self.create_publisher(String, '/journal_debug', 10)

        self._path = []; self._cum = []; self._plen = 0.0
        self._x0 = self._y0 = self._th0 = 0.0
        self._alpha = 0.0; self._omega0 = 0.0
        self._have_gt = self._have_joint = False
        self._t0 = None; self._prev = None; self._frame = -1
        self._ref_idx = 0            # BUG FIX #2: memori indeks progress proyeksi path
        self._u3dot_f = 0.0          # state low-pass filter û̇3
        self._gt0 = None             # FIX FRAME: pose spawn = origin frame map
        self._dt = 1.0 / CONTROL_FREQ

        # 3 origin frame (Fig.8): segitiga moderat mengelilingi pusat oval.
        # DIUBAH: sebelumnya HARDCODE (5.5, 6.0, radius 8.0) -- angka skala
        # oval BESAR dari jurnal asli, sama sekali tidak nyambung dengan
        # oval_cx/oval_cy/oval_r kecil yang dipakai journal_path_planner
        # (mis. 1.2, 0.0, 1.0). Akibatnya rho dihitung dari titik acuan
        # belasan meter jauhnya dari lintasan sebenarnya -> torsi mentok
        # maksimum terus tanpa pernah stabil ("jalan tak karuan").
        # Sekarang jadi parameter ROS, WAJIB disamakan dengan oval_cx/cy/r
        # yang dipakai journal_path_planner di launch file yang sama.
        self.declare_parameter('frame_cx', 1.2)
        self.declare_parameter('frame_cy', 0.0)
        self.declare_parameter('frame_radius', 1.5)
        cx = self.get_parameter('frame_cx').value
        cy = self.get_parameter('frame_cy').value
        Rf = self.get_parameter('frame_radius').value
        self._frames = [(cx + Rf*math.cos(math.radians(a)),
                         cy + Rf*math.sin(math.radians(a))) for a in (90, 210, 330)]

        self.create_timer(self._dt, self._loop)
        self.get_logger().info('Journal SMC (full) siap — 3-surface + H/C eksak + frame-changing.')

    # ── Subscribers ───────────────────────────────────────────────────────────
    def _cb_path(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(pts) < 2: return
        self._path = pts
        c = [0.0]
        for i in range(1, len(pts)):
            c.append(c[-1] + math.hypot(pts[i][0]-pts[i-1][0], pts[i][1]-pts[i-1][1]))
        self._cum = c; self._plen = c[-1]
        self._t0 = None; self._prev = None; self._frame = -1
        self._ref_idx = 0
        self.get_logger().info(f'Path: {len(pts)} titik, {self._plen:.2f} m')

    def _cb_gt(self, msg):
        p = msg.pose.pose.position; q = msg.pose.pose.orientation
        yawv = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        # FIX FRAME: nyatakan posisi relatif spawn → sejajar frame path 'map'
        if self._gt0 is None:
            self._gt0 = (p.x, p.y, yawv)
        dx, dy = p.x - self._gt0[0], p.y - self._gt0[1]
        c, s = math.cos(self._gt0[2]), math.sin(self._gt0[2])
        self._x0 =  c*dx + s*dy
        self._y0 = -s*dx + c*dy
        self._th0 = _wrap(yawv - self._gt0[2])
        self._have_gt = True

    def _cb_joint(self, msg):
        if LEFT_JOINT in msg.name and RIGHT_JOINT in msg.name and msg.velocity:
            wl = msg.velocity[msg.name.index(LEFT_JOINT)]
            wr = msg.velocity[msg.name.index(RIGHT_JOINT)]
            self._omega0 = P.R0 / (2*P.B0) * (wr - wl)
        if 'trolley_hitch_joint' in msg.name:
            self._alpha = msg.position[msg.name.index('trolley_hitch_joint')]
        self._have_joint = True

    # ── Geometri: titik kontrol trailer P (map frame) ──────────────────────────
    def _trailer_P(self):
        th = self._th0 - self._alpha
        hx = self._x0 - HITCH_LENGTH*math.cos(self._th0)
        hy = self._y0 - HITCH_LENGTH*math.sin(self._th0)
        px = hx - D_PRIME*math.cos(th)
        py = hy - D_PRIME*math.sin(th)
        return px, py, th

    # ── Frame-changing dgn HYSTERESIS: cegah flip-flop yg bikin spin ───────────
    def _pick_frame(self, px, py, th):
        sc = []
        for ox, oy in self._frames:
            phi = math.atan2(py - oy, px - ox)
            sc.append(abs(math.cos(_wrap(th - phi))))
        bestk = int(max(range(len(sc)), key=lambda k: sc[k]))
        if self._frame < 0:
            return bestk
        # ganti frame hanya jika kandidat jauh lebih baik (margin 0.20)
        return bestk if sc[bestk] > sc[self._frame] + 0.20 else self._frame

    def _polar(self, px, py, frame):
        ox, oy = self._frames[frame]
        dx, dy = px - ox, py - oy
        return math.hypot(dx, dy), math.atan2(dy, dx)

    # ── Referensi IKUT PROGRES ROBOT (proyeksi P ke path + lookahead) ──────────
    # Cegah referensi-kabur: titik referensi selalu sedikit di depan robot.
    LOOKAHEAD_REF   = 1.0   # m
    # BUG FIX #2: oval kita SENGAJA self-crossing (tail_in & tail_out lewat
    # dekat titik masuk/keluar oval yg cuma berjarak ~0.6 m secara Euclidean,
    # tapi berjarak BANYAK meter secara arc-length -- hampir 1 putaran oval).
    # Pencarian "titik path terdekat" yg dilakukan mentah ke SELURUH path tiap
    # siklus (50x/detik) bisa "melompat" antara ke-2 titik silang itu begitu
    # posisi robot sedikit saja bergeser dekat area itu -> referensi tiba-tiba
    # lompat ke ujung path (LOOKAHEAD lalu ke-clamp ke self._plen), controller
    # melihat error raksasa sesaat, lalu snap balik -> persis gejala "belok ke
    # suatu titik lalu mendadak mundur menjauh". Fix: batasi pencarian ke
    # jendela arc-length di sekitar progres SIKLUS SEBELUMNYA (self._ref_idx),
    # bukan ke seluruh index array -- jendela ini jauh lebih kecil dari jarak
    # arc-length ke titik silang manapun, jadi index TIDAK BISA melompat.
    REF_FWD_WINDOW  = 2.5   # m -- cukup longgar utk progres normal per siklus
    REF_BACK_WINDOW = 0.5   # m -- toleransi kecil mundur (redam noise/derau)

    def _reference(self, t, frame, px, py):
        vr = V_REF * min(1.0, t / V_RAMP)
        # titik path terdekat ke trailer P, dicari HANYA di jendela arc-length
        # di sekitar indeks progres siklus sebelumnya (self._ref_idx) --
        # bukan di seluruh path -- supaya tahan thd self-crossing oval.
        s_lo = max(0.0, self._cum[self._ref_idx] - self.REF_BACK_WINDOW)
        s_hi = min(self._plen, self._cum[self._ref_idx] + self.REF_FWD_WINDOW)
        lo = 0
        while lo < len(self._cum) - 1 and self._cum[lo] < s_lo: lo += 1
        hi = len(self._path) - 1
        while hi > lo and self._cum[hi] > s_hi: hi -= 1
        ci, cd = lo, 1e18
        for k in range(lo, hi + 1):
            dd = (self._path[k][0]-px)**2 + (self._path[k][1]-py)**2
            if dd < cd: cd, ci = dd, k
        self._ref_idx = ci     # simpan progres -> siklus berikutnya lanjut dari sini
        s = min(self._cum[ci] + self.LOOKAHEAD_REF, self._plen)
        i = 0
        while i < len(self._cum)-1 and self._cum[i+1] < s: i += 1
        j = min(i+1, len(self._path)-1)
        seg = max(self._cum[j]-self._cum[i], 1e-6); u = (s-self._cum[i])/seg
        rx = self._path[i][0] + u*(self._path[j][0]-self._path[i][0])
        ry = self._path[i][1] + u*(self._path[j][1]-self._path[i][1])
        i2, j2 = max(i-2, 0), min(j+2, len(self._path)-1)
        th_r = math.atan2(self._path[j2][1]-self._path[i2][1], self._path[j2][0]-self._path[i2][0])
        kappa = self._curv(i2, (i+j)//2, j2)
        th0_r = th_r + math.atan(D_PRIME*kappa)
        ox, oy = self._frames[frame]
        rho_r = math.hypot(rx-ox, ry-oy); phi_r = math.atan2(ry-oy, rx-ox)
        vdot_r = (V_REF/V_RAMP) if t < V_RAMP else 0.0
        rhodot_r = vr*math.cos(_wrap(th_r - phi_r))
        phidot_r = (vr/max(rho_r, RHO_MIN))*math.sin(_wrap(th_r - phi_r))
        done = s >= self._plen - 1e-3
        return dict(rho=rho_r, phi=phi_r, th=th_r, th0=th0_r, v=vr, vdot=vdot_r,
                    rhodot=rhodot_r, phidot=phidot_r, done=done, rx=rx, ry=ry)

    def _curv(self, ia, ib, ic):
        ax, ay = self._path[ia]; bx, by = self._path[ib]; cx, cy = self._path[ic]
        a = math.hypot(bx-ax, by-ay); b = math.hypot(cx-bx, cy-by); c = math.hypot(cx-ax, cy-ay)
        area2 = abs((bx-ax)*(cy-ay)-(by-ay)*(cx-ax))
        return 0.0 if a*b*c < 1e-6 else 2.0*area2/(a*b*c)

    # ── Control loop ───────────────────────────────────────────────────────────
    def _loop(self):
        if not (self._path and self._have_gt and self._have_joint): return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._t0 is None: self._t0 = now
        t = now - self._t0

        px, py, th = self._trailer_P()
        frame = self._pick_frame(px, py, th)
        if frame != self._frame:           # frame berganti → reset turunan numerik & filter
            self._prev = None; self._frame = frame; self._u3dot_f = 0.0
        rho, phi = self._polar(px, py, frame)
        ref = self._reference(t, frame, px, py)

        # selesai → stop di ujung path
        if ref['done'] and t > V_RAMP and math.hypot(self._path[-1][0]-px, self._path[-1][1]-py) < 0.30:
            self._stop(); return

        dphi = _wrap(th - phi)
        cph, sph = math.cos(dphi), math.sin(dphi)
        rho_s = max(rho, RHO_MIN)

        # turunan numerik feedback
        if self._prev is None:
            rhodot, phidot = ref['rhodot'], ref['phidot']
            phidot_r_prev, u3_prev, vest = ref['phidot'], th, ref['v']
        else:
            rho_p, phi_p, phidot_r_prev, u3_prev, vest = self._prev
            rhodot = (rho - rho_p)/self._dt
            phidot = _wrap(phi - phi_p)/self._dt
            vest = rhodot/cph if abs(cph) > COS_SW else vest   # frame-changing jaga |cph|>0.3
        v = vest

        # ── Surface 1 → û1 (Eq.65,67,70) ───────────────────────────────────────
        E_rho = rho - ref['rho']; Edot_rho = rhodot - ref['rhodot']
        S1 = Edot_rho + G1*E_rho
        R1 = (v*v/rho_s)*sph*sph - (v*v/D_PRIME)*sph*math.tan(self._alpha)
        dr = _wrap(ref['th'] - ref['phi'])
        R1r = (ref['v']**2/max(ref['rho'],RHO_MIN))*math.sin(dr)**2
        u1 = (1.0/cph)*(-R1 + ref['vdot']*math.cos(dr) + R1r - G1*Edot_rho - P1*_sat(S1) - Q1*S1)
        u1 = max(-U1_MAX, min(U1_MAX, u1))     # clamp percepatan linear (fisik)

        # ── Surface 2 → û3 virtual (Eq.72,75,79) ───────────────────────────────
        E_phi = _wrap(phi - ref['phi']); Edot_phi = phidot - ref['phidot']
        S2 = Edot_phi + G2*E_phi
        phiddot_r = (ref['phidot'] - phidot_r_prev)/self._dt
        R2 = -(v*rhodot/(rho_s*rho_s))*sph - (v*phidot/rho_s)*cph - phiddot_r + G2*Edot_phi
        v2 = max(v*v, V_FLOOR*V_FLOOR)
        arg = (rho_s*D_PRIME/v2)*(1.0/cph)*(-R2 - (sph/rho_s)*(u1 + P1*_sat(S2)) - Q2*S2)
        arg = max(-1.5, min(1.5, arg))     # clamp ke batas hitch fisik (atan 1.5 ≈ 56°)
        u3 = th + math.atan(arg)

        # ── Surface 3 → û2 (Eq.82,83,86) ───────────────────────────────────────
        E_u = _wrap(self._th0 - u3)
        # û̇3: derivatif numerik DIKERASKAN — clamp ke batas fisik + low-pass filter
        # (cegah ledakan saat u3 melompat / ganti frame). 0 di langkah pertama.
        if self._prev is None:
            u3dot = 0.0
        else:
            raw = _wrap(u3 - u3_prev) / self._dt
            raw = max(-U3DOT_MAX, min(U3DOT_MAX, raw))      # clamp fisik
            self._u3dot_f += U3DOT_FA * (raw - self._u3dot_f)  # low-pass
            u3dot = self._u3dot_f
        Edot_u = self._omega0 - u3dot
        S3 = Edot_u + G3*E_u
        u2 = u3dot - G3*Edot_u - P2*_sat(S3) - Q3*S3
        u2 = max(-U2_MAX, min(U2_MAX, u2))      # clamp percepatan sudut (fisik)

        # ── Inner loop EKSAK: τ = H(q̄)·[û1,û2] + C̃(q̄,u) (Eq.63) ──────────────
        Hm = DYN.H(th, self._th0)
        Cv = DYN.Cterm(th, self._th0, v, self._omega0)
        tau = Hm @ np.array([u1, u2]) + Cv
        tau_l = float(max(-TAU_MAX, min(TAU_MAX, tau[0])))
        tau_r = float(max(-TAU_MAX, min(TAU_MAX, tau[1])))
        self._pub_eff.publish(Float64MultiArray(data=[tau_l, tau_r]))

        self._prev = (rho, phi, ref['phidot'], u3, v)
        self._pub_dbg.publish(String(data=(
            f't={t:.1f} F{frame} | rho={rho:.2f} rho_r={ref["rho"]:.2f} '
            f'phi={math.degrees(phi):.0f} phi_r={math.degrees(ref["phi"]):.0f} '
            f'cos={cph:.2f} | E_rho={E_rho:.3f} E_phi={math.degrees(E_phi):.1f} '
            f'alpha={math.degrees(self._alpha):.1f} | S1={S1:.2f} S2={S2:.2f} S3={S3:.2f} | '
            f'u1={u1:.2f} u2={u2:.2f} | tau=({tau_l:.2f},{tau_r:.2f})')))

    def _stop(self):
        self._pub_eff.publish(Float64MultiArray(data=[0.0, 0.0]))


def main(args=None):
    rclpy.init(args=args)
    node = JournalSMC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
