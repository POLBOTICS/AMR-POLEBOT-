import math
import time
import csv
import os
import heapq
from collections import deque

import numpy as np
from scipy.ndimage import binary_dilation, distance_transform_edt

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Int32, String
from visualization_msgs.msg import Marker, MarkerArray

# Inflation radii per towing mode:
# Solo: 0.58m (lebar AMR 0.67m -> batas clearance fisik aman saat nyerong & belok, tidak menyerempet dinding)
# Fixed: 0.92m (kendaraan kaku panjang 2.766m, lebar 1.00m, clearance ekstra aman untuk swept envelope)
# Pivot: 0.90m (kendaraan gandeng artikulasi panjang 2.766m, clearance ekstra aman agar trailer tidak menyerempet rak)
INFLATE_RADIUS = {0: 0.58, 1: 0.92, 2: 0.90}  # metre

# Kinematic minimum turning radius (R_min) untuk busur non-holonomik:
# Solo: 0.25m (belokan rapat, lincah, dan tidak melebar ke dinding seberang)
# Fixed: 1.15m (kurva kontinu rigid wheelbase panjang 1.931m, radius aman & stabil tanpa off-tracking berlebih)
# Pivot: 1.25m (busur lebar membatasi sudut artikulasi hitch di bawah batas fisik SDF +/- 90 deg / 1.57 rad)
R_MIN = {0: 0.25, 1: 1.15, 2: 1.25}          # metre

# Corner outward push clearance (dorongan sudut belokan ke arah ruang bebas):
# Solo: 0.00m (Solo diff-drive tidak perlu outward push karena tidak membawa trailer yang off-tracking)
# Fixed: 0.30m (dorongan aman sudut dalam tikungan tanpa menabrak dinding seberang di lorong 2.0m)
# Pivot: 0.25m (mengompensasi inside corner cutting trailer 25cm tanpa menabrak dinding luar)
CORNER_PUSH = {0: 0.00, 1: 0.30, 2: 0.25}     # metre

PATH_STEP       = 0.08   # metre — equidistant interpolation step
OCCUPIED_THRESH = 65    # OccupancyGrid cell value (0-100); unknown (-1) treated as free

# 8-connected neighbour offsets with Euclidean step costs (1.0 cardinal, sqrt(2) diagonal)
DIRS_8 = [
    (-1, -1, 1.41421356), (-1, 0, 1.0), (-1, 1, 1.41421356),
    (0, -1, 1.0),                        (0, 1, 1.0),
    (1, -1, 1.41421356),  (1, 0, 1.0),  (1, 1, 1.41421356)
]


class BFSPlannerNode(Node):
    def __init__(self):
        super().__init__('bfs_planner')

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        self._sub_map  = self.create_subscription(OccupancyGrid, '/map', self._cb_map, map_qos)
        self._sub_goal = self.create_subscription(PoseStamped, '/goal_pose', self._cb_goal, 10)
        self._sub_mode = self.create_subscription(Int32, '/towing_mode', self._cb_mode, 10)

        # path di-LATCH (transient_local) → tertangkap record & controller meski mulai belakangan
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        self._pub_path     = self.create_publisher(Path, '/planned_path', latched)
        self._pub_ref_path = self.create_publisher(Path, '/reference_path', latched)
        self._pub_markers  = self.create_publisher(MarkerArray, '/bfs_markers', 10)
        self._pub_warning  = self.create_publisher(String, '/planner_warning', 10)

        # TF buffer created once here — never inside a callback (CLAUDE.md rule)
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._map:  OccupancyGrid | None = None
        self._goal: PoseStamped   | None = None
        self._mode: int = 0
        self._needs_replan: bool = False  # set by goal/mode; map callback only triggers if True

    # ── Subscribers ──────────────────────────────────────────────────────────

    def _cb_map(self, msg: OccupancyGrid):
        self._map = msg
        if self._needs_replan:          # only replan when goal/mode changed, not on every SLAM update
            self._needs_replan = False
            self._try_plan()

    def _cb_goal(self, msg: PoseStamped):
        self._goal = msg
        self._needs_replan = True
        self._try_plan()

    def _cb_mode(self, msg: Int32):
        if msg.data != self._mode:
            self._mode = msg.data
            self._needs_replan = True
            self._try_plan()

    # ── Grid utilities ───────────────────────────────────────────────────────

    def _world_to_cell(self, wx: float, wy: float, origin, resolution: float):
        col = int((wx - origin.x) / resolution)
        row = int((wy - origin.y) / resolution)
        return row, col

    def _cell_to_world(self, row: int, col: int, origin, resolution: float):
        x = origin.x + (col + 0.5) * resolution
        y = origin.y + (row + 0.5) * resolution
        return x, y

    def _inflate_grid(self, grid: np.ndarray, radius_m: float, resolution: float) -> np.ndarray:
        radius_cells = int(math.ceil(radius_m / resolution))
        yr, xr = np.ogrid[-radius_cells:radius_cells + 1, -radius_cells:radius_cells + 1]
        kernel = (xr ** 2 + yr ** 2 <= radius_cells ** 2)
        return binary_dilation(grid, structure=kernel)

    def _find_nearest_free(self, r: int, c: int, grid: np.ndarray, max_cells: int = 30) -> tuple[int, int] | None:
        """Cari sel bebas rintangan terdekat secara radial (Euclidean) hingga max_cells (1.5 meter)."""
        if not grid[r, c]:
            return r, c
        rows, cols = grid.shape
        best_cell = None
        best_d2 = float('inf')
        for dr in range(-max_cells, max_cells + 1):
            nr = r + dr
            if not (0 <= nr < rows):
                continue
            for dc in range(-max_cells, max_cells + 1):
                nc = c + dc
                if not (0 <= nc < cols):
                    continue
                if not grid[nr, nc]:
                    d2 = dr * dr + dc * dc
                    if d2 < best_d2:
                        best_d2 = d2
                        best_cell = (nr, nc)
                        if best_d2 <= 2:
                            return best_cell
        return best_cell

    # ── BFS ──────────────────────────────────────────────────────────────────

    def _bfs(self, inflated: np.ndarray, start: tuple, goal: tuple, occupied: np.ndarray = None, resolution: float = 0.05):
        rows, cols = inflated.shape
        sr, sc = start
        gr, gc = goal

        # Jika goal berada di area terinflasi/dekat obstacle, geser ke sel bebas terdekat
        if inflated[gr, gc]:
            nearest_g = self._find_nearest_free(gr, gc, inflated, max_cells=30)
            if nearest_g is None:
                self.get_logger().error('Goal too deep in obstacle — cannot plan')
                return None
            gr, gc = nearest_g
            self.get_logger().warn(f'Goal in obstacle margin, shifted to free cell ({gr},{gc})')

        # Jika start berada di area terinflasi/dekat obstacle, geser ke sel bebas terdekat
        if inflated[sr, sc]:
            nearest_s = self._find_nearest_free(sr, sc, inflated, max_cells=30)
            if nearest_s is None:
                self.get_logger().error('Start cell too deep in obstacle — cannot plan')
                return None
            sr, sc = nearest_s
            self.get_logger().warn(f'Start in obstacle margin, shifted to free cell ({sr},{sc})')

        # Euclidean Distance Transform untuk bobot clearance (menjauhkan jalur dari tepi dinding/rintangan)
        dist_transform = None
        if occupied is not None:
            dist_transform = distance_transform_edt(~occupied) * resolution
        # Mode 1 & 2 (towing) menggunakan bobot clearance lebih tinggi (0.20) agar jalur berada di tengah koridor
        clearance_weight = 0.08 if self._mode == 0 else 0.20

        # A* Optimal Euclidean Shortest Path (Pencarian Jalur Tercepat & Terpendek)
        h0 = math.hypot(gr - sr, gc - sc)
        pq = [(h0, 0.0, sr, sc)]
        g_score: dict[tuple, float] = {(sr, sc): 0.0}
        parent: dict[tuple, tuple | None] = {(sr, sc): None}

        while pq:
            f, g, r, c = heapq.heappop(pq)
            if r == gr and c == gc:
                path = []
                cur: tuple | None = (r, c)
                while cur is not None:
                    path.append(cur)
                    cur = parent[cur]
                self._last_explored = len(parent)   # jumlah sel dieksplorasi (metrik BFS/A*)
                return path[::-1]

            if g > g_score.get((r, c), float('inf')):
                continue

            for dr, dc, step_w in DIRS_8:
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols and not inflated[nr, nc]:
                    penalty = 0.0
                    if dist_transform is not None:
                        clearance = dist_transform[nr, nc]
                        penalty = clearance_weight / max(clearance, 0.1)
                    tentative_g = g + step_w + penalty

                    if tentative_g < g_score.get((nr, nc), float('inf')):
                        g_score[(nr, nc)] = tentative_g
                        parent[(nr, nc)] = (r, c)
                        h = math.hypot(gr - nr, gc - nc)
                        heapq.heappush(pq, (tentative_g + h, tentative_g, nr, nc))

        return None

    def _log_bfs_csv(self, mode, inflate, t_ms, explored, n_bfs, n_pull, n_interp,
                     n_smooth, plen, dstraight):
        """Akumulasi metrik BFS ke ~/smc_logs/bfs_metrics.csv untuk laporan."""
        path = os.path.expanduser('~/smc_logs/bfs_metrics.csv')
        os.makedirs(os.path.dirname(path), exist_ok=True)
        head = not os.path.exists(path)
        with open(path, 'a', newline='') as f:
            w = csv.writer(f)
            if head:
                w.writerow(['mode', 'inflate_m', 'waktu_ms', 'sel_dieksplorasi',
                            'titik_bfs', 'titik_pull', 'titik_interp', 'titik_smooth',
                            'panjang_m', 'jarak_lurus_m', 'rasio_panjang'])
            w.writerow([mode, inflate, round(t_ms, 2), explored, n_bfs, n_pull,
                        n_interp, n_smooth, round(plen, 3), round(dstraight, 3),
                        round(plen/max(dstraight, 0.01), 3)])

    # ── Path post-processing ─────────────────────────────────────────────────

    def _interpolate_equidistant(
            self, pts: list[tuple[float, float]], step: float
    ) -> list[tuple[float, float]]:
        """Resample path so consecutive points are exactly `step` metres apart."""
        if len(pts) < 2:
            return pts
        result = [pts[0]]
        residual = 0.0
        for i in range(1, len(pts)):
            p0 = result[-1]
            p1 = pts[i]
            seg = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            remaining = seg
            while residual + remaining >= step:
                t = (step - residual) / remaining
                nx = p0[0] + t * (p1[0] - p0[0])
                ny = p0[1] + t * (p1[1] - p0[1])
                result.append((nx, ny))
                remaining -= (step - residual)
                residual = 0.0
                p0 = (nx, ny)
            residual += remaining
        # Always include the original goal point
        if math.hypot(result[-1][0] - pts[-1][0], result[-1][1] - pts[-1][1]) > 0.01:
            result.append(pts[-1])
        return result

    def _smooth(
            self, pts: list[tuple[float, float]], window: int
    ) -> list[tuple[float, float]]:
        """Sliding-window average to reduce BFS staircase artefacts."""
        n = len(pts)
        hw = window // 2
        result = []
        for i in range(n):
            lo, hi = max(0, i - hw), min(n, i + hw + 1)
            xs = [pts[j][0] for j in range(lo, hi)]
            ys = [pts[j][1] for j in range(lo, hi)]
            result.append((sum(xs) / len(xs), sum(ys) / len(ys)))
        return result

    def _line_free(
            self,
            p1: tuple[float, float],
            p2: tuple[float, float],
            inflated: 'np.ndarray',
            origin,
            resolution: float,
    ) -> bool:
        """True jika garis lurus p1→p2 tidak melewati sel inflated obstacle."""
        dist  = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        steps = max(2, int(dist / (resolution * 0.5)))
        rows, cols = inflated.shape
        for k in range(steps + 1):
            t  = k / steps
            wx = p1[0] + t * (p2[0] - p1[0])
            wy = p1[1] + t * (p2[1] - p1[1])
            c  = int((wx - origin.x) / resolution)
            r  = int((wy - origin.y) / resolution)
            if not (0 <= r < rows and 0 <= c < cols):
                return False
            if inflated[r, c]:
                return False
        return True

    def _string_pull(
            self,
            pts: list[tuple[float, float]],
            inflated: 'np.ndarray',
            origin,
            resolution: float,
    ) -> list[tuple[float, float]]:
        """Greedy string pulling: sambungkan titik-titik yang bisa dihubungkan
        lurus tanpa melewati obstacle → path lurus di area bebas."""
        if len(pts) < 3:
            return pts
        result = [pts[0]]
        i = 0
        while i < len(pts) - 1:
            # Coba sambungkan ke titik sejauh mungkin secara langsung
            j = len(pts) - 1
            while j > i + 1:
                if self._line_free(pts[i], pts[j], inflated, origin, resolution):
                    break
                j -= 1
            result.append(pts[j])
            i = j
        return result

    def _prune_collinear(
            self,
            pts: list[tuple[float, float]],
            inflated: 'np.ndarray',
            origin,
            resolution: float,
    ) -> list[tuple[float, float]]:
        """Pangkas titik-titik belokan dangkal yang tidak perlu jika garis lurus bebas rintangan,
        mencegah timbulnya chicane/lekukan patah semu pada koridor lurus."""
        if len(pts) < 3:
            return pts
        pruned = [pts[0]]
        i = 0
        while i < len(pts) - 1:
            best_next = i + 1
            for j in range(len(pts) - 1, i + 1, -1):
                if self._line_free(pts[i], pts[j], inflated, origin, resolution):
                    best_next = j
                    break
            pruned.append(pts[best_next])
            i = best_next
        return pruned

    # ── Robot position via TF ─────────────────────────────────────────────────

    # ── Non-Holonomic Path Synthesis ─────────────────────────────────────────

    def _is_solo_safe(
            self,
            pts: list[tuple[float, float]],
            occupied: 'np.ndarray',
            origin,
            resolution: float,
            margin: float = 0.02,
    ) -> bool:
        """Memeriksa apakah seluruh kontur bodi AMR Solo (panjang 1.172m, lebar 0.670m)
        bebas dari tabrakan sel rintangan (occupied) di setiap titik sepanjang lintasan."""
        if len(pts) < 2:
            return True
        sim_pts = self._interpolate_equidistant(pts, PATH_STEP)
        if len(sim_pts) < 2:
            return True
        thetas = [math.atan2(sim_pts[k + 1][1] - sim_pts[k][1], sim_pts[k + 1][0] - sim_pts[k][0])
                  for k in range(len(sim_pts) - 1)]
        thetas.append(thetas[-1])

        L_half = 0.586 + margin   # 1.172m / 2 + safety margin
        W_half = 0.335 + margin   # 0.670m / 2 + safety margin
        rows, cols = occupied.shape

        # 12 titik kritis di sekeliling batas poligon bodi AMR Solo:
        # 4 sudut bumper, 2 tengah depan/belakang, 4 sisi samping, 2 as roda tengah
        local_pts = [
            (L_half, W_half), (L_half, -W_half),
            (-L_half, W_half), (-L_half, -W_half),
            (L_half, 0.0), (-L_half, 0.0),
            (0.0, W_half), (0.0, -W_half),
            (0.5 * L_half, W_half), (0.5 * L_half, -W_half),
            (-0.5 * L_half, W_half), (-0.5 * L_half, -W_half),
        ]

        for k in range(len(sim_pts)):
            xr, yr = sim_pts[k]
            tr = thetas[k]
            cos_tr = math.cos(tr)
            sin_tr = math.sin(tr)

            for lx, ly in local_pts:
                gx = xr + lx * cos_tr - ly * sin_tr
                gy = yr + lx * sin_tr + ly * cos_tr
                c = int((gx - origin.x) / resolution)
                r = int((gy - origin.y) / resolution)
                if not (0 <= r < rows and 0 <= c < cols) or occupied[r, c]:
                    return False
        return True

    def _is_trailer_safe(
            self,
            pts: list[tuple[float, float]],
            mode: int,
            occupied: 'np.ndarray',
            origin,
            resolution: float,
    ) -> bool:
        """Memeriksa apakah jejak kinematis (swept envelope) trailer bebas tabrakan rintangan."""
        if mode == 0 or len(pts) < 2:
            return True
        # Resample titik-titik agar berjarak PATH_STEP (stabil untuk integrasi numerik sudut trailer)
        sim_pts = self._interpolate_equidistant(pts, PATH_STEP)
        if len(sim_pts) < 2:
            return True
        thetas = [math.atan2(sim_pts[k + 1][1] - sim_pts[k][1], sim_pts[k + 1][0] - sim_pts[k][0])
                  for k in range(len(sim_pts) - 1)]
        thetas.append(thetas[-1])

        # Parameter geometris presisi sesuai polebot_amr_description.sdf:
        d_hitch = 0.786      # base_link robot ke pin pivot (0.586m bumper + 0.200m tongkat pin)
        L_trailer = 1.145    # Jarak kinematis pin pivot ke as roda belakang fixed trolley
        L_rear = 1.393       # Jarak pin pivot ke ujung bumper belakang bodi trolley
        W_half = 0.500       # Separuh lebar rangka trolley (lebar total 1.000m)
        rows, cols = occupied.shape

        trailer_thetas = [thetas[0]]
        for k in range(len(sim_pts) - 1):
            if mode == 1:  # Fixed joint
                trailer_thetas.append(thetas[k + 1])
            else:          # Pivot joint
                hx0 = sim_pts[k][0] - d_hitch * math.cos(thetas[k])
                hy0 = sim_pts[k][1] - d_hitch * math.sin(thetas[k])
                hx1 = sim_pts[k + 1][0] - d_hitch * math.cos(thetas[k + 1])
                hy1 = sim_pts[k + 1][1] - d_hitch * math.sin(thetas[k + 1])
                ds_h = math.hypot(hx1 - hx0, hy1 - hy0)
                phi = math.atan2(hy1 - hy0, hx1 - hx0)
                trailer_thetas.append(trailer_thetas[-1] + (ds_h / L_trailer) * math.sin(phi - trailer_thetas[-1]))

        for k in range(len(sim_pts)):
            xr, yr, tr = sim_pts[k][0], sim_pts[k][1], thetas[k]
            tt = trailer_thetas[k]
            cos_tr, sin_tr = math.cos(tr), math.sin(tr)
            cos_tt, sin_tt = math.cos(tt), math.sin(tt)

            # Batas artikulasi sendi revolute hitch (SDF mechanical limit: +/- 90 deg / 1.57 rad)
            if mode == 2:
                diff = (tr - tt + math.pi) % (2.0 * math.pi) - math.pi
                if abs(diff) > math.radians(82.0):
                    return False

            # Posisi pin pengait (hitch point)
            hx = xr - d_hitch * cos_tr
            hy = yr - d_hitch * sin_tr

            # Titik sudut depan trolley (di sekitar sambungan pivot)
            tf_lx = hx - W_half * sin_tt
            tf_ly = hy + W_half * cos_tt
            tf_rx = hx + W_half * sin_tt
            tf_ry = hy - W_half * cos_tt

            # Titik sudut belakang trolley
            rx = hx - L_rear * cos_tt
            ry = hy - L_rear * sin_tt
            tr_lx = rx - W_half * sin_tt
            tr_ly = ry + W_half * cos_tt
            tr_rx = rx + W_half * sin_tt
            tr_ry = ry - W_half * cos_tt

            # Titik as roda belakang fixed trolley
            ax_x = hx - L_trailer * cos_tt
            ax_y = hy - L_trailer * sin_tt
            ax_lx = ax_x - W_half * sin_tt
            ax_ly = ax_y + W_half * cos_tt
            ax_rx = ax_x + W_half * sin_tt
            ax_ry = ax_y - W_half * cos_tt

            pts_check = [
                # Robot front bumper corners & center
                (xr + 0.586 * cos_tr - 0.335 * sin_tr, yr + 0.586 * sin_tr + 0.335 * cos_tr),
                (xr + 0.586 * cos_tr + 0.335 * sin_tr, yr + 0.586 * sin_tr - 0.335 * cos_tr),
                (xr + 0.586 * cos_tr, yr + 0.586 * sin_tr),
                # Drawbar / hitch pin
                (hx, hy),
                ((xr + hx) * 0.5, (yr + hy) * 0.5),
                # Trolley front & rear corners
                (tf_lx, tf_ly), (tf_rx, tf_ry),
                (tr_lx, tr_ly), (tr_rx, tr_ry),
                # Trolley axle points
                (ax_lx, ax_ly), (ax_rx, ax_ry),
                # Dense side rail sample points (interval ~0.25m - 0.30m)
                ((tf_lx + ax_lx) * 0.5, (tf_ly + ax_ly) * 0.5),
                ((tf_rx + ax_rx) * 0.5, (tf_ry + ax_ry) * 0.5),
                ((ax_lx + tr_lx) * 0.5, (ax_ly + tr_ly) * 0.5),
                ((ax_rx + tr_rx) * 0.5, (ax_ry + tr_ry) * 0.5),
                # Trolley rear bumper (mid & quarter points)
                (rx, ry),
                ((tr_lx + rx) * 0.5, (tr_ly + ry) * 0.5),
                ((tr_rx + rx) * 0.5, (tr_ry + ry) * 0.5),
            ]
            for px, py in pts_check:
                c = int((px - origin.x) / resolution)
                r = int((py - origin.y) / resolution)
                if not (0 <= r < rows and 0 <= c < cols) or occupied[r, c]:
                    return False
        return True

    def _fillet_corner(
            self,
            pa: tuple[float, float],
            pb: tuple[float, float],
            pc: tuple[float, float],
            r_min: float,
            inflated: 'np.ndarray',
            occupied: 'np.ndarray',
            origin,
            resolution: float,
            mode: int = 0,
            num_pts: int = 12,
    ) -> list[tuple[float, float]]:
        """Sisipkan busur lengkung non-holonomik pada sudut pb dengan radius >= r_min,
        lengkap dengan outward push untuk menjamin batas clearance aman saat berbelok."""
        v1 = (pb[0] - pa[0], pb[1] - pa[1])
        v2 = (pc[0] - pb[0], pc[1] - pb[1])
        l1 = math.hypot(v1[0], v1[1])
        l2 = math.hypot(v2[0], v2[1])
        if l1 < 1e-4 or l2 < 1e-4:
            return [pb]

        u1 = (v1[0] / l1, v1[1] / l1)
        u2 = (v2[0] / l2, v2[1] / l2)
        dot = max(-1.0, min(1.0, u1[0] * u2[0] + u1[1] * u2[1]))
        if dot > 0.998:  # hampir lurus, tidak perlu fillet
            return [pb]

        alpha = math.acos(dot)
        bis_x = u1[0] - u2[0]
        bis_y = u1[1] - u2[1]
        bis_len = math.hypot(bis_x, bis_y)
        push_base = CORNER_PUSH.get(mode, 0.0)

        # Terapkan dorongan sudut tikungan ke arah ruang bebas hanya untuk belokan nyata (alpha >= 40 deg)
        # Tikungan dangkal (< 40 deg) TIDAK didorong agar tidak membentuk chicane / zig-zag di samping rintangan
        if bis_len > 1e-3 and alpha >= 0.70 and push_base > 0:
            bis_u = (bis_x / bis_len, bis_y / bis_len)
            push_candidates = [push_base, push_base * 0.75, push_base * 0.50, push_base * 0.25, 0.0]
            for p_dist in push_candidates:
                p_cand = (pb[0] + p_dist * bis_u[0], pb[1] + p_dist * bis_u[1])
                c_c = int((p_cand[0] - origin.x) / resolution)
                r_c = int((p_cand[1] - origin.y) / resolution)
                if 0 <= r_c < occupied.shape[0] and 0 <= c_c < occupied.shape[1] and not occupied[r_c, c_c]:
                    if (self._line_free(pa, p_cand, occupied, origin, resolution) and
                            self._line_free(p_cand, pc, occupied, origin, resolution)):
                        v1c = (p_cand[0] - pa[0], p_cand[1] - pa[1])
                        v2c = (pc[0] - p_cand[0], pc[1] - p_cand[1])
                        l1c, l2c = math.hypot(*v1c), math.hypot(*v2c)
                        u1c, u2c = (v1c[0] / l1c, v1c[1] / l1c), (v2c[0] / l2c, v2c[1] / l2c)
                        dt = min(r_min * math.tan(alpha / 2.0), min(l1c * 0.45, l2c * 0.45))
                        q1 = (p_cand[0] - dt * u1c[0], p_cand[1] - dt * u1c[1])
                        q2 = (p_cand[0] + dt * u2c[0], p_cand[1] + dt * u2c[1])
                        cpts = []
                        for k in range(num_pts):
                            t = k / float(num_pts - 1)
                            bx = (1.0 - t)**2 * q1[0] + 2.0 * (1.0 - t) * t * p_cand[0] + t**2 * q2[0]
                            by = (1.0 - t)**2 * q1[1] + 2.0 * (1.0 - t) * t * p_cand[1] + t**2 * q2[1]
                            cpts.append((bx, by))
                        if all(self._line_free(cpts[k], cpts[k + 1], occupied, origin, resolution)
                                for k in range(len(cpts) - 1)):
                            if mode == 0:
                                if self._is_solo_safe([pa] + cpts + [pc], occupied, origin, resolution):
                                    return cpts
                            else:
                                if self._is_trailer_safe([pa] + cpts + [pc], mode, occupied, origin, resolution):
                                    return cpts

        # Fallback atau Tikungan Dangkal: Early Turn Blending
        # Mulai berbelok lebih awal jika koridor lapang (hingga max_blend)
        # Mode 1 (Fixed) membutuhkan lengkungan 0.80m, Mode 2 (Pivot) 0.85m untuk radius 1.25m
        max_blend = 0.60 if mode == 0 else (0.80 if mode == 1 else 0.85)
        dt_ideal = max(r_min * math.tan(alpha / 2.0), min(max_blend, min(l1 * 0.40, l2 * 0.40)))

        for scale in [1.0, 0.85, 0.70, 0.50, 0.35]:
            dt = dt_ideal * scale
            if dt < 0.05:
                break
            q1 = (pb[0] - dt * u1[0], pb[1] - dt * u1[1])
            q2 = (pb[0] + dt * u2[0], pb[1] + dt * u2[1])

            corner_pts = []
            for k in range(num_pts):
                t = k / float(num_pts - 1)
                bx = (1.0 - t)**2 * q1[0] + 2.0 * (1.0 - t) * t * pb[0] + t**2 * q2[0]
                by = (1.0 - t)**2 * q1[1] + 2.0 * (1.0 - t) * t * pb[1] + t**2 * q2[1]
                corner_pts.append((bx, by))

            safe = all(self._line_free(corner_pts[k], corner_pts[k + 1], occupied, origin, resolution)
                       for k in range(len(corner_pts) - 1))
            if safe:
                if mode == 0:
                    if self._is_solo_safe([pa] + corner_pts + [pc], occupied, origin, resolution):
                        return corner_pts
                else:
                    if self._is_trailer_safe([pa] + corner_pts + [pc], mode, occupied, origin, resolution):
                        return corner_pts

        return [pb]

    def _check_side_clearance(
            self,
            x0: float,
            y0: float,
            theta0: float,
            occupied: 'np.ndarray',
            origin,
            resolution: float,
            max_dist: float = 1.8,
    ) -> tuple[float, float]:
        """Menghitung jarak bebas ke rintangan pada sisi kiri dan kanan robot."""
        nl_x, nl_y = -math.sin(theta0), math.cos(theta0)
        rows, cols = occupied.shape

        def raycast(dx, dy):
            steps = max(2, int(max_dist / (resolution * 0.5)))
            for s in range(1, steps + 1):
                dist = s * resolution * 0.5
                wx = x0 + dist * dx
                wy = y0 + dist * dy
                c = int((wx - origin.x) / resolution)
                r = int((wy - origin.y) / resolution)
                if not (0 <= r < rows and 0 <= c < cols) or occupied[r, c]:
                    return dist
            return max_dist

        dist_left  = raycast(nl_x, nl_y)
        dist_right = raycast(-nl_x, -nl_y)
        return dist_left, dist_right

    def _build_nonholonomic_path(
            self,
            start_pose: tuple[float, float, float],
            waypoints: list[tuple[float, float]],
            r_min: float,
            inflated: 'np.ndarray',
            occupied: 'np.ndarray',
            origin,
            resolution: float,
    ) -> list[tuple[float, float]]:
        """
        Membangun lintasan non-holonomik:
        1. Menghitung kurva fillet non-holonomik (r_min) untuk setiap tikungan
        2. Transisi awal tangensial searah orientasi robot saat ini (theta0)
        3. Memastikan seluruh segmen lintasan bebas tabrakan untuk bodi robot & trailer
        """
        x0, y0, theta0 = start_pose
        if len(waypoints) < 2:
            return waypoints

        # Safety grid dengan margin pengaman 1 sel (5cm) untuk verifikasi amplop belokan mode trailer
        occ_safety = binary_dilation(occupied, structure=np.ones((3, 3))) if self._mode != 0 else occupied

        # 1. Hitung fillet untuk semua sudut belokan (i = 1 .. N-1)
        corners = {}
        for i in range(1, len(waypoints) - 1):
            pa = waypoints[i - 1]
            pb = waypoints[i]
            pc = waypoints[i + 1]
            corners[i] = self._fillet_corner(pa, pb, pc, r_min, inflated, occ_safety, origin, resolution, mode=self._mode)

        first_target = corners[1][0] if (1 in corners and len(corners[1]) > 1) else waypoints[1]

        # 2. Transisi awal: selaraskan tangen awal dengan orientasi hadap robot
        t0 = (math.cos(theta0), math.sin(theta0))
        seg1_dx = first_target[0] - x0
        seg1_dy = first_target[1] - y0
        seg1_len = math.hypot(seg1_dx, seg1_dy)
        seg1_dir = (seg1_dx / max(seg1_len, 1e-4), seg1_dy / max(seg1_len, 1e-4))

        ctrl_dist = min(seg1_len * 0.35, max(r_min * 0.4, 0.12))

        # Cek apakah arah balik (U-turn / sudut belokan besar > 105 deg)
        dot = t0[0] * seg1_dir[0] + t0[1] * seg1_dir[1]
        b1_lat_x, b1_lat_y = 0.0, 0.0
        if dot < -0.25:
            dl, dr = self._check_side_clearance(x0, y0, theta0, occupied, origin, resolution)
            side_sign = 1.0 if dl >= dr else -1.0
            lat_mag = min(max(dl if side_sign > 0 else dr, 0.2) * 0.4, r_min * 0.5)
            b1_lat_x = -side_sign * math.sin(theta0) * lat_mag
            b1_lat_y =  side_sign * math.cos(theta0) * lat_mag
            self.get_logger().info(f'Turn-around detected: clearance Left={dl:.2f}m, Right={dr:.2f}m -> turning to {"LEFT" if side_sign > 0 else "RIGHT"}')

        b0 = (x0, y0)
        b1 = (x0 + ctrl_dist * t0[0] + b1_lat_x, y0 + ctrl_dist * t0[1] + b1_lat_y)
        b3 = first_target
        b2 = (first_target[0] - ctrl_dist * seg1_dir[0], first_target[1] - ctrl_dist * seg1_dir[1])

        num_init = 12
        init_curve = []
        for k in range(num_init):
            t = k / float(num_init - 1)
            bx = ((1.0 - t)**3 * b0[0] + 3.0 * (1.0 - t)**2 * t * b1[0] +
                  3.0 * (1.0 - t) * t**2 * b2[0] + t**3 * b3[0])
            by = ((1.0 - t)**3 * b0[1] + 3.0 * (1.0 - t)**2 * t * b1[1] +
                  3.0 * (1.0 - t) * t**2 * b2[1] + t**3 * b3[1])
            init_curve.append((bx, by))

        safe_init = all(self._line_free(init_curve[k], init_curve[k + 1], occupied, origin, resolution)
                        for k in range(len(init_curve) - 1))
        if safe_init:
            if self._mode == 0:
                safe_init = self._is_solo_safe(init_curve, occupied, origin, resolution)
            else:
                safe_init = self._is_trailer_safe(init_curve, self._mode, occupied, origin, resolution)

        if not safe_init:
            if dot < -0.25:
                # ── SMART TURNAROUND FOR TIGHT CORRIDORS ──
                # Di ruang sempit, trailer tidak boleh memaksakan putar balik di tempat (akan menabrak dinding).
                # Lakukan Forward Search: telusuri koridor lurus ke depan untuk mencari persimpangan/ruang lapang.
                best_fwd_curve = None
                for fwd_d in [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5]:
                    fx = x0 + fwd_d * t0[0]
                    fy = y0 + fwd_d * t0[1]
                    # Pastikan garis lurus maju ke (fx, fy) aman dari rintangan
                    if not self._line_free((x0, y0), (fx, fy), occupied, origin, resolution):
                        break  # Terhalang dinding di depan, hentikan pencarian

                    # Cek clearance di titik maju (fx, fy)
                    dfl, dfr = self._check_side_clearance(fx, fy, theta0, occupied, origin, resolution)
                    fwd_sign = 1.0 if dfl >= dfr else -1.0
                    fwd_lat = min(max(dfl if fwd_sign > 0 else dfr, 0.2) * 0.4, r_min * 0.5)
                    fb1_lat_x = -fwd_sign * math.sin(theta0) * fwd_lat
                    fb1_lat_y =  fwd_sign * math.cos(theta0) * fwd_lat

                    fb0 = (fx, fy)
                    fb1 = (fx + ctrl_dist * t0[0] + fb1_lat_x, fy + ctrl_dist * t0[1] + fb1_lat_y)
                    fb3 = first_target
                    fb2 = (first_target[0] - ctrl_dist * seg1_dir[0], first_target[1] - ctrl_dist * seg1_dir[1])

                    cand_turn = []
                    for k in range(num_init):
                        t = k / float(num_init - 1)
                        bx = ((1.0 - t)**3 * fb0[0] + 3.0 * (1.0 - t)**2 * t * fb1[0] +
                              3.0 * (1.0 - t) * t**2 * fb2[0] + t**3 * fb3[0])
                        by = ((1.0 - t)**3 * fb0[1] + 3.0 * (1.0 - t)**2 * t * fb1[1] +
                              3.0 * (1.0 - t) * t**2 * fb2[1] + t**3 * fb3[1])
                        cand_turn.append((bx, by))

                    # Diskritkan segmen maju lurus
                    fwd_steps = max(2, int(fwd_d / PATH_STEP))
                    straight_fwd = [(x0 + (s / fwd_steps) * (fx - x0), y0 + (s / fwd_steps) * (fy - y0))
                                    for s in range(fwd_steps)]
                    cand_full = straight_fwd + cand_turn

                    cand_safe = all(self._line_free(cand_full[k], cand_full[k + 1], occupied, origin, resolution)
                                    for k in range(len(cand_full) - 1))
                    if cand_safe:
                        if self._mode == 0:
                            cand_safe = self._is_solo_safe(cand_full, occupied, origin, resolution)
                        else:
                            cand_safe = self._is_trailer_safe(cand_full, self._mode, occupied, origin, resolution)

                    if cand_safe:
                        best_fwd_curve = cand_full
                        self.get_logger().info(
                            f'Smart Turnaround: Menemukan ruang putar aman {fwd_d:.1f}m di depan! Robot akan maju sebelum memutar.'
                        )
                        break

                if best_fwd_curve is not None:
                    init_curve = best_fwd_curve
                else:
                    # Tidak ada ruang lapang di depan (koridor buntu sempit)
                    warn_msg = (
                        f'[PLANNER ALERT] Ruang tidak cukup untuk manuver putar balik trolley! '
                        f'Clearance samping ({max(dl, dr):.2f}m) < batas aman. Jalur dibatalkan demi keamanan bodi trolley.'
                    )
                    self.get_logger().error(warn_msg)
                    msg_w = String()
                    msg_w.data = warn_msg
                    self._pub_warning.publish(msg_w)
                    return []
            else:
                init_curve = [(x0, y0)]

        # 3. Rakit lintasan terurut tanpa duplikasi titik dan tanpa lompatan terbalik
        path_pts = list(init_curve)
        for i in range(1, len(waypoints) - 1):
            path_pts.extend(corners[i])
        path_pts.append(waypoints[-1])

        return path_pts

    # ── Robot pose via TF ─────────────────────────────────────────────────────

    def _get_robot_pose(self) -> tuple[float, float, float] | None:
        """Mengambil posisi (x, y) dan orientasi yaw (theta) robot dari TF base_link."""
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            q = tf.transform.rotation
            # Ekstrak sudut yaw dari quaternion
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return x, y, yaw
        except tf2_ros.LookupException as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None
        except Exception as e:
            self.get_logger().warn(f'TF error: {e}')
            return None

    # ── Main planning routine ─────────────────────────────────────────────────

    def _try_plan(self):
        if self._map is None or self._goal is None:
            return

        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            return

        m = self._map
        resolution = m.info.resolution
        origin     = m.info.origin.position
        rows       = m.info.height
        cols       = m.info.width

        # Build binary occupancy array; unknown (-1) treated as free for SLAM startup
        raw = np.array(m.data, dtype=np.int8).reshape(rows, cols)
        occupied = raw >= OCCUPIED_THRESH  # unknown (-1) treated as free — correct for SLAM startup

        # Inflate obstacles based on current towing mode
        inflate_m = INFLATE_RADIUS.get(self._mode, INFLATE_RADIUS[0])
        r_min     = R_MIN.get(self._mode, R_MIN[0])
        inflated = self._inflate_grid(occupied, inflate_m, resolution)

        start_cell = self._world_to_cell(robot_pose[0], robot_pose[1], origin, resolution)
        goal_cell  = self._world_to_cell(
            self._goal.pose.position.x, self._goal.pose.position.y, origin, resolution)

        sr, sc = start_cell
        gr, gc = goal_cell
        # Jika sel berada sedikit di luar batas peta, jepit ke batas sel terdekat
        sr = max(0, min(rows - 1, sr))
        sc = max(0, min(cols - 1, sc))
        gr = max(0, min(rows - 1, gr))
        gc = max(0, min(cols - 1, gc))
        start_cell = (sr, sc)
        goal_cell  = (gr, gc)

        inflate_label = {0: 'Solo', 1: 'Fixed', 2: 'Pivot'}.get(self._mode, '?')
        self.get_logger().info(
            f'Non-Holonomic BFS: mode={inflate_label} inflate={inflate_m}m R_min={r_min}m '
            f'start=({robot_pose[0]:.2f}, {robot_pose[1]:.2f}, {math.degrees(robot_pose[2]):.1f}°) '
            f'goal=({self._goal.pose.position.x:.2f}, {self._goal.pose.position.y:.2f})')

        # ── Ukur waktu komputasi BFS + pipeline non-holonomik ──────────────────
        self._last_explored = 0
        t_plan0 = time.perf_counter()
        cell_path = self._bfs(inflated, start_cell, goal_cell, occupied, resolution)
        if cell_path is None:
            self.get_logger().warn('BFS: no path found')
            return

        # 1. Sel BFS → Titik koordinat dunia (world coordinates)
        world_pts  = [self._cell_to_world(r, c, origin, resolution) for r, c in cell_path]
        # 2. String pull: simplifikasi koridor bebas rintangan
        pulled_pts = self._string_pull(world_pts, inflated, origin, resolution)
        # Prune waypoint belokan dangkal / kolinear agar koridor lurus bebas dari chicane/kink
        pruned_pts = self._prune_collinear(pulled_pts, inflated, origin, resolution)
        # 3. Non-Holonomic Path Synthesis: tangen awal robot_pose[2] + fillet sudut R_min
        nh_pts     = self._build_nonholonomic_path(robot_pose, pruned_pts, r_min, inflated, occupied, origin, resolution)
        if len(nh_pts) < 2:
            self.get_logger().error(
                f'[PLANNER ALERT] Tidak dapat menghasilkan jalur aman untuk {inflate_label}. Manuver terhalang dinding!'
            )
            empty_path = Path()
            empty_path.header.stamp = self.get_clock().now().to_msg()
            empty_path.header.frame_id = 'map'
            self._pub_path.publish(empty_path)
            self._pub_ref_path.publish(empty_path)
            return
        # 4. Interpolasi jarak konstan (equidistant)
        interp_pts = self._interpolate_equidistant(nh_pts, PATH_STEP)
        smooth_pts = self._smooth(interp_pts, 3)
        t_plan_ms  = (time.perf_counter() - t_plan0) * 1000.0

        # ── Metrik path ─────────────────────────────────────────────────────────
        plen = sum(math.hypot(smooth_pts[i][0]-smooth_pts[i-1][0],
                              smooth_pts[i][1]-smooth_pts[i-1][1])
                   for i in range(1, len(smooth_pts)))
        dstraight = math.hypot(smooth_pts[-1][0]-smooth_pts[0][0],
                               smooth_pts[-1][1]-smooth_pts[0][1])
        self.get_logger().info(
            f'Non-Holonomic BFS DONE [{inflate_label}]: waktu={t_plan_ms:.1f}ms | '
            f'sel dieksplorasi={self._last_explored} | '
            f'titik: bfs={len(cell_path)}→pull={len(pulled_pts)}→'
            f'nh={len(nh_pts)}→interp={len(interp_pts)} | '
            f'panjang={plen:.2f}m (lurus={dstraight:.2f}m, rasio={plen/max(dstraight,0.01):.2f})')
        # Tulis metrik ke CSV (akumulasi tiap planning) → ~/smc_logs/bfs_metrics.csv
        self._log_bfs_csv(inflate_label, inflate_m, t_plan_ms, self._last_explored,
                          len(cell_path), len(pulled_pts), len(interp_pts), len(smooth_pts),
                          plen, dstraight)

        now = self.get_clock().now().to_msg()

        # ── Bangun pesan Path dengan orientasi tangensial non-holonomik ─────────
        path_msg = Path()
        path_msg.header.stamp    = now
        path_msg.header.frame_id = 'map'

        n_pts = len(smooth_pts)
        last_yaw = robot_pose[2]
        for i in range(n_pts):
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = smooth_pts[i][0]
            ps.pose.position.y = smooth_pts[i][1]

            if i < n_pts - 1:
                dx = smooth_pts[i + 1][0] - smooth_pts[i][0]
                dy = smooth_pts[i + 1][1] - smooth_pts[i][1]
                if math.hypot(dx, dy) > 1e-4:
                    last_yaw = math.atan2(dy, dx)
            elif self._goal is not None:
                # Titik akhir: gunakan orientasi goal jika ada, atau tangen segmen akhir
                g_q = self._goal.pose.orientation
                if abs(g_q.w**2 + g_q.x**2 + g_q.y**2 + g_q.z**2 - 1.0) < 0.1 and abs(g_q.w) + abs(g_q.z) > 0.1:
                    siny = 2.0 * (g_q.w * g_q.z + g_q.x * g_q.y)
                    cosy = 1.0 - 2.0 * (g_q.y * g_q.y + g_q.z * g_q.z)
                    last_yaw = math.atan2(siny, cosy)

            # Konversi heading yaw menjadi quaternion
            ps.pose.orientation.z = math.sin(last_yaw / 2.0)
            ps.pose.orientation.w = math.cos(last_yaw / 2.0)
            path_msg.poses.append(ps)

        self._pub_path.publish(path_msg)
        self._pub_ref_path.publish(path_msg)

        self.get_logger().info(
            f'Non-Holonomic Path published: {len(smooth_pts)} poses from {len(cell_path)} BFS cells')

        self._publish_markers(smooth_pts, now)

    # ── RViz markers ─────────────────────────────────────────────────────────

    def _publish_markers(self, pts: list[tuple[float, float]], stamp) -> None:
        m = Marker()
        m.header.stamp    = stamp
        m.header.frame_id = 'map'
        m.ns              = 'bfs_path'
        m.id              = 0
        m.type            = Marker.LINE_STRIP
        m.action          = Marker.ADD
        m.scale.x         = 0.05
        m.color.r         = 0.0
        m.color.g         = 0.85
        m.color.b         = 0.25
        m.color.a         = 1.0
        for x, y in pts:
            p = Point()
            p.x = x
            p.y = y
            m.points.append(p)

        arr = MarkerArray()
        arr.markers.append(m)
        self._pub_markers.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = BFSPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():           # cegah "rcl_shutdown already called" di Jazzy
            rclpy.shutdown()
