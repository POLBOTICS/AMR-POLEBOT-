"""Journal path planner — replikasi lintasan Fig.17 Alipour et al. 2019.

Bentuk path = ekor lurus → loop OVAL → ekor lurus, tetapi tetap dibangkitkan
dengan metode BFS: grid-search BFS dijalankan antar-waypoint berurutan lalu
hasilnya disambung, di-interpolasi equidistant, dan dihaluskan.

Waypoint (map frame, robot start = origin) menelusuri oval Fig.17:
  pusat oval ≈ (2.15, 0.82) m, radius ≈ 0.82 m, ditelusuri CCW dari titik bawah.

Publikasi:
  /planned_path   (nav_msgs/Path)        — untuk controller
  /bfs_markers    (visualization_msgs/MarkerArray) — RViz

Trigger: kirim apa saja ke /goal_pose (isi diabaikan) → node merencanakan oval.
"""
import math
from collections import deque

import numpy as np
from scipy.ndimage import binary_dilation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray

INFLATE_RADIUS = {0: 0.55, 1: 0.68, 2: 1.20}
PATH_STEP       = 0.05
SMOOTH_WINDOW   = 25     # ~1.25m: ratakan staircase BFS jadi kurva oval mulus
SMOOTH_PASSES   = 2      # smoothing diterapkan 2x untuk hasil lebih halus
OCCUPIED_THRESH = 65
DIRS_8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


# Oval diskalakan dari Fig.17 agar feasible utk tractor-trailer kita (L2=1.145m).
# Bentuk/proporsi sama dgn jurnal; ukuran diperbesar supaya sudut hitch wajar.
# Ada CELAH sudut di bawah oval → titik masuk (kanan) & keluar (kiri) terpisah,
# sehingga ekor masuk/keluar MENYILANG seperti Fig.17 (bukan garis tumpang tindih).
def oval_waypoints(cx=5.5, cy=6.0, r=6.0, n_oval=64, gap_deg=18.0):
    """Return (tail_in, oval, tail_out) — terskala Fig.17 dgn persimpangan menyilang."""
    gap = math.radians(gap_deg)
    a_in  = -math.pi / 2 + gap                     # titik masuk: kanan-bawah oval
    a_out = -math.pi / 2 - gap                     # titik keluar: kiri-bawah oval
    p_in  = (cx + r * math.cos(a_in),  cy + r * math.sin(a_in))
    p_out = (cx + r * math.cos(a_out), cy + r * math.sin(a_out))

    tail_in = [(0.0, 0.0), (p_in[0] / 2.0, p_in[1] / 2.0), p_in]

    span = 2 * math.pi - 2 * gap                   # oval CCW dari masuk → keluar
    oval = []
    for k in range(n_oval + 1):
        ang = a_in + span * k / n_oval
        oval.append((cx + r * math.cos(ang), cy + r * math.sin(ang)))

    tail_out = [p_out, (cx + 1.5, 0.0), (cx + 3.0, 0.0)]
    return tail_in, oval, tail_out


class JournalPathPlanner(Node):
    def __init__(self):
        super().__init__('journal_path_planner')

        # Skala oval bisa diatur lewat parameter ROS (default persis nilai
        # asli jurnal: cx=5.5, cy=6.0, r=6.0 -- butuh area ~12x12m yang
        # sudah ter-mapping SLAM). Untuk world tes kecil, override lebih
        # kecil lewat launch argument agar waypoint tidak "di luar map
        # bounds" saat peta belum banyak dieksplorasi.
        self.declare_parameter('oval_cx', 5.5)
        self.declare_parameter('oval_cy', 6.0)
        self.declare_parameter('oval_r', 6.0)
        cx = self.get_parameter('oval_cx').value
        cy = self.get_parameter('oval_cy').value
        r = self.get_parameter('oval_r').value

        map_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        self._sub_map  = self.create_subscription(OccupancyGrid, '/map', self._cb_map, map_qos)
        self._sub_goal = self.create_subscription(PoseStamped, '/goal_pose', self._cb_trigger, 10)
        self._sub_mode = self.create_subscription(Int32, '/towing_mode', self._cb_mode, 10)

        self._pub_path     = self.create_publisher(Path, '/planned_path', 10)
        self._pub_ref_path = self.create_publisher(Path, '/reference_path', 10)
        self._pub_markers  = self.create_publisher(MarkerArray, '/bfs_markers', 10)

        self._map: OccupancyGrid | None = None
        self._mode: int = 2
        self._tail_in, self._oval, self._tail_out = oval_waypoints(cx=cx, cy=cy, r=r)
        n = len(self._tail_in) + len(self._oval) + len(self._tail_out)
        self.get_logger().info(f'Journal oval: {n} waypoint siap '
                               f'(ekor {len(self._tail_in)}+{len(self._tail_out)}, '
                               f'oval {len(self._oval)}). Kirim /goal_pose utk merencanakan.')

    # ── Subscribers ───────────────────────────────────────────────────────────
    def _cb_map(self, msg: OccupancyGrid):
        self._map = msg

    def _cb_mode(self, msg: Int32):
        self._mode = msg.data

    def _cb_trigger(self, _msg: PoseStamped):
        if self._map is None:
            self.get_logger().warn('Belum ada /map — tunggu SLAM dulu.')
            return
        self._plan_oval()

    # ── Grid utilities (sama dgn bfs_planner) ──────────────────────────────────
    def _world_to_cell(self, wx, wy, origin, res):
        return int((wy - origin.y) / res), int((wx - origin.x) / res)

    def _cell_to_world(self, r, c, origin, res):
        return origin.x + (c + 0.5) * res, origin.y + (r + 0.5) * res

    def _inflate(self, grid, radius_m, res):
        rc = int(math.ceil(radius_m / res))
        yr, xr = np.ogrid[-rc:rc + 1, -rc:rc + 1]
        return binary_dilation(grid, structure=(xr ** 2 + yr ** 2 <= rc ** 2))

    def _bfs(self, inflated, start, goal):
        rows, cols = inflated.shape
        sr, sc = start; gr, gc = goal
        # geser goal bila di obstacle
        if inflated[gr, gc]:
            found = False
            for dr in range(-10, 11):
                for dc in range(-10, 11):
                    nr, nc = gr + dr, gc + dc
                    if 0 <= nr < rows and 0 <= nc < cols and not inflated[nr, nc]:
                        gr, gc = nr, nc; found = True; break
                if found: break
            if not found: return None
        if inflated[sr, sc]:
            return None
        q = deque([(sr, sc)]); parent = {(sr, sc): None}
        while q:
            r, c = q.popleft()
            if r == gr and c == gc:
                path = []; cur = (r, c)
                while cur is not None:
                    path.append(cur); cur = parent[cur]
                return path[::-1]
            for dr, dc in DIRS_8:
                nr, nc = r + dr, c + dc
                if (0 <= nr < rows and 0 <= nc < cols
                        and not inflated[nr, nc] and (nr, nc) not in parent):
                    parent[(nr, nc)] = (r, c); q.append((nr, nc))
        return None

    # ── Post-processing ────────────────────────────────────────────────────────
    def _interp(self, pts, step):
        if len(pts) < 2: return pts
        out = [pts[0]]; res = 0.0
        for i in range(1, len(pts)):
            p0 = out[-1]; p1 = pts[i]
            seg = math.hypot(p1[0]-p0[0], p1[1]-p0[1]); rem = seg
            while res + rem >= step and seg > 1e-9:
                t = (step - res) / rem
                nx = p0[0] + t*(p1[0]-p0[0]); ny = p0[1] + t*(p1[1]-p0[1])
                out.append((nx, ny)); rem -= (step - res); res = 0.0; p0 = (nx, ny)
            res += rem
        if math.hypot(out[-1][0]-pts[-1][0], out[-1][1]-pts[-1][1]) > 0.01:
            out.append(pts[-1])
        return out

    def _line_free(self, p1, p2, inflated, origin, res):
        dist = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
        steps = max(2, int(dist / (res * 0.5)))
        rows, cols = inflated.shape
        for k in range(steps + 1):
            t = k / steps
            wx = p1[0] + t*(p2[0]-p1[0]); wy = p1[1] + t*(p2[1]-p1[1])
            c = int((wx-origin.x)/res); r = int((wy-origin.y)/res)
            if not (0 <= r < rows and 0 <= c < cols): return False
            if inflated[r, c]: return False
        return True

    def _string_pull(self, pts, inflated, origin, res):
        """Kolaps zigzag BFS jadi garis lurus di area bebas obstacle (Sasaki et al.)."""
        if len(pts) < 3: return pts
        out = [pts[0]]; i = 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1:
                if self._line_free(pts[i], pts[j], inflated, origin, res): break
                j -= 1
            out.append(pts[j]); i = j
        return out

    def _smooth(self, pts, w):
        n = len(pts); hw = w // 2; out = []
        for i in range(n):
            lo, hi = max(0, i-hw), min(n, i+hw+1)
            xs = [pts[j][0] for j in range(lo, hi)]; ys = [pts[j][1] for j in range(lo, hi)]
            out.append((sum(xs)/len(xs), sum(ys)/len(ys)))
        # jaga titik awal & akhir tetap (jangan ketarik smoothing)
        out[0] = pts[0]; out[-1] = pts[-1]
        return out

    # ── Perencanaan oval via multi-waypoint BFS ────────────────────────────────
    def _plan_oval(self):
        m = self._map
        res = m.info.resolution; origin = m.info.origin.position
        rows, cols = m.info.height, m.info.width
        raw = np.array(m.data, dtype=np.int8).reshape(rows, cols)
        occupied = raw >= OCCUPIED_THRESH
        inflate_m = INFLATE_RADIUS.get(self._mode, INFLATE_RADIUS[2])
        inflated = self._inflate(occupied, inflate_m, res)

        # BFS per bagian: ekor & oval diproses terpisah (lihat helper di bawah)
        def bfs_segment(wps):
            cells = []
            for i in range(len(wps) - 1):
                a = self._world_to_cell(*wps[i],   origin, res)
                b = self._world_to_cell(*wps[i+1], origin, res)
                if not (0 <= a[0] < rows and 0 <= a[1] < cols and
                        0 <= b[0] < rows and 0 <= b[1] < cols):
                    self.get_logger().error(f'Waypoint ({wps[i]}) di luar map bounds')
                    return None
                seg = self._bfs(inflated, a, b)
                if seg is None:
                    self.get_logger().warn(f'BFS gagal {wps[i]}→{wps[i+1]}')
                    return None
                if cells and seg:
                    seg = seg[1:]
                cells += seg
            return [self._cell_to_world(r, c, origin, res) for r, c in cells]

        pin   = bfs_segment(self._tail_in)
        poval = bfs_segment(self._oval)
        pout  = bfs_segment(self._tail_out)
        if pin is None or poval is None or pout is None:
            return

        # Ekor: string-pull → garis lurus (hilangkan zigzag tie-break BFS), lalu interp.
        # Oval: interpolasi + smoothing berulang (ratakan staircase BFS).
        pin   = self._interp(self._string_pull(pin,  inflated, origin, res), PATH_STEP)
        pout  = self._interp(self._string_pull(pout, inflated, origin, res), PATH_STEP)
        poval = self._interp(poval, PATH_STEP)
        for _ in range(SMOOTH_PASSES):
            poval = self._smooth(poval, SMOOTH_WINDOW)

        # Sambung tanpa duplikat titik sambungan
        full = pin + poval[1:] + pout[1:]

        # Smoothing ringan pada path utuh → bulatkan kink di sambungan ekor↔oval.
        # Window kecil: ekor tetap lurus, oval tetap, & tak mencampur titik silang
        # (dua lintasan di titik bawah terpisah ~ratusan indeks, di luar window).
        for _ in range(2):
            full = self._smooth(full, 11)

        now = self.get_clock().now().to_msg()
        path = Path(); path.header.stamp = now; path.header.frame_id = 'map'
        for x, y in full:
            ps = PoseStamped(); ps.header = path.header
            ps.pose.position.x = x; ps.pose.position.y = y; ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self._pub_path.publish(path)
        self._pub_ref_path.publish(path)
        self.get_logger().info(
            f'Oval path published: {len(full)} pts (ekor {len(pin)}+{len(pout)}, '
            f'oval {len(poval)})')
        self._markers(full, now)

    def _markers(self, pts, stamp):
        m = Marker(); m.header.stamp = stamp; m.header.frame_id = 'map'
        m.ns = 'journal_path'; m.id = 0; m.type = Marker.LINE_STRIP; m.action = Marker.ADD
        m.scale.x = 0.04; m.color.g = 0.85; m.color.b = 0.2; m.color.a = 1.0
        for x, y in pts:
            p = Point(); p.x = x; p.y = y; m.points.append(p)
        arr = MarkerArray(); arr.markers.append(m); self._pub_markers.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = JournalPathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
