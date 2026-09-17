import os
import signal
import subprocess
import json
import rclpy
from rclpy.node import Node
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
from urllib.parse import urlparse
import mimetypes

# Global state to keep track of processes
processes = {
    'motor': None,
    'slam': None,
    'nav': None,
    'research': None
}

active_motion_mode = {
    'controller': 'native',
    'planner': 'native'
}

class RequestHandler(BaseHTTPRequestHandler):
    def get_src_dir(self):
        # Try source directory first for live editing (no rebuild needed)
        # Walk upward from installed __file__ to find the workspace root
        d = os.path.dirname(os.path.abspath(__file__))
        for _ in range(10):
            candidate = os.path.join(d, 'src', 'polebot_web_interface', 'www', 'NavDashboard')
            if os.path.exists(candidate):
                return candidate
            parent = os.path.dirname(d)
            if parent == d:
                break
            d = parent
        # Otherwise fallback to install share directory
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('polebot_web_interface'), 'www', 'NavDashboard')

    def _set_headers(self, status=200, content_type='application/json'):
        self.send_response(status)
        self.send_header('Content-type', content_type)
        self.send_header('Access-Control-Allow-Origin', '*')
        # Prevent browser caching so edits take effect immediately
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Expires', '0')
        self.end_headers()

    def do_GET(self):
        parsed_path = urlparse(self.path).path
        
        if parsed_path == '/api/status':
            self._set_headers()
            status = {
                'motor': processes['motor'] is not None and processes['motor'].poll() is None,
                'slam': processes['slam'] is not None and processes['slam'].poll() is None,
                'nav': processes['nav'] is not None and processes['nav'].poll() is None
            }
            self.wfile.write(json.dumps(status).encode('utf-8'))
            return
            
        if parsed_path == '/api/maps':
            self._set_headers()
            maps_dir = os.path.join(os.path.expanduser('~'), 'Desktop', 'AMR-POLEBOT-WS', 'maps')
            map_files = []
            if os.path.exists(maps_dir):
                for f in os.listdir(maps_dir):
                    if f.endswith('.yaml') and not f.endswith('_mask.yaml'):
                        map_files.append(f)
            self.wfile.write(json.dumps({'maps': sorted(map_files)}).encode('utf-8'))
            return

        if parsed_path == '/api/zones':
            self._set_headers()
            maps_dir = os.path.join(os.path.expanduser('~'), 'Desktop', 'AMR-POLEBOT-WS', 'maps')
            zones_file = os.path.join(maps_dir, 'zones.json')
            zones = []
            if os.path.exists(zones_file):
                try:
                    with open(zones_file, 'r') as f:
                        zones = json.load(f)
                except Exception:
                    zones = []
            self.wfile.write(json.dumps({'zones': zones}).encode('utf-8'))
            return

        if parsed_path == '/api/motion_modes':
            self._set_headers()
            data = {
                'active': active_motion_mode,
                'status': {
                    'research_running': processes['research'] is not None and processes['research'].poll() is None
                },
                'options': {
                    'controllers': [
                        {
                            'id': 'native',
                            'name': 'Native DWB (Standard OK)',
                            'badge': 'Standard OK',
                            'description': 'Standard Nav2 DWB Local Planner with gentle turns & single-backup goal termination',
                            'params': {'controller': 'DWBLocalPlanner', 'max_vel_x': '0.16 m/s', 'max_vel_theta': '0.22 rad/s', 'recovery': '1x BackUp → Abort'}
                        },
                        {
                            'id': 'pid',
                            'name': 'PID Profiled (S-Curve)',
                            'badge': 'S-Curve Gain Scheduling',
                            'description': 'PI(t)D(t)-Pure Pursuit with S-curve jerk-limited motion profiling',
                            'params': {'profile': 's_curve', 'v_max': '0.18 m/s', 'max_jerk': '0.16 m/s³', 'kp_cruise': '3.2', 'kp_lin': '0.35'}
                        },
                        {
                            'id': 'smc',
                            'name': 'SMC (Sliding Mode)',
                            'badge': 'Alipour Dynamic Inversion',
                            'description': 'Sliding Mode Controller on polar errors (rho, phi) with obstacle guard',
                            'params': {'formulation': 'Alipour et al. 2019', 'lambda1': '0.5', 'lambda2': '1.5', 'K1': '2.0', 'K2': '10.0'}
                        }
                    ],
                    'planners': [
                        {
                            'id': 'native',
                            'name': 'Native Nav2 (Costmap A*)',
                            'badge': 'Standard OK',
                            'description': 'Global costmap-based Navfn planner with keepout & speed filters',
                            'params': {'type': 'nav2_navfn_planner', 'use_astar': 'true', 'costmap': 'global_costmap'}
                        },
                        {
                            'id': 'bfs',
                            'name': 'BFS (Breadth-First Search)',
                            'badge': 'Uniform Wavefront',
                            'description': '8-connected BFS grid planner with string pulling & non-holonomic fillet curves',
                            'params': {'type': 'BFS 8-connected', 'step': '0.08 m', 'fillet_r_min': '0.25 m'}
                        },
                        {
                            'id': 'astar',
                            'name': 'A* Euclidean Optimal',
                            'badge': 'Optimal Clearance',
                            'description': 'A* shortest path search with distance-transform obstacle clearance penalty',
                            'params': {'type': 'A* Heuristic', 'clearance_weight': '0.08', 'step': '0.08 m'}
                        }
                    ]
                }
            }
            self.wfile.write(json.dumps(data).encode('utf-8'))
            return

        # Serve static files
        if parsed_path == '/api/robot_description':
            from ament_index_python.packages import get_package_share_directory
            desc_dir = get_package_share_directory('polebot_description')
            xacro_path = os.path.join(desc_dir, 'urdf', 'polebot.urdf.xacro')
            if os.path.exists(xacro_path):
                try:
                    result = subprocess.run(['xacro', xacro_path], capture_output=True, text=True, check=True)
                    self._set_headers(200, 'application/xml')
                    self.wfile.write(result.stdout.encode('utf-8'))
                except Exception as e:
                    self._set_headers(500, 'text/plain')
                    self.wfile.write(f"Error running xacro: {e}".encode('utf-8'))
            else:
                self._set_headers(404, 'text/plain')
                self.wfile.write(b"XACRO not found")
            return
            
        elif parsed_path.startswith('/models/polebot_description/'):
            from ament_index_python.packages import get_package_share_directory
            desc_dir = get_package_share_directory('polebot_description')
            rel_path = parsed_path[len('/models/polebot_description/'):]
            file_path = os.path.join(desc_dir, rel_path.lstrip('/'))
        else:
            base_dir = self.get_src_dir()
            if parsed_path == '/' or parsed_path == '/NavDashboard' or parsed_path == '/NavDashboard/':
                parsed_path = '/index.html'
            file_path = os.path.join(base_dir, parsed_path.lstrip('/'))
        
        if os.path.exists(file_path) and not os.path.isdir(file_path):
            mime_type, _ = mimetypes.guess_type(file_path)
            self._set_headers(200, mime_type or 'application/octet-stream')
            with open(file_path, 'rb') as f:
                self.wfile.write(f.read())
        else:
            self._set_headers(404, 'text/html')
            self.wfile.write(b"404 Not Found")

    def do_POST(self):
        parsed_path = urlparse(self.path).path
        response = {'status': 'success'}

        if parsed_path == '/api/launch/motor':
            if processes['motor'] is None or processes['motor'].poll() is not None:
                processes['motor'] = subprocess.Popen(
                    ['ros2', 'launch', 'polebot_bringup', 'polebot_motor.launch.py', 'auto_enable:=true'],
                    preexec_fn=os.setsid
                )
        elif parsed_path == '/api/kill/motor':
            if processes['motor'] is not None and processes['motor'].poll() is None:
                os.killpg(os.getpgid(processes['motor'].pid), signal.SIGINT)
                
        elif parsed_path == '/api/launch/slam':
            if processes['slam'] is None or processes['slam'].poll() is not None:
                processes['slam'] = subprocess.Popen(
                    ['ros2', 'launch', 'polebot_bringup', 'polebot_slam_bringup.launch.py', 'launch_lidar:=true'],
                    preexec_fn=os.setsid
                )
        elif parsed_path == '/api/kill/slam':
            if processes['slam'] is not None and processes['slam'].poll() is None:
                os.killpg(os.getpgid(processes['slam'].pid), signal.SIGINT)
                
        elif parsed_path == '/api/launch/nav':
            content_length = int(self.headers.get('Content-Length', 0))
            map_name = 'polebot_map.yaml'
            if content_length > 0:
                try:
                    body = self.rfile.read(content_length).decode('utf-8')
                    data = json.loads(body)
                    if 'map' in data and data['map']:
                        map_name = data['map']
                except Exception:
                    pass
            
            map_path = os.path.join(os.path.expanduser('~'), 'Desktop', 'AMR-POLEBOT-WS', 'maps', map_name)
            
            if processes['nav'] is None or processes['nav'].poll() is not None:
                processes['nav'] = subprocess.Popen(
                    ['ros2', 'launch', 'polebot_navigation', 'navigation.launch.py', f'map:={map_path}'],
                    preexec_fn=os.setsid
                )
        elif parsed_path == '/api/kill/nav':
            if processes['nav'] is not None and processes['nav'].poll() is None:
                os.killpg(os.getpgid(processes['nav'].pid), signal.SIGINT)
                
        elif parsed_path == '/api/save_map':
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length > 0:
                body = self.rfile.read(content_length).decode('utf-8')
                try:
                    data = json.loads(body)
                    map_name = data.get('name', 'polebot_map')
                    save_path = os.path.join(os.path.expanduser('~'), 'Desktop', 'AMR-POLEBOT-WS', 'maps', map_name)
                    # Run nav2_map_server map_saver_cli
                    subprocess.Popen(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', save_path])
                    response['message'] = f'Map saving initiated to {save_path}'
                except json.JSONDecodeError:
                    response = {'status': 'error', 'message': 'Invalid JSON'}
            else:
                response = {'status': 'error', 'message': 'Empty body'}
        
        elif parsed_path == '/api/save_zones':
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length > 0:
                body = self.rfile.read(content_length).decode('utf-8')
                try:
                    data = json.loads(body)
                    width = int(data.get('width', 0))
                    height = int(data.get('height', 0))
                    resolution = float(data.get('resolution', 0.05))
                    origin = data.get('origin_x', 0.0), data.get('origin_y', 0.0), 0.0
                    raw = data.get('data', [])

                    if width <= 0 or height <= 0 or len(raw) != width * height:
                        response = {'status': 'error', 'message': 'Invalid map dimensions or data size'}
                    else:
                        maps_dir = os.path.join(os.path.expanduser('~'), 'Desktop', 'AMR-POLEBOT-WS', 'maps')
                        os.makedirs(maps_dir, exist_ok=True)
                        zones_list = data.get('zones', None)
                        if zones_list is not None:
                            try:
                                with open(os.path.join(maps_dir, 'zones.json'), 'w') as f:
                                    json.dump(zones_list, f, indent=2)
                            except Exception:
                                pass
                        keepout_pgm = os.path.join(maps_dir, "keepout_mask.pgm")
                        keepout_yaml = os.path.join(maps_dir, "keepout_mask.yaml")
                        speed_pgm = os.path.join(maps_dir, "speed_mask.pgm")
                        speed_yaml = os.path.join(maps_dir, "speed_mask.yaml")

                        # Write P5 PGM binary image
                        pgm_header = f"P5\n{width} {height}\n255\n".encode('ascii')
                        keepout_bytes = bytearray(width * height)
                        speed_bytes = bytearray(width * height)
                        
                        for py in range(height):
                            gy = height - 1 - py
                            for gx in range(width):
                                ros_idx = gy * width + gx
                                pgm_idx = py * width + gx
                                v = raw[ros_idx]
                                
                                # Keepout (1 -> Black/0, 0/2 -> White/255)
                                if v == 1:
                                    keepout_bytes[pgm_idx] = 0
                                else:
                                    keepout_bytes[pgm_idx] = 255
                                    
                                # Speed limit (2 -> 50% speed (127 in pixel), 0/1 -> 100% speed (White/255))
                                if v == 2:
                                    # With base=100, multiplier=-1.0, we want mask_value = 50.
                                    # If negate: 0 and mode: scale, mask_value = (255 - p)/255 * 100
                                    # If we want mask_value=50 -> 50 = (255-p)/255*100 -> 0.5 = (255-p)/255 -> 127.5 = 255-p -> p = 127
                                    speed_bytes[pgm_idx] = 127
                                else:
                                    # 100% speed -> mask_value = 0 -> p = 255 (White)
                                    speed_bytes[pgm_idx] = 255
                        
                        with open(keepout_pgm, 'wb') as f:
                            f.write(pgm_header + keepout_bytes)
                        with open(speed_pgm, 'wb') as f:
                            f.write(pgm_header + speed_bytes)

                        ox, oy, oyaw = origin
                        
                        keepout_yaml_content = (
                            f"image: keepout_mask.pgm\n"
                            f"resolution: {resolution}\n"
                            f"origin: [{ox}, {oy}, {oyaw}]\n"
                            f"negate: 0\n"
                            f"occupied_thresh: 0.1\n"
                            f"free_thresh: 0.05\n"
                        )
                        with open(keepout_yaml, 'w') as f:
                            f.write(keepout_yaml_content)

                        speed_yaml_content = (
                            f"image: speed_mask.pgm\n"
                            f"resolution: {resolution}\n"
                            f"origin: [{ox}, {oy}, {oyaw}]\n"
                            f"negate: 0\n"
                            f"occupied_thresh: 1.0\n"
                            f"free_thresh: 0.0\n"
                            f"mode: scale\n"
                        )
                        with open(speed_yaml, 'w') as f:
                            f.write(speed_yaml_content)
                            
                        # Reload Keepout
                        subprocess.Popen(['ros2', 'service', 'call', '/keepout_mask_server/load_map', 'nav2_msgs/srv/LoadMap', f'{{map_url: "{keepout_yaml}"}}'])
                        # Reload Speed Limit
                        subprocess.Popen(['ros2', 'service', 'call', '/speed_mask_server/load_map', 'nav2_msgs/srv/LoadMap', f'{{map_url: "{speed_yaml}"}}'])

                        response['message'] = 'Zones saved and map servers reloaded'
                except Exception as e:
                    response = {'status': 'error', 'message': str(e)}
            else:
                response = {'status': 'error', 'message': 'Empty body'}

        elif parsed_path == '/api/save_edited_map':
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length > 0:
                body = self.rfile.read(content_length).decode('utf-8')
                try:
                    data = json.loads(body)
                    map_name = data.get('name', 'polebot_edited_map')
                    width = int(data.get('width', 0))
                    height = int(data.get('height', 0))
                    resolution = float(data.get('resolution', 0.05))
                    origin = data.get('origin', [0.0, 0.0, 0.0])
                    raw = data.get('data', [])

                    if width <= 0 or height <= 0 or len(raw) != width * height:
                        response = {'status': 'error', 'message': 'Invalid map dimensions or data size'}
                    else:
                        maps_dir = os.path.join(os.path.expanduser('~'), 'Desktop', 'AMR-POLEBOT-WS', 'maps')
                        os.makedirs(maps_dir, exist_ok=True)
                        pgm_path = os.path.join(maps_dir, f"{map_name}.pgm")
                        yaml_path = os.path.join(maps_dir, f"{map_name}.yaml")

                        # Write P5 PGM binary image (ROS format)
                        # ROS values: 0 -> 254 (free), 100 / >50 -> 0 (occupied), -1 / else -> 205 (unknown)
                        pgm_header = f"P5\n{width} {height}\n255\n".encode('ascii')
                        pgm_bytes = bytearray()
                        for py in range(height):
                            gy = height - 1 - py
                            for gx in range(width):
                                ros_idx = gy * width + gx
                                v = raw[ros_idx]
                                if v == 0:
                                    pgm_bytes.append(254)
                                elif v == 100 or v > 50:
                                    pgm_bytes.append(0)
                                else:
                                    pgm_bytes.append(205)
                        
                        with open(pgm_path, 'wb') as f:
                            f.write(pgm_header + pgm_bytes)

                        ox = origin[0] if len(origin) > 0 else 0.0
                        oy = origin[1] if len(origin) > 1 else 0.0
                        oyaw = origin[2] if len(origin) > 2 else 0.0
                        yaml_content = (
                            f"image: {map_name}.pgm\n"
                            f"resolution: {resolution}\n"
                            f"origin: [{ox}, {oy}, {oyaw}]\n"
                            f"negate: 0\n"
                            f"occupied_thresh: 0.65\n"
                            f"free_thresh: 0.25\n"
                        )
                        with open(yaml_path, 'w') as f:
                            f.write(yaml_content)

                        response['message'] = f'Edited map saved successfully as {map_name}.yaml'
                except Exception as e:
                    response = {'status': 'error', 'message': str(e)}
            else:
                response = {'status': 'error', 'message': 'Empty body'}

        elif parsed_path == '/api/set_motion_mode':
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length > 0:
                body = self.rfile.read(content_length).decode('utf-8')
                try:
                    data = json.loads(body)
                    controller = data.get('controller', active_motion_mode['controller']).lower()
                    planner = data.get('planner', active_motion_mode['planner']).lower()

                    # Stop existing research process if running
                    if processes.get('research') is not None and processes['research'].poll() is None:
                        try:
                            os.killpg(os.getpgid(processes['research'].pid), signal.SIGINT)
                            processes['research'].wait(timeout=2.0)
                        except Exception:
                            try:
                                os.killpg(os.getpgid(processes['research'].pid), signal.SIGKILL)
                            except Exception:
                                pass
                        processes['research'] = None

                    active_motion_mode['controller'] = controller
                    active_motion_mode['planner'] = planner

                    # If not native, launch research_control.launch.py with chosen args
                    if controller != 'native' or planner != 'native':
                        c_arg = controller if controller in ('pid', 'smc') else 'none'
                        p_arg = planner if planner in ('bfs', 'astar') else 'none'
                        cmd = [
                            'ros2', 'launch', 'polebot_research_control', 'research_control.launch.py',
                            f'controller:={c_arg}', f'planner:={p_arg}'
                        ]
                        processes['research'] = subprocess.Popen(cmd, preexec_fn=os.setsid)

                    response = {
                        'status': 'success',
                        'active': active_motion_mode,
                        'message': f"Switched to Controller: {controller.upper()}, Planner: {planner.upper()}"
                    }
                except Exception as e:
                    response = {'status': 'error', 'message': str(e)}
            else:
                response = {'status': 'error', 'message': 'Empty body'}
        else:
            self._set_headers(404)
            self.wfile.write(json.dumps({'status': 'error', 'message': 'Unknown endpoint'}).encode('utf-8'))
            return

        self._set_headers()
        self.wfile.write(json.dumps(response).encode('utf-8'))

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PoseStamped

class WebBackendNode(Node):
    def __init__(self):
        super().__init__('web_backend')
        self.server = ThreadingHTTPServer(('0.0.0.0', 5050), RequestHandler)
        self.get_logger().info('Backend server running on port 5050')
        # Log the actual directory being served
        handler = RequestHandler
        test_handler = handler.__new__(handler)
        served_dir = test_handler.get_src_dir()
        self.get_logger().info(f'Serving static files from: {served_dir}')
        
        self.server_thread = threading.Thread(target=self.server.serve_forever)
        self.server_thread.daemon = True
        self.server_thread.start()

        # TF2 Listener for stable robot pose to web UI
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(PoseStamped, '/robot_pose_web', 10)
        self.timer = self.create_timer(0.05, self.publish_pose) # 20Hz

    def publish_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            frame = 'map'
        except TransformException as e:
            # If the exception is about extrapolation, it means map exists but timestamps are temporarily out of sync.
            # We MUST NOT fallback to odom here, otherwise the robot teleports back and forth!
            if 'extrapolat' in str(e).lower():
                return
                
            try:
                t = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
                frame = 'odom'
            except TransformException:
                return
                
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame
        msg.pose.position.x = t.transform.translation.x
        msg.pose.position.y = t.transform.translation.y
        msg.pose.position.z = t.transform.translation.z
        msg.pose.orientation = t.transform.rotation
        self.pose_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WebBackendNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.shutdown()
        # Kill any child processes gracefully
        for key in processes:
            if processes[key] is not None and processes[key].poll() is None:
                os.killpg(os.getpgid(processes[key].pid), signal.SIGINT)
        try:
            node.destroy_node()
        except Exception:
            pass
            
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
