// ═══════════════════════════════════════════════════════
//  POLEBOT Navigation Dashboard
//  Vanilla JS + Three.js + ROSBridge
//  Preserves the operator-verified v7 map/model/LiDAR alignment.
// ═══════════════════════════════════════════════════════
console.log('[POLEBOT] Navigation workspace loaded; calibrated alignment preserved.');

const ROSBRIDGE_HOST = window.location.hostname;
const ROSBRIDGE_PORT = 9090;

let ws = null;
let wsConnected = false;
let nav2Ready = false;

// Map state
let mapData = null;       // OccupancyGrid data
let mapInfo = null;       // metadata {resolution, origin, width, height}
let robotPose = null;     // {x, y, yaw}
let goalPose = null;      // {x, y}
let pendingGoal = null;   // goal waiting to be sent
let laserScan = null;     // sensor_msgs/msg/LaserScan

// 3D Viewer State
let viewer3D = null;
let scene3D = null;
let camera3D = null;
let renderer3D = null;
let controls3D = null;
let robotMesh3D = null;
let mapMesh3D = null;
let scanPoints3D = null;
let animFrame3D = null;

// Canvas pan/zoom
let interactMode = 'view';

const wrap = document.getElementById('map-canvas-wrap');

// ── WebSocket Connection ──────────────────────────────

function connect() {
    const url = `${location.protocol === 'https:' ? 'wss' : 'ws'}://${ROSBRIDGE_HOST}:${ROSBRIDGE_PORT}`;
    ws = new WebSocket(url);

    ws.onopen = function () {
        wsConnected = true;
        document.getElementById('ws-badge').className = 'badge badge-connected';
        document.getElementById('ws-label').textContent = 'Connected';

        subscribeTopics();
        init3DViewer();
        if (currentTab === 'nav') checkNav2Status();
        else checkSlamStatus();
    };

    ws.onclose = () => {
        wsConnected = false;
        setBadge(false);
        setAllNodesOffline();
        rejectPendingServices();
        if (activeActionId) {
            navigationUncertain = true;
            activeActionId = null;
            pendingGoal = null;
            setNavigationState('UNKNOWN', 'Connection lost; awaiting Nav2 status after reconnection.', 'status-checking');
            showNotice('Connection lost during navigation. Check Nav2 status before sending another goal.');
        }
        document.getElementById('btn-cancel').disabled = true;
        setTimeout(connect, 3000); // auto-reconnect
    };

    ws.onerror = () => {
        wsConnected = false;
        setBadge(false);
    };

    ws.onmessage = (event) => {
        try {
            const msg = JSON.parse(event.data);
            handleMessage(msg);
        } catch (e) { console.warn('ROS message could not be processed:', e); }
    };
}

function send(obj) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify(obj));
    }
}

function setBadge(connected) {
    const badge = document.getElementById('ws-badge');
    const label = document.getElementById('ws-label');
    badge.className = 'badge ' + (connected ? 'badge-connected' : 'badge-disconnected');
    label.textContent = connected ? 'Connected' : 'Disconnected';
}

// ── Topic Subscriptions ───────────────────────────────

function subscribeTopics() {
    // Map
    send({ op: 'subscribe', topic: '/map', type: 'nav_msgs/msg/OccupancyGrid', throttle_rate: 1000 });
    // Stable robot pose from web_backend
    send({ op: 'subscribe', topic: '/robot_pose_web', type: 'geometry_msgs/msg/PoseStamped', throttle_rate: 50 });
    // Laser scan for LiDAR visualization
    send({ op: 'subscribe', topic: '/scan', type: 'sensor_msgs/msg/LaserScan', throttle_rate: 200 });
    // Navigation action status
    send({ op: 'subscribe', topic: '/navigate_to_pose/_action/status', type: 'action_msgs/msg/GoalStatusArray', throttle_rate: 500 });
    // Odom for diagnostics
    send({ op: 'subscribe', topic: '/odom', type: 'nav_msgs/msg/Odometry', throttle_rate: 200 });

    // Costmaps
    send({ op: 'subscribe', topic: '/global_costmap/costmap', type: 'nav_msgs/msg/OccupancyGrid', throttle_rate: 1000 });
    send({ op: 'subscribe', topic: '/local_costmap/costmap', type: 'nav_msgs/msg/OccupancyGrid', throttle_rate: 200 });

    send({ op: 'subscribe', topic: '/plan', type: 'nav_msgs/msg/Path', throttle_rate: 250 });
    send({ op: 'subscribe', topic: '/planned_path', type: 'nav_msgs/msg/Path', throttle_rate: 250 });
    send({ op: 'advertise', topic: '/initialpose', type: 'geometry_msgs/msg/PoseWithCovarianceStamped' });
    for (const name of ['global', 'local']) send({ op: 'subscribe', topic: `/${name}_costmap/costmap_updates`, type: 'map_msgs/msg/OccupancyGridUpdate', throttle_rate: 0 });
    send({ op: 'subscribe', topic: '/tf', type: 'tf2_msgs/msg/TFMessage', throttle_rate: 0 });
    send({ op: 'subscribe', topic: '/tf_static', type: 'tf2_msgs/msg/TFMessage', throttle_rate: 0 });
    // Teleop
    send({ op: 'advertise', topic: '/cmd_vel', type: 'geometry_msgs/msg/Twist' });
}

let topicHz = { map: 0, tf: 0, scan: 0, odom: 0 };
let topicCounters = { map: 0, tf: 0, scan: 0, odom: 0 };

setInterval(() => {
    topicHz = { ...topicCounters };
    topicCounters = { map: 0, tf: 0, scan: 0, odom: 0 };

    if (currentTab === 'system') {
        const updateHz = (id, val) => {
            const el = document.getElementById(id);
            if (el) {
                el.textContent = `${val} Hz`;
                el.style.color = val > 0 ? 'var(--green)' : 'var(--red)';
            }
        };
        updateHz('hz-map', topicHz.map);
        updateHz('hz-tf', topicHz.tf);
        updateHz('hz-scan', topicHz.scan);
        updateHz('hz-odom', topicHz.odom);
    }
}, 1000);

function handleMessage(msg) {
    if (handleWorkspaceMessage(msg)) return;
    if (!msg.topic) return;
    topicLastSeen[msg.topic] = Date.now();

    if (msg.topic === '/tf' || msg.topic === '/tf_static') {
        receiveCostmapTransforms(msg.msg, msg.topic === '/tf_static');
    } else if (msg.topic === '/map') {
        topicCounters.map++;
        onMapReceived(msg.msg);
    } else if (msg.topic === '/robot_pose_web') {
        topicCounters.tf++;
        onPoseReceived(msg.msg);
    } else if (msg.topic === '/scan') {
        topicCounters.scan++;
        laserScan = msg.msg;
    } else if (msg.topic === '/odom') {
        topicCounters.odom++;
    } else if (msg.topic === '/navigate_to_pose/_action/status') {
        onNavStatusReceived(msg.msg);
    } else if (msg.topic.endsWith('_costmap/costmap_updates')) {
        updateCostmapRegion(msg.topic.startsWith('/global') ? 'global' : 'local', msg.msg);
    } else if (msg.topic === '/plan' || msg.topic === '/planned_path') {
        drawPath(msg.msg);
    } else if (msg.topic === '/global_costmap/costmap') {
        cachedCostmaps.global = msg.msg;
        if (showGlobalCostmap) {
            globalCostmapMesh3D = processCostmapToMesh(msg.msg, globalCostmapMesh3D, 'global');
        }
    } else if (msg.topic === '/local_costmap/costmap') {
        cachedCostmaps.local = msg.msg;
        if (showLocalCostmap) {
            localCostmapMesh3D = processCostmapToMesh(msg.msg, localCostmapMesh3D, 'local');
        }
    }
    applyLayerVisibility();
}

// Smoothing parameter (EMA) for jitter reduction
const POSE_ALPHA = 1.0; // Set to 1.0 for true real-time AMCL/TF matching without lag

// ── Pose / Status Updates ─────────────────────────────

function onPoseReceived(data) {
    robotPoseFrame = data.header?.frame_id?.replace(/^\//, '') || '';
    const pos = data.pose.position || data.pose.pose.position; // handle both PoseStamped and PoseWithCovarianceStamped
    const quat = data.pose.orientation || data.pose.pose.orientation;
    const yaw = quaternionToYaw(quat);

    // Apply EMA smoothing
    if (!robotPose) {
        robotPose = { x: pos.x, y: pos.y, yaw: yaw };
    } else {
        robotPose.x = robotPose.x + POSE_ALPHA * (pos.x - robotPose.x);
        robotPose.y = robotPose.y + POSE_ALPHA * (pos.y - robotPose.y);
        let dyaw = yaw - robotPose.yaw;
        while (dyaw > Math.PI) dyaw -= 2 * Math.PI;
        while (dyaw < -Math.PI) dyaw += 2 * Math.PI;
        robotPose.yaw = robotPose.yaw + POSE_ALPHA * dyaw;
    }

    const elX = document.getElementById('pose-x');
    if (elX) {
        elX.textContent = robotPose.x.toFixed(2);
        document.getElementById('pose-y').textContent = robotPose.y.toFixed(2);
        document.getElementById('pose-yaw').textContent = (robotPose.yaw * 180 / Math.PI).toFixed(1);
    }


    update3DRobotPose();
}

// ── Nav2 Status Check ─────────────────────────────────

const NAV2_TOPICS = {
    'node-map_server': '/map',
    'node-amcl': '/amcl_pose',
    'node-bt_navigator': '/navigate_to_pose/_action/status',
    'node-planner': '/plan',
    'node-controller': '/cmd_vel',
};

async function checkNav2Status() {
    if (!wsConnected) { setAllNodesOffline(); return; }
    if (checkingLifecycle) return;
    checkingLifecycle = true;
    const nodes = {
        'node-map_server': 'map_server', 'node-amcl': 'amcl',
        'node-bt_navigator': 'bt_navigator', 'node-planner': 'planner_server', 'node-controller': 'controller_server'
    };
    try {
        // Query active ROS services first to avoid logging InvalidServiceException in rosbridge console when Nav2 is offline
        let availableServices = [];
        try {
            const res = await rosService('/rosapi/services', {});
            availableServices = res.services || [];
        } catch (e) { }

        await Promise.all(Object.entries(nodes).map(async ([id, name]) => {
            const serviceName = `/${name}/get_state`;
            if (availableServices.length > 0 && !availableServices.includes(serviceName)) {
                lifecycleStates[name] = false;
                setNodeStatus(id, 'offline');
                const stEl = document.querySelector(`#${id} .node-status`);
                if (stEl) stEl.textContent = 'Inactive';
                return;
            }

            try {
                const values = await rosService(serviceName, {});
                lifecycleStates[name] = values.current_state?.id === 3;
                setNodeStatus(id, lifecycleStates[name] ? 'online' : 'checking');
                const stEl = document.querySelector(`#${id} .node-status`);
                if (stEl) stEl.textContent = values.current_state?.label || 'Unknown';
            } catch (error) {
                lifecycleStates[name] = false;
                setNodeStatus(id, 'offline');
                const stEl = document.querySelector(`#${id} .node-status`);
                if (stEl) stEl.textContent = 'Unavailable';
            }
        }));
        nav2Ready = Object.values(nodes).every(name => lifecycleStates[name]);
        if (lifecycleStates.map_server) isNavMapLoaded = true;
        updateNavReadyUI();
    } finally { checkingLifecycle = false; }
}

function setNodeStatus(id, status) {
    const el = document.getElementById(id);
    if (!el) return;
    const badge = el.querySelector('.node-status');
    badge.className = 'node-status';
    if (status === 'online') {
        badge.classList.add('status-online');
        badge.textContent = 'Online';
    } else if (status === 'offline') {
        badge.classList.add('status-offline');
        badge.textContent = 'Offline';
    } else {
        badge.classList.add('status-checking');
        badge.textContent = 'Checking';
    }
}

function setAllNodesOffline() {
    Object.keys(NAV2_TOPICS).forEach(id => setNodeStatus(id, 'offline'));
    nav2Ready = false;
    updateNavReadyUI();
}

let isNavMapLoaded = false;

let showGlobalCostmap = false;
let showLocalCostmap = false;
let globalCostmapMesh3D = null;
let localCostmapMesh3D = null;

function toggleGlobalCostmap() {
    showGlobalCostmap = document.getElementById('cb-global-costmap').checked;
    if (showGlobalCostmap && cachedCostmaps.global && scene3D)
        globalCostmapMesh3D = processCostmapToMesh(cachedCostmaps.global, globalCostmapMesh3D, 'global');
    applyLayerVisibility();
}
function toggleLocalCostmap() {
    showLocalCostmap = document.getElementById('cb-local-costmap').checked;
    if (showLocalCostmap && cachedCostmaps.local && scene3D)
        localCostmapMesh3D = processCostmapToMesh(cachedCostmaps.local, localCostmapMesh3D, 'local');
    applyLayerVisibility();
}

async function loadSelectedMap() {
    const mapName = document.getElementById('map-dropdown').value;
    if (!mapName) {
        alert("Please select a map first.");
        return;
    }

    document.getElementById('overlay-title').textContent = "Starting Nav2...";
    document.getElementById('overlay-sub').textContent = `Loading map: ${mapName}`;
    document.getElementById('map-select-ui').style.display = 'none';

    try {
        const response = await fetch('/api/launch/nav', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ map: mapName })
        });
        const result = await response.json();
        if (!response.ok || result.status !== 'success') throw new Error(result.message || 'Nav2 launch failed');
        isNavMapLoaded = true;
        updateNavReadyUI();
        // Force refresh of Nav2 status
        setTimeout(checkNav2Status, 1000);
    } catch (e) {
        showNotice("Failed to launch Nav2: " + e.message);
        updateNavReadyUI();
    }
}

function updateNavReadyUI() {
    const overlay = document.getElementById('map-overlay');
    const overlayTitle = document.getElementById('overlay-title');
    const overlaySub = document.getElementById('overlay-sub');
    const btnGoal = document.getElementById('btn-send-goal');
    const mapSelectUI = document.getElementById('map-select-ui');
    const viewer3D = document.getElementById('viewer-3d');

    if (!wsConnected) {
        overlayTitle.textContent = 'WebSocket Disconnected';
        overlaySub.textContent = `Connecting to ws://${ROSBRIDGE_HOST}:${ROSBRIDGE_PORT}...`;
        if (mapSelectUI) mapSelectUI.style.display = 'none';
        overlay.style.display = 'flex';
        if (viewer3D) viewer3D.style.display = 'none';
    } else if (currentTab === 'nav' && !isNavMapLoaded) {
        overlayTitle.textContent = 'Select Map to Load';
        overlaySub.textContent = 'Choose a static map to begin AMCL and Navigation';
        if (mapSelectUI) mapSelectUI.style.display = 'flex';
        overlay.style.display = 'flex';
        if (viewer3D) viewer3D.style.display = 'none';
    } else if (currentTab === 'slam' && (!slamReady || !mapData)) {
        overlayTitle.textContent = 'Waiting for SLAM';
        overlaySub.textContent = slamReady ? 'SLAM Toolbox running. Waiting for map data...' : 'Start SLAM Toolbox to begin mapping';
        if (mapSelectUI) mapSelectUI.style.display = 'none';
        overlay.style.display = 'flex';
        if (viewer3D) viewer3D.style.display = 'none';
    } else if (!mapData) {
        overlayTitle.textContent = 'Waiting for Map';
        overlaySub.textContent = 'Waiting for map data to arrive...';
        if (mapSelectUI) mapSelectUI.style.display = 'none';
        overlay.style.display = 'flex';
        if (viewer3D) viewer3D.style.display = 'none';
    } else {
        overlay.style.display = 'none';
        if (viewer3D) viewer3D.style.display = 'block';
    }

    if (btnGoal) btnGoal.disabled = !wsConnected || !nav2Ready || !pendingGoal || !!activeActionId || navigationUncertain || currentTab !== 'nav';
    document.getElementById('btn-send-init').disabled = !wsConnected || !pendingInitPose || !lifecycleStates.amcl || currentTab !== 'nav';
    for (const id of ['btn-init-mode', 'btn-goal-mode']) document.getElementById(id).disabled = !wsConnected || !mapData || currentTab !== 'nav';
}

// ── SLAM Status Check & Map Service ───────────────────

async function checkSlamStatus() {
    slamReady = false;
    try {
        const result = await rosService('/rosapi/nodes', {});
        slamReady = (result.nodes || []).some(name => {
            const basename = name.split('/').pop();
            return basename === 'slam_toolbox' || basename === 'async_slam_toolbox_node' || basename.includes('slam_toolbox');
        });
    } catch (error) { /* Unavailable is distinct from a map topic being present. */ }
    setNodeStatus('node-slam_toolbox', slamReady ? 'online' : 'offline');
    document.getElementById('btn-save-map').disabled = !slamReady;
    document.getElementById('btn-serialize-map').disabled = !slamReady;
    updateNavReadyUI();
}

function saveMap() {
    if (!wsConnected) return;
    const mapName = document.getElementById('map-name-input').value.trim() || 'polebot_map';

    const hintText = document.querySelector('#panel-slam .goal-hint');
    hintText.textContent = `Saving map to ${mapName}...`;
    hintText.style.color = 'var(--cyan)';

    fetch('/api/save_map', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ name: mapName })
    }).then(res => res.json()).then(data => {
        hintText.textContent = '✅ Map saved successfully to workspace!';
        hintText.style.color = 'var(--green)';
        setTimeout(() => {
            hintText.textContent = 'Save the map to the workspace directory on the robot.';
            hintText.style.color = 'var(--text-secondary)';
        }, 3000);
    }).catch(err => {
        hintText.textContent = '❌ Failed to save map.';
        hintText.style.color = 'var(--red)';
    });
}

function fetchMapList() {
    fetch('/api/maps')
        .then(res => res.json())
        .then(data => {
            const dropdown = document.getElementById('map-dropdown');
            if (dropdown && data.maps) {
                dropdown.innerHTML = '';
                if (data.maps.length === 0) {
                    dropdown.innerHTML = '<option value="">No maps found</option>';
                    return;
                }
                data.maps.forEach(map => {
                    const opt = document.createElement('option');
                    opt.value = map;
                    opt.textContent = map;
                    dropdown.appendChild(opt);
                });
            }
        })
        .catch(err => console.error("Failed to fetch map list", err));
}

if (typeof window !== 'undefined' && window?.addEventListener) {
    window.addEventListener('DOMContentLoaded', () => {
        fetchMapList();
        if (typeof fetchMotionModes === 'function') fetchMotionModes();
    });
}

function init3DViewer() {
    if (viewer3D) return;
    const wrapEl = document.getElementById('map-canvas-wrap');
    const w = wrapEl.clientWidth;
    const h = wrapEl.clientHeight;

    // Scene
    scene3D = new THREE.Scene();
    scene3D.background = new THREE.Color('#080c10');

    // Camera (top-down perspective)
    camera3D = new THREE.PerspectiveCamera(60, w / h, 0.1, 1000);
    camera3D.position.set(0, 0, 15);
    camera3D.lookAt(0, 0, 0);

    // Renderer
    renderer3D = new THREE.WebGLRenderer({ antialias: true });
    renderer3D.setSize(w, h);
    renderer3D.setPixelRatio(window.devicePixelRatio);
    const container = document.getElementById('viewer-3d');
    container.innerHTML = '';
    container.appendChild(renderer3D.domElement);

    // Orbit Controls
    controls3D = new THREE.OrbitControls(camera3D, renderer3D.domElement);
    controls3D.enableDamping = true;
    controls3D.dampingFactor = 0.1;
    controls3D.maxPolarAngle = Math.PI / 2;

    // Grid helper
    const grid = new THREE.GridHelper(50, 50, 0x333333, 0x222222);
    grid.rotation.x = Math.PI / 2; // ROS XY plane
    scene3D.add(grid);
    gridMesh3D = grid;
    for (const material of (Array.isArray(grid.material) ? grid.material : [grid.material])) {
        material.transparent = true; material.opacity = 0.28; material.depthWrite = false;
    }

    // Ambient + directional light
    scene3D.add(new THREE.AmbientLight(0xffffff, 0.6));
    const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
    dirLight.position.set(5, 5, 10);
    scene3D.add(dirLight);

    // Robot model group
    const robotGroup = new THREE.Group();
    robotMesh3D = robotGroup;
    scene3D.add(robotMesh3D);

    // Arrow cone for direction (keep this to clearly see heading)
    const arrowGeo = new THREE.ConeGeometry(0.12, 0.3, 8);
    const arrowMat = new THREE.MeshPhongMaterial({ color: 0x00ffff });
    const arrow = new THREE.Mesh(arrowGeo, arrowMat);
    arrow.rotation.z = -Math.PI / 2; // point along X axis (forward)
    arrow.position.set(0.35, 0, 0.4); // float above the robot
    robotGroup.add(arrow);

    // Load 3D STL Model
    const loader = new THREE.STLLoader();
    const stlUrl = '/models/polebot_description/meshes/polebot_amr.stl';

    loader.load(
        stlUrl,
        function (geometry) {
            const material = new THREE.MeshStandardMaterial({
                color: 0x8899aa,
                metalness: 0.3,
                roughness: 0.6
            });
            const mesh = new THREE.Mesh(geometry, material);

            // Adjust scale (STL is likely exported in millimeters, scale down to meters). 
            // Halved from 0.001 to 0.0005 to better match the map scale based on user feedback.
            mesh.scale.set(0.0005, 0.0005, 0.0005);

            // The Web UI 3D scene uses the exact same coordinate system as ROS
            // (X = forward, Y = left, Z = up). We do NOT need to rotate the mesh!
            mesh.rotation.set(0, 0, 0);

            robotGroup.add(mesh);
            console.log("STL Model loaded successfully");
        },
        function (xhr) {
            console.log((xhr.loaded / xhr.total * 100) + '% loaded');
        },
        function (error) {
            console.error('Failed to load STL model:', error);
            // Fallback to simple box if STL fails
            const bodyGeo = new THREE.BoxGeometry(0.5, 0.4, 0.2);
            const bodyMat = new THREE.MeshPhongMaterial({ color: 0x00d8ff, transparent: true, opacity: 0.85 });
            const body = new THREE.Mesh(bodyGeo, bodyMat);
            body.position.z = 0.1;
            robotGroup.add(body);
        }
    );

    viewer3D = true;
    installViewportControls();
    animate3D();
}

function animate3D() {
    if (!viewer3D) return;
    animFrame3D = requestAnimationFrame(animate3D);
    updateFollowCamera();
    controls3D.update();

    // Update map plane from 2D data
    update3DMap();
    update3DScan();

    // Render Vector SVG Zone Overlay
    renderZoneOverlay();

    applyLayerVisibility();
    renderer3D.render(scene3D, camera3D);
}

function update3DScan() {
    if (!viewer3D || !laserScan || !robotMesh3D) return;

    if (!scanPoints3D) {
        const geo = new THREE.BufferGeometry();
        // Allocate max possible points for typical lidars
        const maxPts = Math.max(2000, laserScan.ranges.length);
        const positions = new Float32Array(maxPts * 3);
        geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        const mat = new THREE.PointsMaterial({ color: 0xff0000, size: 0.08 });
        scanPoints3D = new THREE.Points(geo, mat);
        scanPoints3D.position.z = 0.2; // Float above the ground/robot slightly
        robotMesh3D.add(scanPoints3D);
    }

    const positions = scanPoints3D.geometry.attributes.position.array;
    let pIdx = 0;

    for (let i = 0; i < laserScan.ranges.length; i++) {
        const r = laserScan.ranges[i];
        if (r < laserScan.range_min || r > laserScan.range_max || !isFinite(r)) continue;

        const angle = laserScan.angle_min + i * laserScan.angle_increment;

        // Cartesian in laser frame
        const lx = r * Math.cos(angle);
        const ly = r * Math.sin(angle);

        // User requested opposite of the previous guess.
        // Previous guess: -ly, lx
        // New guess: ly, -lx (270 degree rotation)

        positions[pIdx++] = ly + 0.496;
        positions[pIdx++] = -lx;
        positions[pIdx++] = 0;
    }

    scanPoints3D.geometry.setDrawRange(0, pIdx / 3);
    scanPoints3D.geometry.attributes.position.needsUpdate = true;
}

function update3DMap() {
    if (!mapData || !mapData.bitmap || !mapInfo) return;
    // Only rebuild when map changes
    if (mapMesh3D && mapMesh3D.userData.mapVersion === mapData.version) return;

    if (mapMesh3D) disposeMesh(mapMesh3D);

    // Render map bitmap to a canvas texture
    const texCanvas = document.createElement('canvas');
    texCanvas.width = mapData.w;
    texCanvas.height = mapData.h;
    const texCtx = texCanvas.getContext('2d');
    texCtx.drawImage(mapData.bitmap, 0, 0);
    const texture = new THREE.CanvasTexture(texCanvas);
    texture.minFilter = THREE.NearestFilter;
    texture.magFilter = THREE.NearestFilter;

    // By default, Three.js CanvasTexture has flipY = true.
    texture.flipY = true;

    const worldW = mapData.w * mapInfo.resolution;
    const worldH = mapData.h * mapInfo.resolution;
    const planeGeo = new THREE.PlaneGeometry(worldW, worldH);
    const planeMat = new THREE.MeshBasicMaterial({ map: texture, transparent: true, opacity: 0.9, side: THREE.DoubleSide });
    mapMesh3D = new THREE.Mesh(planeGeo, planeMat);

    // Position: map origin is bottom-left corner
    const ox = mapInfo.origin.position.x;
    const oy = mapInfo.origin.position.y;
    // Position map so its origin matches the physical world
    mapMesh3D.position.set(ox + worldW / 2, oy + worldH / 2, -0.01);
    mapMesh3D.userData.mapVersion = mapData.version;
    scene3D.add(mapMesh3D);
    if (!hasFittedMap) { resetView(); hasFittedMap = true; }
}

function update3DRobotPose() {
    if (!robotMesh3D || !robotPose) return;
    robotMesh3D.position.set(robotPose.x, robotPose.y, 0);
    robotMesh3D.rotation.z = robotPose.yaw;
}


function serializeMap() {
    if (!wsConnected) return;
    const mapName = document.getElementById('map-name-input').value.trim() || 'polebot_map';

    const hintText = document.querySelector('#panel-slam .goal-hint');
    hintText.textContent = `Serializing map to ${mapName}...`;
    hintText.style.color = 'var(--orange)';

    send({
        op: 'call_service',
        id: 'serialize_map_call',
        service: '/slam_toolbox/serialize_map',
        args: {
            filename: `/home/mirae/Desktop/AMR-POLEBOT-WS/maps/${mapName}`
        }
    });

    ws.addEventListener('message', function handler(event) {
        try {
            const data = JSON.parse(event.data);
            if (data.op === 'service_response' && data.id === 'serialize_map_call') {
                ws.removeEventListener('message', handler);
                hintText.textContent = '✅ Map serialized successfully!';
                hintText.style.color = 'var(--green)';
                setTimeout(() => {
                    hintText.textContent = 'Save the map to the workspace directory on the robot.';
                    hintText.style.color = 'var(--text-secondary)';
                }, 3000);
            }
        } catch (e) { }
    });
}

// ── Map Editor State (GIMP Tools) ──────────────────────
let mapEditVal = 100;         // 100: Wall, 0: Free, -1: Unknown
let mapEditShape = 'pencil';   // 'pencil', 'line', 'rect'
let mapEditBrushSize = 5;      // Brush size in pixels
let editUndoStack = [];
let editRedoStack = [];
let originalMapData = null;    // Int8Array copy of raw unedited map

// --- Adobe-Style Vector Zone Editor State ---
let zones = []; // Array of { id, name, type: 'keepout'|'speed', speedLimit: 0.5, x, y, w, h, visible: true }
let selectedZoneId = null;
let zoneTool = 'keepout'; // 'keepout', 'speed', 'select'
let zoneOverlayVisible = true;
let isDrawingZone = false;
let zoneDrawStart = null; // { wx, wy }
let zoneDrawCurrent = null; // { wx, wy }
let zoneDragState = null; // { mode: 'move'|'resize', handle, startWx, startWy, origX, origY, origW, origH }
let zoneMesh3D = null;

function setZoneTool(tool) {
    zoneTool = tool;
    const btnKeepout = document.getElementById('btn-tool-keepout');
    const btnSpeed = document.getElementById('btn-tool-speed');
    const btnSelect = document.getElementById('btn-tool-select');
    if (btnKeepout) btnKeepout.classList.toggle('active', tool === 'keepout');
    if (btnSpeed) btnSpeed.classList.toggle('active', tool === 'speed');
    if (btnSelect) btnSelect.classList.toggle('active', tool === 'select');

    if (interactMode !== 'edit_zone') {
        setMode('edit_zone');
    }
    const badge = document.getElementById('zone-mode-badge');
    if (badge) {
        badge.textContent = tool === 'keepout' ? 'Keepout Tool' : (tool === 'speed' ? 'Speed Tool' : 'Select Tool');
        badge.style.background = tool === 'keepout' ? 'rgba(239,68,68,0.25)' : (tool === 'speed' ? 'rgba(245,158,11,0.25)' : 'rgba(88,166,255,0.25)');
        badge.style.color = tool === 'keepout' ? '#fca5a5' : (tool === 'speed' ? '#fde68a' : '#58a6ff');
    }
    renderZoneOverlay();
}

function toggleZoneOverlayVisibility(visible) {
    zoneOverlayVisible = visible;
    const svg = document.getElementById('zone-svg-overlay');
    if (svg) svg.style.display = visible ? 'block' : 'none';
    if (zoneMesh3D) zoneMesh3D.visible = visible;
    renderZoneOverlay();
}

function worldToScreen(wx, wy, wz = 0) {
    if (!renderer3D || !camera3D) return null;
    const rect = renderer3D.domElement.getBoundingClientRect();
    if (!rect || rect.width === 0 || rect.height === 0) return null;
    const v = new THREE.Vector3(wx, wy, wz);
    v.project(camera3D);
    if (v.z > 1) return null; // Behind camera
    const sx = (v.x * 0.5 + 0.5) * rect.width;
    const sy = (-(v.y * 0.5) + 0.5) * rect.height;
    return { x: sx, y: sy };
}

function screenToWorld(screenX, screenY) {
    if (!renderer3D || !camera3D) return null;
    const rect = renderer3D.domElement.getBoundingClientRect();
    if (!rect || rect.width === 0 || rect.height === 0) return null;
    const ndc = new THREE.Vector2(
        (screenX / rect.width) * 2 - 1,
        -(screenY / rect.height) * 2 + 1
    );
    const ray = new THREE.Raycaster();
    ray.setFromCamera(ndc, camera3D);
    const plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
    const target = new THREE.Vector3();
    const hit = ray.ray.intersectPlane(plane, target);
    return hit || null;
}

let isMapEditing = false;
let activeEditChanges = new Map(); // rosIdx -> { rosIdx, gx, gy, oldVal, newVal }
let editStartCell = null;      // { gx, gy, x, y } for line/rect drawing
let editPreviewMesh = null;    // Preview line or box while dragging
let lastPencilCell = null;

function setMapEditVal(val) {
    mapEditVal = val;
    document.getElementById('btn-edit-val-wall')?.classList.toggle('active', val === 100);
    document.getElementById('btn-edit-val-free')?.classList.toggle('active', val === 0);
    document.getElementById('btn-edit-val-unknown')?.classList.toggle('active', val === -1);
}

function setMapEditShape(shape) {
    mapEditShape = shape;
    document.getElementById('btn-edit-shape-pencil')?.classList.toggle('active', shape === 'pencil');
    document.getElementById('btn-edit-shape-line')?.classList.toggle('active', shape === 'line');
    document.getElementById('btn-edit-shape-rect')?.classList.toggle('active', shape === 'rect');
}

function updateEditUndoRedoButtons() {
    const btnUndo = document.getElementById('btn-edit-undo');
    const btnRedo = document.getElementById('btn-edit-redo');
    if (btnUndo) btnUndo.disabled = editUndoStack.length === 0;
    if (btnRedo) btnRedo.disabled = editRedoStack.length === 0;
}

function setMapCellPixel(gx, gy, val) {
    if (!mapData || !mapData.bitmap) return;
    const w = mapData.w;
    const h = mapData.h;
    if (gx < 0 || gx >= w || gy < 0 || gy >= h) return;

    const rosIdx = gy * w + gx;
    mapData.raw[rosIdx] = val;

    // Update 2D canvas texture pixel directly
    const ctx = mapData.bitmap.getContext('2d');
    const imgData = ctx.createImageData(1, 1);
    let r, g, b;
    if (val === -1) { r = 120; g = 130; b = 140; }       // Unknown — blue-grey
    else if (val === 0) { r = 240; g = 245; b = 250; }    // Free — near white
    else { r = 30; g = 35; b = 45; }                    // Occupied — dark
    imgData.data[0] = r; imgData.data[1] = g; imgData.data[2] = b; imgData.data[3] = 255;
    ctx.putImageData(imgData, gx, h - 1 - gy);
}

function paintBrushAt(gx, gy) {
    if (!mapData || !mapInfo) return;
    const w = mapData.w;
    const h = mapData.h;
    const r = (mapEditBrushSize - 1) / 2;
    const r2 = r * r;

    const minX = Math.max(0, Math.floor(gx - r));
    const maxX = Math.min(w - 1, Math.ceil(gx + r));
    const minY = Math.max(0, Math.floor(gy - r));
    const maxY = Math.min(h - 1, Math.ceil(gy + r));

    for (let y = minY; y <= maxY; y++) {
        for (let x = minX; x <= maxX; x++) {
            if ((x - gx) * (x - gx) + (y - gy) * (y - gy) <= r2 + 0.25) {
                const rosIdx = y * w + x;
                const oldVal = mapData.raw[rosIdx];
                if (oldVal !== mapEditVal) {
                    if (!activeEditChanges.has(rosIdx)) {
                        activeEditChanges.set(rosIdx, { rosIdx, gx: x, gy: y, oldVal, newVal: mapEditVal });
                    }
                    setMapCellPixel(x, y, mapEditVal);
                }
            }
        }
    }
    if (mapMesh3D?.material?.map) mapMesh3D.material.map.needsUpdate = true;
}

function paintLineBetween(gx0, gy0, gx1, gy1) {
    let dx = Math.abs(gx1 - gx0);
    let dy = Math.abs(gy1 - gy0);
    let sx = gx0 < gx1 ? 1 : -1;
    let sy = gy0 < gy1 ? 1 : -1;
    let err = dx - dy;

    let x = gx0;
    let y = gy0;

    while (true) {
        paintBrushAt(x, y);
        if (x === gx1 && y === gy1) break;
        let e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x += sx; }
        if (e2 < dx) { err += dx; y += sy; }
    }
}

function paintBoxBetween(gx0, gy0, gx1, gy1) {
    const minX = Math.min(gx0, gx1);
    const maxX = Math.max(gx0, gx1);
    const minY = Math.min(gy0, gy1);
    const maxY = Math.max(gy0, gy1);

    for (let y = minY; y <= maxY; y++) {
        for (let x = minX; x <= maxX; x++) {
            const rosIdx = y * mapData.w + x;
            const oldVal = mapData.raw[rosIdx];
            if (oldVal !== mapEditVal) {
                if (!activeEditChanges.has(rosIdx)) {
                    activeEditChanges.set(rosIdx, { rosIdx, gx: x, gy: y, oldVal, newVal: mapEditVal });
                }
                setMapCellPixel(x, y, mapEditVal);
            }
        }
    }
    if (mapMesh3D?.material?.map) mapMesh3D.material.map.needsUpdate = true;
}


function renderZoneOverlay() {
    const svg = document.getElementById('zone-svg-overlay');
    if (!svg || !camera3D || !renderer3D) return;

    const gPolys = document.getElementById('zone-svg-polygons');
    const gBadges = document.getElementById('zone-svg-badges');
    const gHandles = document.getElementById('zone-svg-handles');
    const gPreview = document.getElementById('zone-svg-preview');

    if (!gPolys || !gBadges || !gHandles || !gPreview) return;

    gPolys.innerHTML = '';
    gBadges.innerHTML = '';
    gHandles.innerHTML = '';
    gPreview.innerHTML = '';

    if (!zoneOverlayVisible) return;

    for (const zone of zones) {
        if (!zone.visible) continue;

        const s0 = worldToScreen(zone.x, zone.y);
        const s1 = worldToScreen(zone.x + zone.w, zone.y);
        const s2 = worldToScreen(zone.x + zone.w, zone.y + zone.h);
        const s3 = worldToScreen(zone.x, zone.y + zone.h);

        if (!s0 || !s1 || !s2 || !s3) continue;

        const pts = `${s0.x.toFixed(1)},${s0.y.toFixed(1)} ${s1.x.toFixed(1)},${s1.y.toFixed(1)} ${s2.x.toFixed(1)},${s2.y.toFixed(1)} ${s3.x.toFixed(1)},${s3.y.toFixed(1)}`;
        const isSelected = zone.id === selectedZoneId;
        const isKeepout = zone.type === 'keepout';

        // 1. Polygon Fill & Border
        const poly = document.createElementNS('http://www.w3.org/2000/svg', 'polygon');
        poly.setAttribute('points', pts);
        poly.setAttribute('class', 'zone-svg-poly' + (isSelected ? ' selected' : ''));
        poly.setAttribute('fill', isKeepout ? 'url(#keepout-stripes)' : 'url(#speed-stripes)');
        poly.setAttribute('stroke', isKeepout ? '#ef4444' : '#f59e0b');
        poly.setAttribute('stroke-width', isSelected ? '3.5' : '2.5');
        poly.setAttribute('filter', isKeepout ? 'url(#glow-red)' : 'url(#glow-amber)');
        poly.setAttribute('opacity', '0.9');
        gPolys.appendChild(poly);

        if (isSelected) {
            // Extra Adobe bounding outline
            const selOutline = document.createElementNS('http://www.w3.org/2000/svg', 'polygon');
            selOutline.setAttribute('points', pts);
            selOutline.setAttribute('fill', 'none');
            selOutline.setAttribute('stroke', '#00a8ff');
            selOutline.setAttribute('stroke-width', '2');
            selOutline.setAttribute('stroke-dasharray', '5,3');
            gPolys.appendChild(selOutline);
        }

        // 2. Centered Floating Pill Badge
        const sc = worldToScreen(zone.x + zone.w / 2, zone.y + zone.h / 2);
        if (sc) {
            const badgeG = document.createElementNS('http://www.w3.org/2000/svg', 'g');
            badgeG.setAttribute('class', 'zone-svg-badge');
            badgeG.setAttribute('filter', 'url(#badge-shadow)');

            const labelText = isKeepout ? '🚫 KEEPOUT' : `⚠️ SPEED LIMIT (${Math.round((zone.speedLimit || 0.5) * 100)}%)`;
            const dimText = `${zone.w.toFixed(2)}m × ${zone.h.toFixed(2)}m`;

            const bWidth = Math.max(120, labelText.length * 7 + 24);
            const bHeight = 34;
            const bx = sc.x - bWidth / 2;
            const by = sc.y - bHeight / 2;

            const rect = document.createElementNS('http://www.w3.org/2000/svg', 'rect');
            rect.setAttribute('x', bx.toFixed(1));
            rect.setAttribute('y', by.toFixed(1));
            rect.setAttribute('width', bWidth);
            rect.setAttribute('height', bHeight);
            rect.setAttribute('rx', '7');
            rect.setAttribute('fill', 'rgba(13, 17, 23, 0.92)');
            rect.setAttribute('stroke', isKeepout ? '#ef4444' : '#f59e0b');
            rect.setAttribute('stroke-width', '1.5');
            badgeG.appendChild(rect);

            const text1 = document.createElementNS('http://www.w3.org/2000/svg', 'text');
            text1.setAttribute('x', sc.x.toFixed(1));
            text1.setAttribute('y', (by + 14).toFixed(1));
            text1.setAttribute('text-anchor', 'middle');
            text1.setAttribute('fill', isKeepout ? '#fca5a5' : '#fde68a');
            text1.setAttribute('font-size', '10px');
            text1.setAttribute('font-weight', '700');
            text1.setAttribute('font-family', 'Inter, sans-serif');
            text1.textContent = labelText;
            badgeG.appendChild(text1);

            const text2 = document.createElementNS('http://www.w3.org/2000/svg', 'text');
            text2.setAttribute('x', sc.x.toFixed(1));
            text2.setAttribute('y', (by + 27).toFixed(1));
            text2.setAttribute('text-anchor', 'middle');
            text2.setAttribute('fill', '#8b949e');
            text2.setAttribute('font-size', '9px');
            text2.setAttribute('font-family', "'JetBrains Mono', monospace");
            text2.textContent = dimText;
            badgeG.appendChild(text2);

            gBadges.appendChild(badgeG);
        }

        // 3. Adobe 8 Transform Handles (if selected and in edit_zone mode)
        if (isSelected && interactMode === 'edit_zone') {
            const handles = [
                { name: 'nw', wx: zone.x, wy: zone.y + zone.h },
                { name: 'ne', wx: zone.x + zone.w, wy: zone.y + zone.h },
                { name: 'se', wx: zone.x + zone.w, wy: zone.y },
                { name: 'sw', wx: zone.x, wy: zone.y },
                { name: 'n', wx: zone.x + zone.w / 2, wy: zone.y + zone.h },
                { name: 's', wx: zone.x + zone.w / 2, wy: zone.y },
                { name: 'e', wx: zone.x + zone.w, wy: zone.y + zone.h / 2 },
                { name: 'w', wx: zone.x, wy: zone.y + zone.h / 2 },
            ];

            for (const h of handles) {
                const sp = worldToScreen(h.wx, h.wy);
                if (!sp) continue;
                const hRect = document.createElementNS('http://www.w3.org/2000/svg', 'rect');
                hRect.setAttribute('class', 'zone-svg-handle');
                hRect.setAttribute('data-handle', h.name);
                hRect.setAttribute('x', (sp.x - 4.5).toFixed(1));
                hRect.setAttribute('y', (sp.y - 4.5).toFixed(1));
                hRect.setAttribute('width', '9');
                hRect.setAttribute('height', '9');
                gHandles.appendChild(hRect);
            }

            // Dimension tool tag attached above top edge
            const topMid = worldToScreen(zone.x + zone.w / 2, zone.y + zone.h);
            if (topMid) {
                const dimTag = document.createElementNS('http://www.w3.org/2000/svg', 'g');
                dimTag.setAttribute('class', 'zone-svg-badge');
                const tagText = `📏 ${zone.w.toFixed(2)}m × ${zone.h.toFixed(2)}m`;
                const tagW = tagText.length * 6.8 + 14;
                const tagH = 20;

                const tr = document.createElementNS('http://www.w3.org/2000/svg', 'rect');
                tr.setAttribute('x', (topMid.x - tagW / 2).toFixed(1));
                tr.setAttribute('y', (topMid.y - 26).toFixed(1));
                tr.setAttribute('width', tagW);
                tr.setAttribute('height', tagH);
                tr.setAttribute('rx', '4');
                tr.setAttribute('fill', '#00a8ff');
                dimTag.appendChild(tr);

                const tt = document.createElementNS('http://www.w3.org/2000/svg', 'text');
                tt.setAttribute('x', topMid.x.toFixed(1));
                tt.setAttribute('y', (topMid.y - 12).toFixed(1));
                tt.setAttribute('text-anchor', 'middle');
                tt.setAttribute('fill', '#ffffff');
                tt.setAttribute('font-size', '10px');
                tt.setAttribute('font-weight', '700');
                tt.setAttribute('font-family', 'Inter, sans-serif');
                tt.textContent = tagText;
                dimTag.appendChild(tt);

                gHandles.appendChild(dimTag);
            }
        }
    }

    // 4. Live Drawing Preview Rectangle
    if (isDrawingZone && zoneDrawStart && zoneDrawCurrent) {
        const minX = Math.min(zoneDrawStart.wx, zoneDrawCurrent.wx);
        const maxX = Math.max(zoneDrawStart.wx, zoneDrawCurrent.wx);
        const minY = Math.min(zoneDrawStart.wy, zoneDrawCurrent.wy);
        const maxY = Math.max(zoneDrawStart.wy, zoneDrawCurrent.wy);

        const s0 = worldToScreen(minX, minY);
        const s1 = worldToScreen(maxX, minY);
        const s2 = worldToScreen(maxX, maxY);
        const s3 = worldToScreen(minX, maxY);

        if (s0 && s1 && s2 && s3) {
            const pts = `${s0.x.toFixed(1)},${s0.y.toFixed(1)} ${s1.x.toFixed(1)},${s1.y.toFixed(1)} ${s2.x.toFixed(1)},${s2.y.toFixed(1)} ${s3.x.toFixed(1)},${s3.y.toFixed(1)}`;
            const isKeepout = zoneTool === 'keepout';

            const poly = document.createElementNS('http://www.w3.org/2000/svg', 'polygon');
            poly.setAttribute('points', pts);
            poly.setAttribute('fill', isKeepout ? 'url(#keepout-stripes)' : 'url(#speed-stripes)');
            poly.setAttribute('stroke', isKeepout ? '#ef4444' : '#f59e0b');
            poly.setAttribute('stroke-width', '2.5');
            poly.setAttribute('stroke-dasharray', '6,3');
            poly.setAttribute('opacity', '0.85');
            gPreview.appendChild(poly);

            // Floating dimension tooltip near cursor
            const sc = worldToScreen((minX + maxX) / 2, (minY + maxY) / 2);
            if (sc) {
                const tipG = document.createElementNS('http://www.w3.org/2000/svg', 'g');
                tipG.setAttribute('class', 'zone-svg-badge');
                const text = `${(maxX - minX).toFixed(2)}m × ${(maxY - minY).toFixed(2)}m`;
                const tw = text.length * 7 + 16;
                const th = 22;

                const tr = document.createElementNS('http://www.w3.org/2000/svg', 'rect');
                tr.setAttribute('x', (sc.x - tw / 2).toFixed(1));
                tr.setAttribute('y', (sc.y - th / 2).toFixed(1));
                tr.setAttribute('width', tw);
                tr.setAttribute('height', th);
                tr.setAttribute('rx', '5');
                tr.setAttribute('fill', 'rgba(13, 17, 23, 0.95)');
                tr.setAttribute('stroke', isKeepout ? '#ef4444' : '#f59e0b');
                tr.setAttribute('stroke-width', '1.5');
                tipG.appendChild(tr);

                const tt = document.createElementNS('http://www.w3.org/2000/svg', 'text');
                tt.setAttribute('x', sc.x.toFixed(1));
                tt.setAttribute('y', (sc.y + 4).toFixed(1));
                tt.setAttribute('text-anchor', 'middle');
                tt.setAttribute('fill', '#ffffff');
                tt.setAttribute('font-size', '10px');
                tt.setAttribute('font-weight', '700');
                tt.setAttribute('font-family', "'JetBrains Mono', monospace");
                tt.textContent = text;
                tipG.appendChild(tt);

                gPreview.appendChild(tipG);
            }
        }
    }
}

function updateZoneUI() {
    const inspector = document.getElementById('zone-inspector');
    const selected = zones.find(z => z.id === selectedZoneId);

    if (inspector) {
        if (selected) {
            inspector.style.display = 'block';
            const nameInput = document.getElementById('zone-prop-name');
            if (nameInput) nameInput.value = selected.name || '';
            const typeSelect = document.getElementById('zone-prop-type');
            if (typeSelect) typeSelect.value = selected.type;
            const speedField = document.getElementById('zone-speed-field');
            if (speedField) speedField.style.display = selected.type === 'speed' ? 'flex' : 'none';
            const speedSelect = document.getElementById('zone-prop-speed');
            if (speedSelect) speedSelect.value = String(selected.speedLimit || 0.5);

            const wInput = document.getElementById('zone-prop-w');
            if (wInput) wInput.value = selected.w.toFixed(2);
            const hInput = document.getElementById('zone-prop-h');
            if (hInput) hInput.value = selected.h.toFixed(2);
            const xInput = document.getElementById('zone-prop-x');
            if (xInput) xInput.value = selected.x.toFixed(2);
            const yInput = document.getElementById('zone-prop-y');
            if (yInput) yInput.value = selected.y.toFixed(2);
        } else {
            inspector.style.display = 'none';
        }
    }

    // Active zones list
    const countEl = document.getElementById('zone-count');
    if (countEl) countEl.textContent = zones.length;

    const listEl = document.getElementById('zone-list-items');
    if (listEl) {
        if (zones.length === 0) {
            listEl.innerHTML = '<div class="empty-zones-hint">No zones defined.<br>Drag on the map to draw a rectangle!</div>';
        } else {
            listEl.innerHTML = '';
            zones.forEach(z => {
                const card = document.createElement('div');
                card.className = `zone-item-card type-${z.type}${z.id === selectedZoneId ? ' selected' : ''}`;
                card.onclick = () => { selectZone(z.id); };

                const icon = z.type === 'keepout' ? '🚫' : '⚠️';
                const sub = `${z.w.toFixed(2)}m × ${z.h.toFixed(2)}m · (${z.x.toFixed(1)}, ${z.y.toFixed(1)})`;

                card.innerHTML = `
                    <span style="font-size: 13px;">${icon}</span>
                    <div class="zone-item-info">
                        <div class="zone-item-title">${z.name || (z.type === 'keepout' ? 'Keepout' : 'Speed Limit')}</div>
                        <div class="zone-item-sub">${sub}</div>
                    </div>
                    <button class="zone-item-btn" title="Toggle visibility" onclick="event.stopPropagation(); toggleZoneItemVisibility('${z.id}')">
                        ${z.visible ? '👁️' : '🕶️'}
                    </button>
                    <button class="zone-item-btn" style="color:#ff7b72;" title="Delete zone" onclick="event.stopPropagation(); deleteZoneById('${z.id}')">
                        🗑️
                    </button>
                `;
                listEl.appendChild(card);
            });
        }
    }
    renderZoneOverlay();
}

function updateZoneInspectorInputs(z) {
    const wInput = document.getElementById('zone-prop-w');
    if (wInput) wInput.value = z.w.toFixed(2);
    const hInput = document.getElementById('zone-prop-h');
    if (hInput) hInput.value = z.h.toFixed(2);
    const xInput = document.getElementById('zone-prop-x');
    if (xInput) xInput.value = z.x.toFixed(2);
    const yInput = document.getElementById('zone-prop-y');
    if (yInput) yInput.value = z.y.toFixed(2);
}

function updateSelectedZoneProp(prop, val) {
    const z = zones.find(item => item.id === selectedZoneId);
    if (!z) return;
    if (prop === 'name') z.name = String(val);
    else if (prop === 'type') {
        z.type = val;
        const sf = document.getElementById('zone-speed-field');
        if (sf) sf.style.display = val === 'speed' ? 'flex' : 'none';
    } else if (prop === 'speedLimit') z.speedLimit = Number(val);
    else if (prop === 'w' && Number.isFinite(val) && val > 0.05) z.w = val;
    else if (prop === 'h' && Number.isFinite(val) && val > 0.05) z.h = val;
    else if (prop === 'x' && Number.isFinite(val)) z.x = val;
    else if (prop === 'y' && Number.isFinite(val)) z.y = val;

    updateZoneUI();
    update3DZoneMesh();
}

function selectZone(id) {
    selectedZoneId = id;
    updateZoneUI();
}

function deleteSelectedZone() {
    if (!selectedZoneId) return;
    deleteZoneById(selectedZoneId);
}

function deleteZoneById(id) {
    zones = zones.filter(z => z.id !== id);
    if (selectedZoneId === id) selectedZoneId = null;
    updateZoneUI();
    update3DZoneMesh();
    showWorkspaceMessage('Zone deleted');
}

function clearAllZones() {
    if (zones.length === 0) return;
    zones = [];
    selectedZoneId = null;
    updateZoneUI();
    update3DZoneMesh();
    showWorkspaceMessage('All zones cleared');
}

function toggleZoneItemVisibility(id) {
    const z = zones.find(item => item.id === id);
    if (z) {
        z.visible = !z.visible;
        updateZoneUI();
        update3DZoneMesh();
    }
}

function rasterizeZonesToGrid() {
    if (!mapData || !mapInfo) return null;
    const w = mapData.w;
    const h = mapData.h;
    const res = mapInfo.resolution;
    const ox = mapInfo.origin.position.x;
    const oy = mapInfo.origin.position.y;
    const raw = new Int8Array(w * h); // 0 = free

    for (const z of zones) {
        if (!z.visible) continue;
        const val = z.type === 'keepout' ? 1 : 2;
        const minGx = Math.max(0, Math.floor((z.x - ox) / res));
        const maxGx = Math.min(w - 1, Math.ceil((z.x + z.w - ox) / res));
        const minGy = Math.max(0, Math.floor((z.y - oy) / res));
        const maxGy = Math.min(h - 1, Math.ceil((z.y + z.h - oy) / res));

        for (let gy = minGy; gy <= maxGy; gy++) {
            for (let gx = minGx; gx <= maxGx; gx++) {
                const idx = gy * w + gx;
                if (val === 1 || raw[idx] === 0) {
                    raw[idx] = val;
                }
            }
        }
    }
    return raw;
}

function update3DZoneMesh() {
    if (!mapData || !mapInfo || !scene3D) return;
    const w = mapData.w;
    const h = mapData.h;
    const res = mapInfo.resolution;
    const ox = mapInfo.origin.position.x;
    const oy = mapInfo.origin.position.y;

    if (!zoneMesh3D) {
        const off = document.createElement('canvas');
        off.width = w; off.height = h;
        const tex = new THREE.CanvasTexture(off);
        tex.magFilter = THREE.NearestFilter;
        tex.minFilter = THREE.NearestFilter;
        const geo = new THREE.PlaneGeometry(w * res, h * res);
        const mat = new THREE.MeshBasicMaterial({ map: tex, transparent: true, opacity: 0.7, depthWrite: false, side: THREE.DoubleSide });
        zoneMesh3D = new THREE.Mesh(geo, mat);
        zoneMesh3D.position.set(ox + (w * res) / 2, oy + (h * res) / 2, 0.015);
        scene3D.add(zoneMesh3D);
    }

    const canvas = zoneMesh3D.material.map.image;
    if (!canvas || canvas.width !== w || canvas.height !== h) {
        canvas.width = w; canvas.height = h;
    }
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, w, h);

    for (const z of zones) {
        if (!z.visible) continue;
        const minGx = Math.max(0, Math.floor((z.x - ox) / res));
        const maxGx = Math.min(w - 1, Math.ceil((z.x + z.w - ox) / res));
        const minGy = Math.max(0, Math.floor((z.y - oy) / res));
        const maxGy = Math.min(h - 1, Math.ceil((z.y + z.h - oy) / res));

        const rx = minGx;
        const ry = h - 1 - maxGy;
        const rw = Math.max(1, maxGx - minGx + 1);
        const rh = Math.max(1, maxGy - minGy + 1);

        ctx.fillStyle = z.type === 'keepout' ? 'rgba(239, 68, 68, 0.45)' : 'rgba(245, 158, 11, 0.45)';
        ctx.fillRect(rx, ry, rw, rh);
    }
    zoneMesh3D.material.map.needsUpdate = true;
}

async function fetchZones() {
    try {
        const res = await fetch('/api/zones');
        if (!res.ok) return;
        const data = await res.json();
        if (data && Array.isArray(data.zones)) {
            zones = data.zones;
            updateZoneUI();
            update3DZoneMesh();
        }
    } catch (e) {
        // Fallback gracefully
    }
}

async function saveZones() {
    if (!mapData || !mapInfo) {
        const status = document.getElementById('zone-edit-status');
        if (status) status.textContent = 'Please load a map first.';
        return;
    }

    const btn = document.querySelector('#section-zone-editor .btn-primary');
    if (btn) {
        btn.disabled = true;
        btn.textContent = '💾 Saving...';
    }

    const rawGrid = rasterizeZonesToGrid();
    const rawArray = rawGrid ? Array.from(rawGrid) : [];

    try {
        const res = await fetch('/api/save_zones', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                width: mapData.w,
                height: mapData.h,
                resolution: mapInfo.resolution,
                origin_x: mapInfo.origin.position.x,
                origin_y: mapInfo.origin.position.y,
                data: rawArray,
                zones: zones
            })
        });
        const data = await res.json();
        const status = document.getElementById('zone-edit-status');
        if (data.status === 'success' || data.status === 'ok') {
            if (status) status.textContent = '✅ Zones saved and applied to Nav2!';
            showWorkspaceMessage('Zones successfully saved & applied to Nav2 costmaps!');
            setTimeout(() => {
                if (status) status.textContent = 'Ready to draw zones';
            }, 4000);
        } else {
            if (status) status.textContent = 'Error: ' + (data.message || data.error);
        }
    } catch (e) {
        const status = document.getElementById('zone-edit-status');
        if (status) status.textContent = 'Network error saving zones.';
    }
    if (btn) {
        btn.disabled = false;
        btn.textContent = '💾 Save Zones & Apply to Nav2';
    }
}

function startMapEditing(gx, gy, x, y) {
    if (!mapData) return;
    isMapEditing = true;
    activeEditChanges.clear();
    editStartCell = { gx, gy, x, y };

    if (mapEditShape === 'pencil') {
        lastPencilCell = { gx, gy };
        paintBrushAt(gx, gy);
    }
}

function continueMapEditing(gx, gy, x, y) {
    if (!isMapEditing || !mapData) return;

    if (mapEditShape === 'pencil') {
        if (lastPencilCell) {
            paintLineBetween(lastPencilCell.gx, lastPencilCell.gy, gx, gy);
        } else {
            paintBrushAt(gx, gy);
        }
        lastPencilCell = { gx, gy };
    } else if (mapEditShape === 'line' || mapEditShape === 'rect') {
        updateEditPreview(editStartCell, { gx, gy, x, y });
    }
}

function finishMapEditing(gx, gy, x, y) {
    if (!isMapEditing || !mapData) return;
    isMapEditing = false;
    clearEditPreview();

    if (mapEditShape === 'line' && editStartCell) {
        paintLineBetween(editStartCell.gx, editStartCell.gy, gx, gy);
    } else if (mapEditShape === 'rect' && editStartCell) {
        paintBoxBetween(editStartCell.gx, editStartCell.gy, gx, gy);
    }
    lastPencilCell = null;
    editStartCell = null;

    if (activeEditChanges.size > 0) {
        const patch = Array.from(activeEditChanges.values());
        editUndoStack.push(patch);
        if (editUndoStack.length > 50) editUndoStack.shift();
        editRedoStack = [];
        updateEditUndoRedoButtons();
        const statusEl = document.getElementById('edit-map-status');
        if (statusEl) statusEl.textContent = `Edited (${activeEditChanges.size} cells modified)`;
    }
}

function updateEditPreview(start, end) {
    clearEditPreview();
    if (!scene3D || !start || !end) return;

    if (mapEditShape === 'line') {
        const points = [new THREE.Vector3(start.x, start.y, 0.02), new THREE.Vector3(end.x, end.y, 0.02)];
        const geo = new THREE.BufferGeometry().setFromPoints(points);
        const mat = new THREE.LineBasicMaterial({ color: mapEditVal === 100 ? 0xff3333 : mapEditVal === 0 ? 0x33ff33 : 0x888888 });
        editPreviewMesh = new THREE.Line(geo, mat);
        scene3D.add(editPreviewMesh);
    } else if (mapEditShape === 'rect') {
        const minX = Math.min(start.x, end.x);
        const maxX = Math.max(start.x, end.x);
        const minY = Math.min(start.y, end.y);
        const maxY = Math.max(start.y, end.y);
        const w = maxX - minX;
        const h = maxY - minY;
        const geo = new THREE.PlaneGeometry(Math.max(0.01, w), Math.max(0.01, h));
        const mat = new THREE.MeshBasicMaterial({ color: mapEditVal === 100 ? 0xff3333 : mapEditVal === 0 ? 0x33ff33 : 0x888888, opacity: 0.5, transparent: true, side: THREE.DoubleSide });
        editPreviewMesh = new THREE.Mesh(geo, mat);
        editPreviewMesh.position.set(minX + w / 2, minY + h / 2, 0.02);
        scene3D.add(editPreviewMesh);
    }
}

function clearEditPreview() {
    if (editPreviewMesh) {
        disposeMesh(editPreviewMesh);
        editPreviewMesh = null;
    }
}

function undoMapEdit() {
    if (editUndoStack.length === 0 || !mapData) return;
    const patch = editUndoStack.pop();
    for (const item of patch) {
        mapData.raw[item.rosIdx] = item.oldVal;
        setMapCellPixel(item.gx, item.gy, item.oldVal);
    }
    editRedoStack.push(patch);
    if (mapMesh3D?.material?.map) mapMesh3D.material.map.needsUpdate = true;
    updateEditUndoRedoButtons();
    const statusEl = document.getElementById('edit-map-status');
    if (statusEl) statusEl.textContent = `Undone edit (${patch.length} cells reverted)`;
}

function redoMapEdit() {
    if (editRedoStack.length === 0 || !mapData) return;
    const patch = editRedoStack.pop();
    for (const item of patch) {
        mapData.raw[item.rosIdx] = item.newVal;
        setMapCellPixel(item.gx, item.gy, item.newVal);
    }
    editUndoStack.push(patch);
    if (mapMesh3D?.material?.map) mapMesh3D.material.map.needsUpdate = true;
    updateEditUndoRedoButtons();
    const statusEl = document.getElementById('edit-map-status');
    if (statusEl) statusEl.textContent = `Redone edit (${patch.length} cells modified)`;
}

function resetMapEdit() {
    if (!originalMapData || !mapData) return;
    for (let i = 0; i < mapData.raw.length; i++) {
        mapData.raw[i] = originalMapData[i];
    }
    renderMapBitmap();
    if (mapMesh3D?.material?.map) mapMesh3D.material.map.needsUpdate = true;
    editUndoStack = [];
    editRedoStack = [];
    updateEditUndoRedoButtons();
    const statusEl = document.getElementById('edit-map-status');
    if (statusEl) statusEl.textContent = 'Reset back to original SLAM map';
}

async function saveEditedMap() {
    if (!mapData || !mapInfo) {
        alert('No map available to save!');
        return;
    }
    const mapNameInput = document.getElementById('edited-map-name-input');
    const mapName = mapNameInput ? mapNameInput.value.trim() : 'polebot_map_edited';

    if (!mapName) {
        alert('Please enter a valid map name');
        return;
    }

    const statusEl = document.getElementById('edit-map-status');
    if (statusEl) statusEl.textContent = `Saving map ${mapName}...`;

    const origin = [
        mapInfo.origin.position.x,
        mapInfo.origin.position.y,
        quaternionToYaw(mapInfo.origin.orientation || { w: 1, x: 0, y: 0, z: 0 })
    ];

    try {
        const response = await fetch('/api/save_edited_map', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                name: mapName,
                width: mapData.w,
                height: mapData.h,
                resolution: mapInfo.resolution,
                origin: origin,
                data: Array.from(mapData.raw)
            })
        });

        const res = await response.json();
        if (res.status === 'ok' || response.ok) {
            if (statusEl) statusEl.textContent = `✅ Saved as maps/${mapName}.yaml`;
            showNotice(`Edited map saved successfully to maps/${mapName}.yaml`);
            if (typeof fetchMapList === 'function') fetchMapList();
        } else {
            if (statusEl) statusEl.textContent = `❌ Error: ${res.message || 'Save failed'}`;
            showNotice(`Failed to save edited map: ${res.message}`);
        }
    } catch (err) {
        if (statusEl) statusEl.textContent = `❌ Error: ${err.message}`;
        showNotice(`Error saving map: ${err.message}`);
    }
}

// ── Map Rendering ─────────────────────────────────────

function renderMapBitmap() {
    if (!mapData) return;
    const { w, h, raw } = mapData;
    const off = mapData.bitmap || document.createElement('canvas');
    off.width = w;
    off.height = h;
    const octx = off.getContext('2d');
    const img = octx.createImageData(w, h);

    for (let y = 0; y < h; y++) {
        for (let x = 0; x < w; x++) {
            const rosIdx = y * w + x;
            const canvasIdx = (h - 1 - y) * w + x;
            const v = raw[rosIdx];
            let r, g, b;
            if (v === -1) { r = 120; g = 130; b = 140; }       // Unknown — blue-grey
            else if (v === 0) { r = 240; g = 245; b = 250; }    // Free — near white
            else { r = 30; g = 35; b = 45; }                    // Occupied — dark

            img.data[canvasIdx * 4] = r;
            img.data[canvasIdx * 4 + 1] = g;
            img.data[canvasIdx * 4 + 2] = b;
            img.data[canvasIdx * 4 + 3] = 255;
        }
    }

    octx.putImageData(img, 0, 0);
    mapData.bitmap = off;
}

function onMapReceived(data) {
    const newInfo = data.info;
    let rawData = data.data;
    const w = newInfo.width;
    const h = newInfo.height;

    if (!w || !h) return; // Prevent creating 0-size canvas if map is empty

    mapInfo = newInfo;

    // In some rosbridge configurations, byte arrays are sent as base64 string
    if (typeof rawData === 'string') {
        const decoded = atob(rawData);
        const arr = new Int8Array(decoded.length);
        for (let i = 0; i < decoded.length; i++) {
            let val = decoded.charCodeAt(i);
            if (val > 127) val -= 256; // Convert to signed Int8
            arr[i] = val;
        }
        rawData = arr;
    }

    // Preserve original map snapshot for Reset functionality
    if (!originalMapData || originalMapData.length !== rawData.length) {
        originalMapData = rawData.slice();
    }

    // If user is currently editing or has active edits, don't auto-overwrite user canvas
    if (interactMode === 'edit_map' || editUndoStack.length > 0) {
        originalMapData = rawData.slice();
        return;
    }

    mapData = { w, h, raw: rawData, version: (mapData ? (mapData.version || 0) + 1 : 1) };
    renderMapBitmap();

    document.getElementById('map-topic-label').textContent =
        `Topic: /map — ${w}×${h} @ ${mapInfo.resolution.toFixed(3)}m/px`;

    updateNavReadyUI();
    if (zones.length === 0) {
        fetchZones();
    } else {
        update3DZoneMesh();
    }
}

// ── Costmap Rendering (RViz-style gradient) ───────────

// OccupancyGrid cost values, matching RViz's costmap palette.
// Source: ros2/rviz, rviz_default_plugins/displays/map/palette_builder.cpp.
// Do not interpret this 0..100 topic as the 0..255 costmap_raw format.
function costmapColor(value) {
    if (!Number.isInteger(value) || value <= 0 || value > 100) return [0, 0, 0, 0];
    if (value === 99) return [0, 255, 255, 255];
    if (value === 100) return [255, 0, 255, 255];
    const red = Math.floor(255 * value / 100);
    return [red, 0, 255 - red, 255];
}

function processCostmapToMesh(data, existingMesh, layer) {
    const info = data.info;
    const w = info.width;
    const h = info.height;
    if (!w || !h || !Number.isFinite(info.resolution) || info.resolution <= 0) return existingMesh;

    let rawData = data.data;
    if (typeof rawData === 'string') {
        const decoded = atob(rawData);
        const arr = new Int8Array(decoded.length);
        for (let i = 0; i < decoded.length; i++) {
            let val = decoded.charCodeAt(i);
            if (val > 127) val -= 256;
            arr[i] = val;
        }
        rawData = arr;
    }

    const off = document.createElement('canvas');
    off.width = w;
    off.height = h;
    const octx = off.getContext('2d');
    const img = octx.createImageData(w, h);

    for (let y = 0; y < h; y++) {
        for (let x = 0; x < w; x++) {
            const rosIdx = y * w + x;
            const canvasIdx = (h - 1 - y) * w + x;
            const v = rawData[rosIdx];
            const [r, g, b, a] = costmapColor(v);
            const displayAlpha = v < costmapMinVisible ? 0 : a;
            img.data[canvasIdx * 4] = r;
            img.data[canvasIdx * 4 + 1] = g;
            img.data[canvasIdx * 4 + 2] = b;
            img.data[canvasIdx * 4 + 3] = displayAlpha;
        }
    }

    octx.putImageData(img, 0, 0);

    const texture = new THREE.CanvasTexture(off);
    texture.minFilter = THREE.NearestFilter;
    texture.magFilter = THREE.NearestFilter;
    texture.flipY = true;

    if (existingMesh && scene3D) {
        scene3D.remove(existingMesh);
        if (existingMesh.geometry) existingMesh.geometry.dispose();
        if (existingMesh.material) {
            if (existingMesh.material.map) existingMesh.material.map.dispose();
            existingMesh.material.dispose();
        }
    }

    const worldW = w * info.resolution;
    const worldH = h * info.resolution;
    const planeGeo = new THREE.PlaneGeometry(worldW, worldH);
    const planeMat = new THREE.MeshBasicMaterial({ map: texture, transparent: true, depthWrite: false, side: THREE.DoubleSide });
    const newMesh = new THREE.Mesh(planeGeo, planeMat);

    const ox = info.origin.position.x;
    const oy = info.origin.position.y;
    // Global costmap at z=0.01, local costmap at z=0.02 (above global)
    const zOff = layer === 'local' ? 0.02 : 0.01;
    newMesh.position.set(ox + worldW / 2, oy + worldH / 2, zOff);
    newMesh.userData.costmap = { frame: data.header?.frame_id?.replace(/^\//, '') || '', info, zOff };
    document.getElementById(`${layer}-extent`).textContent = `${worldW.toFixed(1)} × ${worldH.toFixed(1)} m · ${info.resolution.toFixed(2)} m/cell`;

    scene3D.add(newMesh);
    return newMesh;
}





// ── 3D Interaction ────────────────────────────────
function setMode(mode) {
    if (currentTab === 'nav' && mode === 'edit_map') {
        mode = 'view';
    }
    cancelPlacement();
    interactMode = mode;
    const btnGoal = document.getElementById('btn-goal-mode');
    const btnInit = document.getElementById('btn-init-mode');
    const btnEdit = document.getElementById('btn-edit-mode');
    const btnZone = document.getElementById('btn-edit-zone-mode');
    const btnView = document.getElementById('btn-view-mode');
    if (btnGoal) btnGoal.classList.toggle('active', mode === 'goal');
    if (btnInit) btnInit.classList.toggle('active', mode === 'initial_pose');
    if (btnEdit) btnEdit.classList.toggle('active', mode === 'edit_map');
    if (btnZone) btnZone.classList.toggle('active', mode === 'edit_zone');
    if (btnView) btnView.classList.toggle('active', mode === 'view');

    const hint = document.getElementById('interaction-hint');
    if (mode === 'edit_zone') {
        if (hint) hint.textContent = '🛡️ Adobe Zone Tool: Click & drag on map to draw rectangles · Drag handles to resize · Esc to exit';
    } else if (mode === 'edit_map') {
        if (hint) hint.textContent = '🎨 Map Edit Mode: Left-drag to draw · Use sidebar tools & brush size · Esc to leave edit mode';
    } else {
        if (hint) hint.textContent = mode === 'view' ? 'Drag to orbit · Right-drag to pan · Scroll to zoom' : 'Click to place · Drag to choose heading · Esc to cancel';
    }
    if (controls3D) controls3D.enabled = (mode === 'view');
    renderZoneOverlay();
}

function resetView() {
    if (!camera3D || !controls3D || !mapInfo) return;
    const width = mapInfo.width * mapInfo.resolution;
    const height = mapInfo.height * mapInfo.resolution;
    const center = new THREE.Vector3(mapInfo.origin.position.x + width / 2, mapInfo.origin.position.y + height / 2, 0);
    const halfFov = camera3D.fov * Math.PI / 360;
    const distance = Math.max(height / 2 / Math.tan(halfFov), width / 2 / (Math.tan(halfFov) * camera3D.aspect)) * 1.2;
    // Preserve the existing viewing orientation; only fit position and distance.
    const direction = camera3D.position.clone().sub(controls3D.target).normalize();
    controls3D.target.copy(center);
    camera3D.position.copy(center).addScaledVector(direction, Math.max(distance, 2));
    camera3D.lookAt(center);
    controls3D.update();
}

function quaternionToYaw(q) {
    return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

const NAV_STATUS_MAP = {
    0: ['UNKNOWN', 'status-idle'],
    1: ['ACCEPTED', 'status-navigating'],
    2: ['EXECUTING', 'status-navigating'],
    3: ['CANCELING', 'status-checking'],
    4: ['SUCCEEDED', 'status-reached'],
    5: ['CANCELED', 'status-idle'],
    6: ['ABORTED', 'status-failed'],
};

function onNavStatusReceived(data) {
    if (activeActionId) return;
    const statuses = data.status_list;
    const el = document.getElementById('nav-status-value');
    const sub = document.getElementById('nav-status-sub');
    const btnCancel = document.getElementById('btn-cancel');

    if (!statuses || statuses.length === 0) {
        if (navigationUncertain) return;
        el.className = 'nav-status-value status-idle';
        el.textContent = 'IDLE';
        sub.textContent = 'No active goals';
        btnCancel.disabled = true;
        return;
    }

    // Status arrays are not guaranteed to be sorted.
    const latest = [...statuses].sort((a, b) => (a.goal_info?.stamp?.sec || 0) - (b.goal_info?.stamp?.sec || 0) || (a.goal_info?.stamp?.nanosec || 0) - (b.goal_info?.stamp?.nanosec || 0)).at(-1);
    if (navigationUncertain && !statuses.some(item => [1, 2, 3].includes(item.status))) {
        navigationUncertain = false;
        showNotice('Nav2 reports no active navigation goals. Previous disconnected goal result could not be verified.');
        updateNavReadyUI();
    }
    const [label, cls] = NAV_STATUS_MAP[latest.status] || ['UNKNOWN', 'status-idle'];
    el.className = `nav-status-value ${cls}`;
    el.textContent = label;
    sub.textContent = `Goal ID: ${latest.goal_info?.goal_id?.uuid?.slice(0, 8) || '—'}...`;

    const isActive = [1, 2, 3].includes(latest.status);
    btnCancel.disabled = true; // External goals are observed, never canceled by this client.
    sub.textContent = isActive ? 'Navigation goal from another client' : label;

}

// ── Goal Setting ──────────────────────────────────────

let initMesh3D = null;
let pendingInitPose = null;

function setInitialPose(wx, wy, yaw = 0) {
    clearPlacementPreview();
    pendingInitPose = { x: wx, y: wy, yaw };
    document.getElementById('init-yaw').value = (yaw * 180 / Math.PI).toFixed(1);
    document.getElementById('init-hint').textContent = 'Initial pose selected. Publish to initialize AMCL.';
    document.getElementById('init-coords').textContent = `X: ${wx.toFixed(3)}m  Y: ${wy.toFixed(3)}m  Yaw: ${(yaw * 180 / Math.PI).toFixed(1)}°`;
    document.getElementById('btn-send-init').disabled = false;
    setMode('view'); // auto switch back to view

    if (scene3D) {
        // Remove old marker
        if (initMesh3D) {
            initMesh3D.traverse(o => { o.geometry?.dispose(); o.material?.dispose(); });
            scene3D.remove(initMesh3D);
        }
        // ArrowHelper showing yaw direction (green, like RViz 2D Pose Estimate)
        const dir = new THREE.Vector3(Math.cos(yaw), Math.sin(yaw), 0);
        const origin = new THREE.Vector3(wx, wy, 0.15);
        initMesh3D = new THREE.ArrowHelper(dir, origin, 0.8, 0x63e6a5, 0.22, 0.14);
        scene3D.add(initMesh3D);
    }
}

function sendInitialPose() {
    if (!pendingInitPose || !wsConnected || !lifecycleStates.amcl) return;
    const { x, y, yaw } = pendingInitPose;

    send({
        op: 'publish',
        topic: '/initialpose',
        msg: {
            header: { frame_id: 'map' },
            pose: {
                pose: {
                    position: { x, y, z: 0 },
                    orientation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) }
                },
                covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.068]
            }
        }
    });

    document.getElementById('init-hint').textContent = 'Initial pose published. Check robot pose for AMCL localization.';
    pendingInitPose = null;
    document.getElementById('btn-send-init').disabled = true;

    // Hide mesh after 2 seconds
    setTimeout(() => {
        if (pendingInitPose) return;
        if (initMesh3D && scene3D) {
            disposeMesh(initMesh3D);
            initMesh3D = null;
        }
        document.getElementById('init-hint').textContent = 'Hold at the robot location, drag toward its front, then release and publish.';
        document.getElementById('init-coords').textContent = '';
    }, 2000);
}
let goalMesh3D = null;
function setGoalPose(wx, wy, yaw = 0) {
    clearPlacementPreview();
    pendingGoal = { x: wx, y: wy, yaw };
    document.getElementById('goal-yaw').value = (yaw * 180 / Math.PI).toFixed(1);
    document.getElementById('goal-hint').textContent = 'Goal selected. Send navigation goal to begin.';
    document.getElementById('goal-coords').textContent = `X: ${wx.toFixed(3)}m  Y: ${wy.toFixed(3)}m  Yaw: ${(yaw * 180 / Math.PI).toFixed(1)}°`;
    document.getElementById('btn-send-goal').disabled = !nav2Ready;
    setMode('view'); // auto switch back to view

    if (scene3D) {
        // Remove old marker
        if (goalMesh3D) {
            goalMesh3D.traverse(o => { o.geometry?.dispose(); o.material?.dispose(); });
            scene3D.remove(goalMesh3D);
        }
        // ArrowHelper showing yaw direction (yellow/gold, like RViz Nav2 Goal)
        const dir = new THREE.Vector3(Math.cos(yaw), Math.sin(yaw), 0);
        const origin = new THREE.Vector3(wx, wy, 0.15);
        goalMesh3D = new THREE.ArrowHelper(dir, origin, 0.8, 0xf4bd62, 0.22, 0.14);
        scene3D.add(goalMesh3D);
    }
}

function sendGoal() {
    if (!pendingGoal || !wsConnected || !nav2Ready || activeActionId || navigationUncertain) return;
    const { x, y, yaw } = pendingGoal;
    activeActionId = `nav-${Date.now()}-${++requestCounter}`;
    actionPhase = 'sending';
    goalPose = { ...pendingGoal };
    setNavigationState('SENDING', 'Waiting for Nav2 feedback or result.', 'status-checking');
    document.getElementById('goal-hint').textContent = 'Goal requested. Waiting for Nav2.';
    document.getElementById('btn-cancel').disabled = false;
    updateNavReadyUI();
    send({
        op: 'send_action_goal', id: activeActionId, action: '/navigate_to_pose',
        action_type: 'nav2_msgs/action/NavigateToPose', feedback: true,
        args: {
            pose: {
                header: { frame_id: 'map' }, pose: {
                    position: { x, y, z: 0 }, orientation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) }
                }
            }, behavior_tree: ''
        }
    });

}
function cancelNav() {
    if (!wsConnected || !activeActionId) return;
    actionPhase = 'canceling';
    setNavigationState('CANCELING', 'Cancellation requested; waiting for Nav2 result.', 'status-checking');
    send({ op: 'cancel_action_goal', id: activeActionId, action: '/navigate_to_pose' });
}



// ── Tabs ──────────────────────────────────────────────

let currentTab = 'nav';

function clearMap() {
    // Clear 3D Map
    if (mapMesh3D && scene3D) {
        scene3D.remove(mapMesh3D);
        if (mapMesh3D.geometry) mapMesh3D.geometry.dispose();
        if (mapMesh3D.material) {
            if (mapMesh3D.material.map) mapMesh3D.material.map.dispose();
            mapMesh3D.material.dispose();
        }
        mapMesh3D = null;
    }

    // Clear Costmaps
    if (globalCostmapMesh3D && scene3D) {
        disposeMesh(globalCostmapMesh3D);
        globalCostmapMesh3D = null;
    }
    if (localCostmapMesh3D && scene3D) {
        disposeMesh(localCostmapMesh3D);
        localCostmapMesh3D = null;
    }
    disposeMesh(pathMesh3D);
    pathMesh3D = null;
    // Note: We do NOT reset isNavMapLoaded here, because Nav2 might still be running!
    // The map will be re-drawn automatically by the next ROS topic message.
}

function switchTab(tabId) {
    if (currentTab !== tabId && (tabId === 'nav' || tabId === 'slam')) {
        clearMap();
    }
    currentTab = tabId;
    setMode('view');
    document.getElementById('display-panel').style.display = tabId === 'system' ? 'none' : '';

    // Update button states
    document.getElementById('tab-nav').classList.toggle('active', tabId === 'nav');
    document.getElementById('tab-slam').classList.toggle('active', tabId === 'slam');
    document.getElementById('tab-system').classList.toggle('active', tabId === 'system');

    // Update panel visibility
    document.getElementById('panel-nav').style.display = tabId === 'nav' ? 'flex' : 'none';
    document.getElementById('panel-slam').style.display = tabId === 'slam' ? 'flex' : 'none';
    document.getElementById('panel-system').style.display = tabId === 'system' ? 'grid' : 'none';

    // Hide map in system tab
    document.getElementById('map-container').style.display = tabId === 'system' ? 'none' : 'flex';

    // Sidebar sections visibility
    const isNav = tabId === 'nav';
    const isSlam = tabId === 'slam';
    const sceneLayersSec = document.getElementById('section-scene-layers');
    const costmapsSec = document.getElementById('section-costmaps');
    const costDisplaySec = document.getElementById('section-cost-display');
    const mapEditorSec = document.getElementById('section-map-editor');
    const zoneEditorSec = document.getElementById('section-zone-editor');
    const motionArchSec = document.getElementById('section-motion-architecture');

    if (sceneLayersSec) sceneLayersSec.style.display = isNav ? '' : 'none';
    if (costmapsSec) costmapsSec.style.display = isNav ? '' : 'none';
    if (costDisplaySec) costDisplaySec.style.display = isNav ? '' : 'none';
    if (zoneEditorSec) zoneEditorSec.style.display = isNav ? '' : 'none';
    if (motionArchSec) motionArchSec.style.display = isNav ? '' : 'none';

    if (mapEditorSec) mapEditorSec.style.display = isSlam ? '' : 'none';

    // Toolbar buttons visibility for active tab
    const btnEdit = document.getElementById('btn-edit-mode');
    const btnZone = document.getElementById('btn-edit-zone-mode');
    const btnInit = document.getElementById('btn-init-mode');
    const btnGoal = document.getElementById('btn-goal-mode');

    if (btnEdit) btnEdit.style.display = isSlam ? '' : 'none';
    if (btnZone) btnZone.style.display = isNav ? '' : 'none';
    if (btnInit) btnInit.style.display = isNav ? '' : 'none';
    if (btnGoal) btnGoal.style.display = isNav ? '' : 'none';

    // Status checks
    if (tabId === 'nav') {
        checkNav2Status();
        updateNavReadyUI();
    } else if (tabId === 'slam') {
        checkSlamStatus();
        updateNavReadyUI();
    } else {
        pollStatus();
    }
}

window.addEventListener('resize', () => {
    if (camera3D && renderer3D) {
        const wrap = document.getElementById('map-canvas-wrap');
        if (wrap && wrap.offsetWidth > 0) {
            const w = wrap.offsetWidth;
            const h = wrap.offsetHeight;
            camera3D.aspect = w / h;
            camera3D.updateProjectionMatrix();
            renderer3D.setSize(w, h);
        }
    }
});

connect();

// ── System Control API ──────────────────────────────
async function apiControl(endpoint) {
    try {
        await fetch(`/api/${endpoint}`, { method: 'POST' });
        pollStatus(); // Immediate refresh
    } catch (e) {
        console.error("API Error:", e);
    }
}

async function pollStatus() {
    if (currentTab !== 'system') return;
    try {
        const res = await fetch('/api/status');
        const status = await res.json();

        document.getElementById('btn-start-motor').disabled = status.motor;
        document.getElementById('btn-stop-motor').disabled = !status.motor;

        document.getElementById('btn-start-slam').disabled = status.slam;
        document.getElementById('btn-stop-slam').disabled = !status.slam;

        document.getElementById('btn-stop-nav').disabled = !status.nav;
    } catch (e) {
        console.error("Status Poll Error:", e);
    }
}

setInterval(pollStatus, 2000);

// ── Teleop Overlay ───────────────────────────────────

let teleopActive = false;
let joystickActive = false;
let joyX = 0, joyY = 0; // -1.0 to 1.0

function toggleTeleop() {
    const overlay = document.getElementById('teleop-overlay');
    teleopActive = !teleopActive;
    overlay.style.display = teleopActive ? 'flex' : 'none';
    if (!teleopActive) {
        joyX = 0; joyY = 0;
        publishCmdVel(0, 0); // stop immediately when closed
    } else {
        initJoystick();
    }
}

// Speed Buttons
let teleopMaxV = 0.10;
let teleopMaxW = 0.2;

function setTeleopSpeed(v, w, btnId) {
    teleopMaxV = v;
    teleopMaxW = w;
    const btns = document.querySelectorAll('#teleop-speed-controls .speed-btn');
    btns.forEach(b => b.classList.remove('active'));
    document.getElementById(btnId).classList.add('active');
}

// Joystick Canvas
function initJoystick() {
    const canvas = document.getElementById('joystick-canvas');
    if (!canvas || canvas.joystickInit) return;
    canvas.joystickInit = true;

    const ctx = canvas.getContext('2d');
    const radius = 40;
    const center = { x: canvas.width / 2, y: canvas.height / 2 };
    let thumb = { x: center.x, y: center.y };

    function draw() {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        // Base
        ctx.beginPath();
        ctx.arc(center.x, center.y, radius, 0, 2 * Math.PI);
        ctx.fillStyle = 'rgba(255, 255, 255, 0.05)';
        ctx.fill();
        ctx.lineWidth = 2;
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)';
        ctx.stroke();

        // Thumb
        ctx.beginPath();
        ctx.arc(thumb.x, thumb.y, 20, 0, 2 * Math.PI);
        ctx.fillStyle = joystickActive ? 'rgba(0, 216, 255, 0.8)' : 'rgba(255, 255, 255, 0.4)';
        ctx.fill();
    }

    function handleInput(x, y) {
        const dx = x - center.x;
        const dy = y - center.y;
        const dist = Math.min(Math.hypot(dx, dy), radius);
        const angle = Math.atan2(dy, dx);

        thumb.x = center.x + Math.cos(angle) * dist;
        thumb.y = center.y + Math.sin(angle) * dist;

        // Map to -1.0 to 1.0 (Note: -dy because Canvas Y is down, ROS X is up)
        // dx is Left/Right (ROS Z rotation: left is positive, right is negative)
        joyX = -(thumb.y - center.y) / radius; // Forward/Backward
        joyY = -(thumb.x - center.x) / radius; // Left/Right turn
        draw();
    }

    function resetInput() {
        thumb = { x: center.x, y: center.y };
        joyX = 0; joyY = 0;
        joystickActive = false;
        draw();
    }

    // Mouse Events
    canvas.addEventListener('mousedown', e => {
        joystickActive = true;
        handleInput(e.offsetX, e.offsetY);
    });
    window.addEventListener('mousemove', e => {
        if (!joystickActive || !teleopActive) return;
        const rect = canvas.getBoundingClientRect();
        handleInput(e.clientX - rect.left, e.clientY - rect.top);
    });
    window.addEventListener('mouseup', () => {
        if (joystickActive) resetInput();
    });

    // Touch Events
    canvas.addEventListener('touchstart', e => {
        joystickActive = true;
        const rect = canvas.getBoundingClientRect();
        handleInput(e.touches[0].clientX - rect.left, e.touches[0].clientY - rect.top);
        e.preventDefault();
    }, { passive: false });
    canvas.addEventListener('touchmove', e => {
        if (!joystickActive) return;
        const rect = canvas.getBoundingClientRect();
        handleInput(e.touches[0].clientX - rect.left, e.touches[0].clientY - rect.top);
        e.preventDefault();
    }, { passive: false });
    canvas.addEventListener('touchend', e => {
        if (joystickActive) resetInput();
    });

    draw();
}

// Publish loop (10Hz)
function publishCmdVel(v, w) {
    if (!wsConnected) return;
    send({
        op: 'publish',
        topic: '/cmd_vel',
        msg: {
            linear: { x: v, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: w }
        }
    });
}

// ── Physical Gamepad (HTML5) ─────────────────────────

let physicalGamepadIndex = null;
let physGamepadActive = false;

window.addEventListener("gamepadconnected", (e) => {
    physicalGamepadIndex = e.gamepad.index;
    const statusText = document.getElementById('gamepad-status-text');
    if (statusText) statusText.textContent = `🟢 Connected: ${e.gamepad.id}`;
    const btn = document.getElementById('btn-toggle-gamepad');
    if (btn) btn.disabled = false;
});

window.addEventListener("gamepaddisconnected", (e) => {
    if (e.gamepad.index === physicalGamepadIndex) {
        physicalGamepadIndex = null;
        physGamepadActive = false;
        const statusText = document.getElementById('gamepad-status-text');
        if (statusText) statusText.textContent = `🔴 Disconnected: ${e.gamepad.id}`;
        const btn = document.getElementById('btn-toggle-gamepad');
        if (btn) {
            btn.disabled = true;
            btn.textContent = '❌ Enable Physical Gamepad';
            btn.style.background = 'linear-gradient(135deg, #a371f7, #8957e5)';
        }
    }
});

function toggleGamepadInput() {
    physGamepadActive = !physGamepadActive;
    const btn = document.getElementById('btn-toggle-gamepad');
    if (!btn) return;

    if (physGamepadActive) {
        btn.textContent = '✅ Gamepad Active (Press to Disable)';
        btn.style.background = 'linear-gradient(135deg, #238636, #2ea043)';
    } else {
        btn.textContent = '❌ Enable Physical Gamepad';
        btn.style.background = 'linear-gradient(135deg, #a371f7, #8957e5)';
        publishCmdVel(0, 0); // Stop
    }
}

setInterval(() => {
    // 1. Physical Gamepad has priority if active
    if (physGamepadActive && physicalGamepadIndex !== null) {
        const gamepads = navigator.getGamepads();
        const gp = gamepads[physicalGamepadIndex];
        if (gp) {
            // Standard mapping: Axis 1 (Left Y) -> Linear, Axis 2 (Right X) -> Angular
            // Some controllers use Axis 3 for Right Y, Axis 2 for Right X. 
            // Stick UP is usually -1.0, Stick LEFT is usually -1.0
            let axisLin = gp.axes[1] || 0;
            let axisAng = gp.axes[2] || 0;

            // Apply deadzone
            if (Math.abs(axisLin) < 0.1) axisLin = 0;
            if (Math.abs(axisAng) < 0.1) axisAng = 0;

            const v = -axisLin * teleopMaxV; // Negate so UP (-1.0) becomes positive V
            const w = -axisAng * teleopMaxW; // Negate so LEFT (-1.0) becomes positive W

            publishCmdVel(v, w);
            return; // Skip virtual joystick
        }
    }

    // 2. Virtual Joystick fallback
    if (!teleopActive) return;

    // If joystick is near zero, snap to zero to prevent drifting
    const v = Math.abs(joyX) < 0.05 ? 0 : joyX * teleopMaxV;
    const w = Math.abs(joyY) < 0.05 ? 0 : joyY * teleopMaxW;

    publishCmdVel(v, w);
}, 100);

// ── Motion & Planning Architecture Selector ────────────────────────
let motionMode = {
    controller: 'native',
    planner: 'native'
};
let motionModeData = null;

async function fetchMotionModes() {
    try {
        const res = await fetch('/api/motion_modes');
        if (!res.ok) return;
        const data = await res.json();
        motionModeData = data;
        if (data.active) {
            motionMode = data.active;
        }
        updateMotionUI();
    } catch (e) {
        console.warn('Error fetching motion modes:', e);
    }
}

async function selectControllerMode(controllerId) {
    motionMode.controller = controllerId;
    await applyMotionMode();
}

async function selectPlannerMode(plannerId) {
    motionMode.planner = plannerId;
    await applyMotionMode();
}

async function revertToStandardOK() {
    motionMode.controller = 'native';
    motionMode.planner = 'native';
    await applyMotionMode();
}

async function applyMotionMode() {
    updateMotionUI();
    try {
        const res = await fetch('/api/set_motion_mode', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(motionMode)
        });
        const result = await res.json();
        if (result.status === 'success') {
            if (typeof showWorkspaceMessage === 'function') {
                showWorkspaceMessage(result.message || 'Motion architecture updated');
            } else if (typeof showNotice === 'function') {
                showNotice(result.message || 'Motion architecture updated');
            }
        } else {
            if (typeof showNotice === 'function') showNotice(`Failed to switch mode: ${result.message}`);
        }
    } catch (e) {
        if (typeof showNotice === 'function') showNotice(`Connection error setting motion mode: ${e.message}`);
    }
}

function updateMotionUI() {
    // Buttons state
    const ctrlNative = document.getElementById('btn-ctrl-native');
    const ctrlPid = document.getElementById('btn-ctrl-pid');
    const ctrlSmc = document.getElementById('btn-ctrl-smc');

    if (ctrlNative) ctrlNative.classList.toggle('active', motionMode.controller === 'native');
    if (ctrlPid) ctrlPid.classList.toggle('active', motionMode.controller === 'pid');
    if (ctrlSmc) ctrlSmc.classList.toggle('active', motionMode.controller === 'smc');

    const planNative = document.getElementById('btn-plan-native');
    const planBfs = document.getElementById('btn-plan-bfs');
    const planAstar = document.getElementById('btn-plan-astar');

    if (planNative) planNative.classList.toggle('active', motionMode.planner === 'native');
    if (planBfs) planBfs.classList.toggle('active', motionMode.planner === 'bfs');
    if (planAstar) planAstar.classList.toggle('active', motionMode.planner === 'astar');

    // Badge state
    const badge = document.getElementById('motion-mode-badge');
    if (badge) {
        if (motionMode.controller === 'native' && motionMode.planner === 'native') {
            badge.className = 'mode-status-badge badge-native';
            badge.textContent = 'Native OK';
        } else if (motionMode.controller === 'smc') {
            badge.className = 'mode-status-badge badge-smc';
            badge.textContent = 'SMC Active';
        } else {
            badge.className = 'mode-status-badge badge-pid';
            badge.textContent = `${motionMode.controller.toUpperCase()} + ${motionMode.planner.toUpperCase()}`;
        }
    }

    // Title and Parameter Details Grid
    const title = document.getElementById('arch-param-title');
    const grid = document.getElementById('arch-param-grid');

    const ctrlName = motionMode.controller === 'native' ? 'Native DWB' : (motionMode.controller === 'pid' ? 'PID Profiled' : 'SMC Control');
    const planName = motionMode.planner === 'native' ? 'Nav2 A*' : (motionMode.planner === 'bfs' ? 'BFS Grid' : 'A* Optimal');
    if (title) title.textContent = `Active: ${ctrlName} + ${planName}`;

    if (grid) {
        if (motionMode.controller === 'native') {
            grid.innerHTML = `
                <div><span>Linear:</span> <strong>0.18 m/s</strong></div>
                <div><span>Angular:</span> <strong>0.22 rad/s (Gentle)</strong></div>
                <div><span>Decel:</span> <strong>-0.40 m/s²</strong></div>
                <div><span>Recovery:</span> <strong>1x BackUp → Terminate</strong></div>
            `;
        } else if (motionMode.controller === 'pid') {
            grid.innerHTML = `
                <div><span>Profile:</span> <strong>S-Curve Jerk</strong></div>
                <div><span>Max Jerk:</span> <strong>0.16 m/s³</strong></div>
                <div><span>Cruise Kp:</span> <strong>3.20</strong></div>
                <div><span>Decel Kp:</span> <strong>3.00</strong></div>
            `;
        } else if (motionMode.controller === 'smc') {
            grid.innerHTML = `
                <div><span>Method:</span> <strong>Alipour 2019</strong></div>
                <div><span>Surfaces:</span> <strong>S1(ρ), S2(φ)</strong></div>
                <div><span>Lambda:</span> <strong>λ1=0.5, λ2=1.5</strong></div>
                <div><span>Switch K:</span> <strong>K1=2.0, K2=10.0</strong></div>
            `;
        }
    }
}
