// Workspace behavior. The calibrated map, robot and laser transforms stay in app.js.
const serviceRequests = new Map();
const lifecycleStates = {};
const topicLastSeen = {};
const cachedCostmaps = { global: null, local: null };
const costmapOpacity = { global: 0.3, local: 0.65 };
let costmapMinVisible = 1;
const costmapTransforms = new Map();
const layers = { map: true, robot: true, scan: true, path: true, grid: true };
let requestCounter = 0;
let checkingLifecycle = false;
let slamReady = false;
let activeActionId = null;
let actionPhase = 'idle';
let navigationUncertain = false;
let robotPoseFrame = "";
let gridMesh3D = null;
let pathMesh3D = null;
let hasFittedMap = false;
let followRobot = false;
let lastFollowPose = null;
let placement = null;
let placementArrow = null;

function showNotice(message) {
    document.getElementById('workspace-message').textContent = message;
}
function rosService(service, args, timeout = 4000) {
    return new Promise((resolve, reject) => {
        if (!wsConnected) { reject(new Error('ROSBridge is disconnected')); return; }
        const id = `service-${++requestCounter}`;
        const timer = setTimeout(() => {
            serviceRequests.delete(id);
            reject(new Error(`${service} timed out`));
        }, timeout);
        serviceRequests.set(id, { resolve, reject, timer });
        send({ op: 'call_service', id, service, args });
    });
}
function rejectPendingServices() {
    for (const request of serviceRequests.values()) {
        clearTimeout(request.timer);
        request.reject(new Error('ROSBridge disconnected'));
    }
    serviceRequests.clear();
}
function setNavigationState(label, detail, cls) {
    const el = document.getElementById('nav-status-value');
    el.textContent = label;
    el.className = `nav-status-value ${cls}`;
    document.getElementById('nav-status-sub').textContent = detail;
}
function handleWorkspaceMessage(message) {
    if (message.op === 'service_response') {
        const request = serviceRequests.get(message.id);
        if (!request) return false;
        clearTimeout(request.timer);
        serviceRequests.delete(message.id);
        if (message.result === false) request.reject(new Error(String(message.values || 'Service failed')));
        else request.resolve(message.values || {});
        return true;
    }
    if (message.id !== activeActionId || !activeActionId) return false;
    if (message.op === 'action_feedback') {
        const feedback = message.values || {};
        if (actionPhase !== 'canceling') actionPhase = 'executing';
        if (actionPhase !== 'canceling') setNavigationState('EXECUTING', 'Nav2 is navigating to the selected goal.', 'status-navigating');
        document.getElementById('nav-distance').textContent = Number.isFinite(feedback.distance_remaining) ? `${feedback.distance_remaining.toFixed(2)} m` : '—';
        document.getElementById('nav-eta').textContent = feedback.estimated_time_remaining ? `${feedback.estimated_time_remaining.sec} s` : '—';
        document.getElementById('nav-recoveries').textContent = feedback.number_of_recoveries ?? '—';
        return true;
    }
    if (message.op === 'action_result') {
        const result = message.result !== false && message.status === 4;
        const label = result ? 'SUCCEEDED' : message.status === 5 ? 'CANCELED' : 'FAILED';
        const detail = typeof message.values === 'string' ? message.values : message.values?.error_msg || (result ? 'Destination reached.' : message.status === 5 ? 'Nav2 confirmed cancellation.' : `Nav2 finished with status ${message.status}.`);
        setNavigationState(label, detail, result ? 'status-reached' : message.status === 5 ? 'status-idle' : 'status-failed');
        document.getElementById('goal-hint').textContent = detail;
        activeActionId = null;
        actionPhase = 'idle';
        pendingGoal = null;
        document.getElementById('btn-cancel').disabled = true;
        updateNavReadyUI();
        return true;
    }
    if (message.op === 'status' && message.level === 'error') {
        const detail = message.msg || 'ROSBridge rejected the navigation request.';
        showNotice(detail);
        if (actionPhase === 'sending') {
            setNavigationState('FAILED', detail, 'status-failed');
            activeActionId = null;
            actionPhase = 'idle';
            document.getElementById('btn-cancel').disabled = true;
            updateNavReadyUI();
        } else {
            setNavigationState('UNKNOWN', 'Action communication error. Await Nav2 result or reconnect to reconcile status.', 'status-checking');
        }
        return true;
    }
    return false;
}
function disposeMesh(mesh) {
    if (!mesh) return;
    mesh.parent?.remove(mesh);
    const dispose = object => {
        object.geometry?.dispose();
        for (const material of (Array.isArray(object.material) ? object.material : [object.material])) {
            material?.map?.dispose(); material?.dispose();
        }
    };
    if (mesh.traverse) mesh.traverse(dispose); else dispose(mesh);
}
function setLayer(name, visible) {
    layers[name] = visible;
    applyLayerVisibility();
}
function setCostmapOpacity(name, value) {
    costmapOpacity[name] = Number(value);
    applyLayerVisibility();
}
function applyLayerVisibility() {
    if (!scene3D) return;
    if (mapMesh3D) mapMesh3D.visible = layers.map;
    // Scan remains visible independently, even though its calibrated parent is robotMesh3D.
    if (robotMesh3D) for (const child of robotMesh3D.children) child.visible = child === scanPoints3D ? layers.scan : layers.robot;
    if (gridMesh3D) gridMesh3D.visible = layers.grid;
    if (pathMesh3D) pathMesh3D.visible = layers.path && currentTab === 'nav';
    for (const [name, mesh, visible] of [['global', globalCostmapMesh3D, showGlobalCostmap], ['local', localCostmapMesh3D, showLocalCostmap]]) {
        if (mesh) { mesh.visible = visible && currentTab === 'nav' && positionCostmap(mesh, name); mesh.material.opacity = costmapOpacity[name]; }
    }
}
function drawPath(message) {
    if (!scene3D) return;
    disposeMesh(pathMesh3D);
    pathMesh3D = null;
    if (message.header?.frame_id?.replace(/^\//, '') !== 'map') {
        showNotice(`Global path frame is ${message.header?.frame_id || 'unknown'}; expected map. Path hidden to avoid misalignment.`);
        return;
    }
    const rawPoses = message.poses || [];
    if (rawPoses.length < 2) return;

    const group = new THREE.Group();
    const points = rawPoses.map(pose => new THREE.Vector3(pose.pose.position.x, pose.pose.position.y, 0.08));

    // 1. Crisp white core center line
    const lineGeo = new THREE.BufferGeometry().setFromPoints(points);
    const lineMat = new THREE.LineBasicMaterial({ color: 0xffffff, depthTest: false });
    const lineMesh = new THREE.Line(lineGeo, lineMat);
    group.add(lineMesh);

    // 2. Thick 3D glowing neon cyan/green tube (10cm diameter, depthTest: false)
    try {
        const curvePoints = rawPoses.map(pose => new THREE.Vector3(pose.pose.position.x, pose.pose.position.y, 0.06));
        const curve = new THREE.CatmullRomCurve3(curvePoints);
        const tubularSegments = Math.max(rawPoses.length * 2, 24);
        const tubeGeo = new THREE.TubeGeometry(curve, tubularSegments, 0.05, 8, false);
        const tubeMat = new THREE.MeshBasicMaterial({ color: 0x00ffcc, transparent: true, opacity: 0.9, depthTest: false });
        const tubeMesh = new THREE.Mesh(tubeGeo, tubeMat);
        group.add(tubeMesh);
    } catch (e) {
        console.warn('TubeGeometry creation fallback:', e);
    }

    pathMesh3D = group;
    pathMesh3D.visible = layers.path && currentTab === 'nav';
    scene3D.add(pathMesh3D);
}
function toggleFollow() {
    followRobot = !followRobot;
    lastFollowPose = robotPose ? { ...robotPose } : null;
    const button = document.getElementById('btn-follow');
    button.classList.toggle('active', followRobot);
    button.setAttribute('aria-pressed', String(followRobot));
    if (followRobot && robotPose && controls3D) {
        const delta = new THREE.Vector3(robotPose.x - controls3D.target.x, robotPose.y - controls3D.target.y, 0);
        controls3D.target.add(delta);
        camera3D.position.add(delta);
    }
}
function updateFollowCamera() {
    if (!followRobot || !robotPose || !controls3D) return;
    if (lastFollowPose) {
        const delta = new THREE.Vector3(robotPose.x - lastFollowPose.x, robotPose.y - lastFollowPose.y, 0);
        controls3D.target.add(delta);
        camera3D.position.add(delta);
    }
    lastFollowPose = { ...robotPose };
}
function pointOnMap(event) {
    if (!renderer3D || !camera3D || !mapMesh3D) return null;
    const rect = renderer3D.domElement.getBoundingClientRect();
    const ray = new THREE.Raycaster();
    ray.setFromCamera(new THREE.Vector2((event.clientX - rect.left) / rect.width * 2 - 1, -(event.clientY - rect.top) / rect.height * 2 + 1), camera3D);
    return ray.intersectObject(mapMesh3D)[0]?.point || null;
}
function clearPlacementPreview() {
    disposeMesh(placementArrow);
    placementArrow = null;
}
function cancelPlacement() {
    if (placement && renderer3D?.domElement.hasPointerCapture(placement.pointerId))
        renderer3D.domElement.releasePointerCapture(placement.pointerId);
    placement = null;
    clearPlacementPreview();
}
function showPlacementArrow(x, y, yaw, mode) {
    if (!placementArrow) {
        placementArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(x, y, 0.15), 0.8, mode === 'goal' ? 0xf4bd62 : 0x63e6a5, 0.22, 0.14);
        scene3D.add(placementArrow);
    }
    placementArrow.position.set(x, y, 0.15);
    placementArrow.setDirection(new THREE.Vector3(Math.cos(yaw), Math.sin(yaw), 0));
}
function editHeading(mode, degrees) {
    const pose = mode === 'goal' ? pendingGoal : pendingInitPose;
    if (!pose || !Number.isFinite(Number(degrees))) return;
    pose.yaw = Number(degrees) * Math.PI / 180;
    document.getElementById(mode === 'goal' ? 'goal-coords' : 'init-coords').textContent = `X: ${pose.x.toFixed(3)} m  Y: ${pose.y.toFixed(3)} m  Yaw: ${Number(degrees).toFixed(1)}°`;
    const marker = mode === 'goal' ? goalMesh3D : initMesh3D;
    marker?.setDirection(new THREE.Vector3(Math.cos(pose.yaw), Math.sin(pose.yaw), 0));
}
function installViewportControls() {
    const canvas = renderer3D.domElement;
    canvas.addEventListener('pointerdown', event => {
        if (event.button !== 0 || !mapData) return;

        
        if (interactMode === 'edit_zone') {
            const point = pointOnMap(event);
            if (!point || !mapInfo) return;
            const gx = Math.floor((point.x - mapInfo.origin.position.x) / mapInfo.resolution);
            const gy = Math.floor((point.y - mapInfo.origin.position.y) / mapInfo.resolution);
            if (gx >= 0 && gx < mapData.w && gy >= 0 && gy < mapData.h) {
                canvas.setPointerCapture(event.pointerId);
                startZoneEditing(gx, gy);
            }
            return;
        }
        if (interactMode === 'edit_map') {
            const point = pointOnMap(event);
            if (!point || !mapInfo) return;
            const gx = Math.floor((point.x - mapInfo.origin.position.x) / mapInfo.resolution);
            const gy = Math.floor((point.y - mapInfo.origin.position.y) / mapInfo.resolution);
            if (gx >= 0 && gx < mapData.w && gy >= 0 && gy < mapData.h) {
                canvas.setPointerCapture(event.pointerId);
                startMapEditing(gx, gy, point.x, point.y);
            }
            return;
        }

        if (placement || interactMode === 'view' || currentTab !== 'nav') return;
        const point = pointOnMap(event);
        if (!point) return;
        placement = { x: point.x, y: point.y, yaw: 0, mode: interactMode, pointerId: event.pointerId };
        canvas.setPointerCapture(event.pointerId);
        showPlacementArrow(point.x, point.y, 0, interactMode);
        document.getElementById('interaction-hint').textContent = 'Position anchored · Hold and drag toward the robot front · Release to finish';
    });
    canvas.addEventListener('pointermove', event => {
        const point = pointOnMap(event);
        if (!point) return;
        document.getElementById('coord-readout').textContent = `map · X ${point.x.toFixed(2)} m · Y ${point.y.toFixed(2)} m`;

        if (interactMode === 'edit_map' && isMapEditing && mapInfo) {
            const gx = Math.floor((point.x - mapInfo.origin.position.x) / mapInfo.resolution);
            const gy = Math.floor((point.y - mapInfo.origin.position.y) / mapInfo.resolution);
            continueMapEditing(gx, gy, point.x, point.y);
            return;
        }

        if (!placement || placement.pointerId !== event.pointerId) return;
        if (Math.hypot(point.x - placement.x, point.y - placement.y) > 0.05)
            placement.yaw = Math.atan2(point.y - placement.y, point.x - placement.x);
        showPlacementArrow(placement.x, placement.y, placement.yaw, placement.mode);
        document.getElementById('interaction-hint').textContent = `X ${placement.x.toFixed(2)} m · Y ${placement.y.toFixed(2)} m · Yaw ${(placement.yaw * 180 / Math.PI).toFixed(1)}° · Release to finish`;
    });
    canvas.addEventListener('pointerup', event => {
        
        if (interactMode === 'edit_zone' && isZoneEditing) {
            const point = pointOnMap(event);
            if (point && mapInfo) {
                const gx = Math.floor((point.x - mapInfo.origin.position.x) / mapInfo.resolution);
                const gy = Math.floor((point.y - mapInfo.origin.position.y) / mapInfo.resolution);
                continueZoneEditing(gx, gy);
            }
            return;
        }
        
        if (interactMode === 'edit_zone' && isZoneEditing) {
            canvas.releasePointerCapture(event.pointerId);
            const point = pointOnMap(event);
            if (point && mapInfo) {
                const gx = Math.floor((point.x - mapInfo.origin.position.x) / mapInfo.resolution);
                const gy = Math.floor((point.y - mapInfo.origin.position.y) / mapInfo.resolution);
                finishZoneEditing(gx, gy);
            }
            return;
        }
        if (interactMode === 'edit_map' && isMapEditing) {
            const point = pointOnMap(event);
            const gx = (point && mapInfo) ? Math.floor((point.x - mapInfo.origin.position.x) / mapInfo.resolution) : (editStartCell ? editStartCell.gx : 0);
            const gy = (point && mapInfo) ? Math.floor((point.y - mapInfo.origin.position.y) / mapInfo.resolution) : (editStartCell ? editStartCell.gy : 0);
            const px = point ? point.x : (editStartCell ? editStartCell.x : 0);
            const py = point ? point.y : (editStartCell ? editStartCell.y : 0);
            finishMapEditing(gx, gy, px, py);
            if (canvas.hasPointerCapture(event.pointerId)) canvas.releasePointerCapture(event.pointerId);
            return;
        }

        if (!placement || placement.pointerId !== event.pointerId) return;
        const { x, y, yaw, mode } = placement;
        placement = null;
        if (canvas.hasPointerCapture(event.pointerId)) canvas.releasePointerCapture(event.pointerId);
        if (mode === 'goal') setGoalPose(x, y, yaw); else setInitialPose(x, y, yaw);
        updateNavReadyUI();
    });
    canvas.addEventListener('pointercancel', () => {
        if (isMapEditing) finishMapEditing(editStartCell?.gx || 0, editStartCell?.gy || 0, editStartCell?.x || 0, editStartCell?.y || 0);
        setMode('view');
    });
    canvas.addEventListener('lostpointercapture', () => {
        if (isMapEditing) finishMapEditing(editStartCell?.gx || 0, editStartCell?.gy || 0, editStartCell?.x || 0, editStartCell?.y || 0);
        if (placement) setMode('view');
    });
    new ResizeObserver(() => {
        const { clientWidth: width, clientHeight: height } = wrap;
        if (!width || !height) return;
        camera3D.aspect = width / height;
        camera3D.updateProjectionMatrix();
        renderer3D.setSize(width, height);
    }).observe(wrap);
}
window.addEventListener('keydown', event => {
    if (/INPUT|SELECT|TEXTAREA/.test(event.target.tagName)) return;
    if (event.key === 'Escape') setMode('view');
    if (event.key.toLowerCase() === 'f') resetView();
    if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === 'z') {
        if (event.shiftKey) redoMapEdit(); else undoMapEdit();
        event.preventDefault();
    }
    if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === 'y') {
        redoMapEdit();
        event.preventDefault();
    }
});
window.addEventListener('DOMContentLoaded', () => {
    // Render the workspace before connection; no robot command is sent by initialization.
    try { init3DViewer(); updateNavReadyUI(); }
    catch (error) { showNotice(`3D viewer unavailable: ${error.message}. Check that Three.js loaded, then reload.`); }
});
setInterval(() => {
    if (!wsConnected) return;
    if (currentTab === 'nav') checkNav2Status();
    else if (currentTab === 'slam') checkSlamStatus();
}, 6000);
setInterval(() => {
    const topics = ['/map', '/robot_pose_web', '/scan', '/plan'];
    const labels = topics.map(topic => {
        const age = topicLastSeen[topic] ? Math.floor((Date.now() - topicLastSeen[topic]) / 1000) : null;
        return `${topic}: ${age === null ? 'waiting' : `${age}s ago`}`;
    });
    document.getElementById('data-health').textContent = labels.join('\n');
}, 1000);

function updateCostmapRegion(name, update) {
    const grid = cachedCostmaps[name];
    if (!grid || !update.width || !update.height) return;
    if (update.x < 0 || update.y < 0 || update.x + update.width > grid.info.width || update.y + update.height > grid.info.height) return;
    const decode = data => typeof data === 'string' ? Array.from(atob(data), char => char.charCodeAt(0) > 127 ? char.charCodeAt(0) - 256 : char.charCodeAt(0)) : data;
    grid.data = decode(grid.data);
    const values = decode(update.data);
    if (!values || values.length !== update.width * update.height) return;
    for (let y = 0; y < update.height; y++) for (let x = 0; x < update.width; x++)
        grid.data[(update.y + y) * grid.info.width + update.x + x] = values[y * update.width + x];
    if (name === 'global' && showGlobalCostmap) toggleGlobalCostmap();
    if (name === 'local' && showLocalCostmap) toggleLocalCostmap();
}

function setCostmapDetail(value) {
    costmapMinVisible = Math.max(1, Math.min(98, Number(value) || 1));
    document.getElementById('costmap-detail-value').textContent = costmapMinVisible === 1 ? 'All costs · RViz' : `Show costs ≥ ${costmapMinVisible} · display only`;
    toggleGlobalCostmap();
    toggleLocalCostmap();
}
function receiveCostmapTransforms(message, isStatic) {
    if (typeof THREE === 'undefined') return;
    for (const item of message.transforms || []) {
        const parent = item.header?.frame_id?.replace(/^\//, '');
        const child = item.child_frame_id?.replace(/^\//, '');
        if (!parent || !child || !item.transform) continue;
        const t = item.transform.translation, q = item.transform.rotation;
        const matrix = new THREE.Matrix4().compose(new THREE.Vector3(t.x, t.y, t.z), new THREE.Quaternion(q.x, q.y, q.z, q.w), new THREE.Vector3(1, 1, 1));
        costmapTransforms.set(child, { parent, matrix, isStatic, seen: Date.now() });
    }
}
function costmapFrameToMap(source) {
    if (source === 'map') return new THREE.Matrix4();
    const queue = [{ frame: source, matrix: new THREE.Matrix4() }], visited = new Set([source]);
    for (let i = 0; i < queue.length; i++) {
        const current = queue[i];
        for (const [child, edge] of costmapTransforms) {
            if (!edge.isStatic && Date.now() - edge.seen > 5000) continue;
            let next, step;
            if (current.frame === child) { next = edge.parent; step = edge.matrix; }
            else if (current.frame === edge.parent) { next = child; step = edge.matrix.clone().invert(); }
            else continue;
            if (visited.has(next)) continue;
            const matrix = step.clone().multiply(current.matrix);
            if (next === 'map') return matrix;
            visited.add(next); queue.push({ frame: next, matrix });
        }
    }
    return null;
}
function positionCostmap(mesh, name) {
    const data = mesh.userData.costmap;
    if (!data) return true;
    const transform = costmapFrameToMap(data.frame);
    const status = document.getElementById(`${name}-frame-status`);
    if (!transform) {
        status.textContent = `Waiting for TF: ${data.frame || 'unknown'} → map`;
        return false;
    }
    status.textContent = data.frame === 'map' ? 'Frame: map' : `Frame: ${data.frame} → map`;
    const p = data.info.origin.position, q = data.info.origin.orientation;
    const origin = new THREE.Matrix4().compose(new THREE.Vector3(p.x, p.y, p.z || 0), new THREE.Quaternion(q.x, q.y, q.z, q.w), new THREE.Vector3(1, 1, 1));
    const center = new THREE.Matrix4().makeTranslation(data.info.width * data.info.resolution / 2, data.info.height * data.info.resolution / 2, data.zOff);
    mesh.matrixAutoUpdate = false;
    mesh.matrix.copy(transform.multiply(origin).multiply(center));
    mesh.matrixWorldNeedsUpdate = true;
    return true;
}
