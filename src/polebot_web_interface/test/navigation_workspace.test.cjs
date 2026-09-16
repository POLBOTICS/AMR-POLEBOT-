// Run with node --test test/navigation_workspace.test.cjs (no ROS or robot required).
const test = require('node:test');
const assert = require('node:assert/strict');
const vm = require('node:vm');
const fs = require('node:fs');
const path = require('node:path');
function harness() {
    const nodes = new Map();
    const element = id => {
        if (!nodes.has(id)) nodes.set(id, { textContent: '', value: '0', disabled: false, style: {}, classList: { toggle() {}, add() {}, remove() {} }, querySelector: () => element(id + '-status') });
        return nodes.get(id);
    };
    const sent = [];
    class Socket { static OPEN = 1; constructor() { this.readyState = 1; } send(data) { sent.push(JSON.parse(data)); } }
    const context = vm.createContext({ console, Date, Math, Number, JSON, Map, Promise, setTimeout, clearTimeout, setInterval() {}, WebSocket: Socket,
        window: { location: { hostname: 'localhost' }, addEventListener() {} }, location: { protocol: 'http:' },
        document: { getElementById: element, querySelector: element }, navigator: {} });
    for (const file of ['workspace.js', 'app.js']) vm.runInContext(fs.readFileSync(path.join(__dirname, '../www/NavDashboard', file), 'utf8'), context);
    return { run: code => vm.runInContext(code, context), sent, element };
}
test('goal sends an action with chosen heading and cancellation targets its request ID', () => {
    const h = harness();
    h.run('wsConnected = true; nav2Ready = true; pendingGoal = {x: 2, y: -1, yaw: Math.PI/2}; sendGoal();');
    const goal = h.sent.find(m => m.op === 'send_action_goal');
    assert.equal(goal.action, '/navigate_to_pose');
    assert.equal(goal.action_type, 'nav2_msgs/action/NavigateToPose');
    assert.ok(Math.abs(goal.args.pose.pose.orientation.z - Math.SQRT1_2) < 1e-10);
    assert.equal(h.element('nav-status-value').textContent, 'SENDING');
    h.run('cancelNav()');
    const cancel = h.sent.at(-1);
    assert.equal(cancel.op, 'cancel_action_goal');
    assert.equal(cancel.id, goal.id);
    assert.equal(h.element('nav-status-value').textContent, 'CANCELING');
    h.run(`handleMessage(${JSON.stringify({op:'action_result', id:goal.id, result:true,status:5,values:{}})})`);
    assert.equal(h.element('nav-status-value').textContent, 'CANCELED');
    assert.equal(h.element('btn-cancel').disabled, true);
});
test('disconnected or inactive Nav2 cannot send a goal', () => {
    const h = harness();
    h.run('pendingGoal = {x:1,y:2,yaw:0}; sendGoal(); wsConnected = true; sendGoal();');
    assert.equal(h.sent.length, 0);
});
test('feedback from another request cannot overwrite current goal; rejected result is failure', () => {
    const h = harness();
    h.run('wsConnected = true; nav2Ready = true; pendingGoal = {x:0,y:0,yaw:0}; sendGoal();');
    const id = h.sent[0].id;
    h.run(`handleMessage(${JSON.stringify({op:'action_feedback',id:'other',values:{distance_remaining:999}})})`);
    assert.equal(h.element('nav-distance').textContent, '');
    h.run(`handleMessage(${JSON.stringify({op:'action_feedback',id,values:{distance_remaining:2.25,estimated_time_remaining:{sec:10},number_of_recoveries:1}})})`);
    assert.equal(h.element('nav-distance').textContent, '2.25 m');
    h.run(`handleMessage(${JSON.stringify({op:'action_result',id,result:false,status:0,values:'Goal rejected'})})`);
    assert.equal(h.element('nav-status-value').textContent, 'FAILED');
    assert.equal(h.element('nav-status-sub').textContent, 'Goal rejected');
});
test('initial pose preserves covariance and publishes selected yaw', () => {
    const h = harness();
    h.run('wsConnected = true; lifecycleStates.amcl = true; pendingInitPose = {x:1,y:2,yaw:0}; sendInitialPose();');
    const msg = h.sent[0].msg;
    assert.equal(msg.pose.pose.orientation.w, 1);
    assert.equal(msg.pose.covariance.length, 36);
    assert.equal(msg.pose.covariance[0], 0.25);
    assert.equal(msg.pose.covariance[35], 0.068);
});
test('lifecycle readiness requires every managed node to be active', async () => {
    const h = harness();
    h.run('wsConnected = true;');
    const check = h.run('checkNav2Status()');
    for (const request of h.sent) {
        const state = request.service === '/planner_server/get_state' ? {id:2,label:'inactive'} : {id:3,label:'active'};
        h.run(`handleMessage(${JSON.stringify({op:'service_response',id:request.id,result:true,values:{current_state:state}})})`);
    }
    await check;
    assert.equal(h.run('nav2Ready'), false);
    assert.equal(h.run('serviceRequests.size'), 0);
});
test('incremental costmap updates preserve cells outside the update rectangle', () => {
    const h = harness();
    h.run("cachedCostmaps.global = {info:{width:3,height:3},data:Array(9).fill(0)}; updateCostmapRegion('global',{x:1,y:1,width:2,height:1,data:[75,100]});");
    assert.deepEqual(JSON.parse(h.run('JSON.stringify(cachedCostmaps.global.data)')), [0,0,0,0,75,100,0,0,0]);
    h.run("updateCostmapRegion('global',{x:2,y:2,width:2,height:1,data:[1,1]});");
    assert.deepEqual(JSON.parse(h.run('JSON.stringify(cachedCostmaps.global.data)')), [0,0,0,0,75,100,0,0,0]);
});
test('disconnect invalidates ownership and blocks automatic goal replay', () => {
    const h = harness();
    h.run('wsConnected=true; nav2Ready=true; pendingGoal={x:0,y:0,yaw:0}; sendGoal(); ws.onclose();');
    assert.equal(h.run('navigationUncertain'), true);
    assert.equal(h.run('activeActionId'), null);
    assert.equal(h.run('pendingGoal'), null);
    assert.equal(h.element('nav-status-value').textContent, 'UNKNOWN');
    const count=h.sent.length;
    h.run('wsConnected=true; nav2Ready=true; pendingGoal={x:0,y:0,yaw:0}; sendGoal();');
    assert.equal(h.sent.length, count);
});
test('correlated send rejection clears ownership and enables a corrected retry', () => {
    const h=harness();
    h.run('wsConnected=true; nav2Ready=true; pendingGoal={x:1,y:2,yaw:0}; sendGoal();');
    const id=h.sent[0].id;
    h.run(`handleMessage(${JSON.stringify({op:'status',level:'error',id,msg:'Unsupported action request'})})`);
    assert.equal(h.run('activeActionId'), null);
    assert.equal(h.element('nav-status-value').textContent, 'FAILED');
    assert.equal(h.element('btn-send-goal').disabled, false);
});
test('incremental costmap transport is unthrottled', () => {
    const h=harness();h.run('subscribeTopics();');
    const patches=h.sent.filter(m => m.topic?.endsWith('/costmap_updates'));
    assert.equal(patches.length, 2);
    assert.ok(patches.every(m=>m.throttle_rate===0));
});
test('costmap uses RViz OccupancyGrid palette, not raw-cost encoding', () => {
    const h=harness();
    const color=v=>JSON.parse(h.run(`JSON.stringify(costmapColor(${v}))`));
    assert.deepEqual(color(99),[0,255,255,255]);
    assert.deepEqual(color(100),[255,0,255,255]);
    assert.deepEqual(color(50),[127,0,128,255]);
    for (const value of [-1,0,101,254,255]) assert.equal(color(value)[3],0);
});
test('heading edits rotate the final marker without creating a second preview', () => {
    const h=harness();
    h.run(`var directions=[]; var THREE={Vector3:class {constructor(x,y,z){this.x=x;this.y=y;this.z=z;}}}; pendingGoal={x:1,y:2,yaw:0}; goalMesh3D={setDirection(v){directions.push(v);}}; editHeading('goal',90);`);
    const direction=JSON.parse(h.run('JSON.stringify(directions[0])'));
    assert.ok(Math.abs(direction.x)<1e-10);
    assert.equal(direction.y,1);
    assert.equal(h.run('placementArrow'),null);
});
