// Live View (live.html): 지도 위 로봇 이동 현황 + 로봇별 주행 영상(/<ns>/camera/rec). 보기 전용 — 아무것도 발행하지 않는다.
//  - 로봇 목록·지도: FMS 백엔드(/robots, /maps/<name>)   지도 이름은 ?map= 또는 관제 GUI 가 저장한 localStorage 'fms.map'
//  - 위치: /fleet/robot_status (Nav2·차선 공통), 차선 주행 중이면 /<ns>/lane_status 의 상태·목표도 표시
const ROSBRIDGE_URL = 'ws://localhost:9090';
const BACKEND_URL = 'http://localhost:8000';
const ROBOT_COLORS = ['#38bdf8', '#f59e0b', '#a78bfa', '#34d399', '#f472b6'];   // app.js 와 같은 순서·색
const TRAIL_MS = 60000;
const displayName = (id) => id.toUpperCase().replace('_', '-');
const qs = new URLSearchParams(location.search);

let ros = null, rosOnline = false, handles = [];
let selected = qs.get('robot');
const robots = {};          // id -> { id, ns, color, status, statusAt, lane, trail: [[t,x,y]], cam: {...} }
const map = { meta: null, bmp: null, w: 0, h: 0 };
const canvas = document.getElementById('map'), ctx = canvas.getContext('2d');

// ---------- 지도 (app.js 의 parsePgm / buildMapBitmap 과 같은 규칙) ----------
function parsePgm(buf) {
    const u = new Uint8Array(buf);
    let i = 0;
    const tok = () => {
        while (i < u.length) {
            if (u[i] === 35) { while (i < u.length && u[i] !== 10) i++; }
            else if (u[i] <= 32) i++;
            else break;
        }
        let t = '';
        while (i < u.length && u[i] > 32) t += String.fromCharCode(u[i++]);
        return t;
    };
    if (tok() !== 'P5') throw new Error('P5(PGM) 형식이 아님');
    const w = parseInt(tok(), 10), h = parseInt(tok(), 10);
    tok(); i++;
    return { w, h, data: u.subarray(i, i + w * h) };
}

function buildBitmap(w, h, gray, meta) {
    const off = document.createElement('canvas');
    off.width = w; off.height = h;
    const c = off.getContext('2d'), img = c.createImageData(w, h);
    for (let k = 0; k < w * h; k++) {
        const occ = meta.negate ? gray[k] / 255 : (255 - gray[k]) / 255;
        const rgb = occ > meta.occupied_thresh ? [15, 23, 42] : occ < meta.free_thresh ? [226, 232, 240] : [100, 116, 139];
        img.data.set([rgb[0], rgb[1], rgb[2], 255], k * 4);
    }
    c.putImageData(img, 0, 0);
    return off;
}

async function loadMap() {
    let name = qs.get('map');
    if (!name) { try { name = localStorage.getItem('fms.map'); } catch (e) { /* 저장소 없음 */ } }
    const msg = document.getElementById('map-msg');
    try {
        if (!name) {
            const list = await (await fetch(`${BACKEND_URL}/maps`)).json();
            name = list.length ? list[0].name : null;
        }
        if (!name) { msg.textContent = '지도가 없습니다 — 관제 GUI 에서 지도를 선택하세요'; return; }
        const meta = await (await fetch(`${BACKEND_URL}/maps/${encodeURIComponent(name)}`)).json();
        const res = await fetch(`${BACKEND_URL}/maps/${encodeURIComponent(name)}/image`);
        if (!res.ok) throw new Error(`이미지 HTTP ${res.status}`);
        let w, h, gray;
        if (meta.image.toLowerCase().endsWith('.pgm')) ({ w, h, data: gray } = parsePgm(await res.arrayBuffer()));
        else {
            const bmp = await createImageBitmap(await res.blob());
            w = bmp.width; h = bmp.height;
            const t = document.createElement('canvas'); t.width = w; t.height = h;
            const tc = t.getContext('2d'); tc.drawImage(bmp, 0, 0);
            const px = tc.getImageData(0, 0, w, h).data;
            gray = new Uint8Array(w * h);
            for (let k = 0; k < w * h; k++) gray[k] = px[k * 4];
        }
        Object.assign(map, { meta, w, h, bmp: buildBitmap(w, h, gray, meta) });
        document.getElementById('map-name').textContent = name;
        msg.textContent = '';
        draw();
    } catch (e) { msg.textContent = `지도를 불러오지 못했습니다: ${e.message} (백엔드 확인)`; }
}

function view() {
    const cw = canvas.clientWidth, ch = canvas.clientHeight;
    const scale = Math.min(cw / map.w, ch / map.h) * 0.96;
    return { scale, ox: (cw - map.w * scale) / 2, oy: (ch - map.h * scale) / 2 };
}
function w2s(x, y, v) {
    const m = map.meta;
    return [v.ox + (x - m.origin[0]) / m.resolution * v.scale, v.oy + (map.h - (y - m.origin[1]) / m.resolution) * v.scale];
}

function arrow(x, y, yaw, len, color, width) {
    ctx.strokeStyle = color; ctx.lineWidth = width;
    ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + Math.cos(-yaw) * len, y + Math.sin(-yaw) * len); ctx.stroke();
}

function pose(r) {   // 차선 노드의 위치가 더 자주 오므로 있으면 그것을 쓴다
    if (r.lane && Date.now() - r.lane.t < 3000 && r.lane.pose) return { x: r.lane.pose[0], y: r.lane.pose[1], yaw: r.lane.pose[2] };
    if (r.status && r.status.localized && r.status.state !== 'OFFLINE' && Date.now() - r.statusAt < 5000) return r.status;
    return null;
}

let drawQueued = false;
function draw() {
    if (drawQueued) return;
    drawQueued = true;
    requestAnimationFrame(() => { drawQueued = false; drawNow(); });
}
function drawNow() {
    const dpr = window.devicePixelRatio || 1, cw = canvas.clientWidth, ch = canvas.clientHeight;
    if (canvas.width !== Math.round(cw * dpr) || canvas.height !== Math.round(ch * dpr)) { canvas.width = Math.round(cw * dpr); canvas.height = Math.round(ch * dpr); }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, cw, ch);
    if (!map.bmp) return;
    const v = view();
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(map.bmp, v.ox, v.oy, map.w * v.scale, map.h * v.scale);
    const order = Object.values(robots).sort((a, b) => (a.id === selected) - (b.id === selected));   // 선택한 로봇을 맨 위에
    const now = Date.now();
    order.forEach(r => {
        const sel = r.id === selected, c = r.color;
        r.trail = r.trail.filter(p => now - p[0] < TRAIL_MS);
        if (r.trail.length > 1) {
            ctx.strokeStyle = c; ctx.globalAlpha = sel ? 0.9 : 0.45; ctx.lineWidth = sel ? 3 : 2;
            ctx.beginPath();
            r.trail.forEach((p, i) => { const [sx, sy] = w2s(p[1], p[2], v); i ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy); });
            ctx.stroke(); ctx.globalAlpha = 1;
        }
        const goal = r.lane && now - r.lane.t < 3000 && r.lane.goal && !['ARRIVED', 'IDLE', 'FAILED'].includes(r.lane.state) ? r.lane.goal : null;
        if (goal) {
            const [gx, gy] = w2s(goal[0], goal[1], v);
            ctx.strokeStyle = c; ctx.lineWidth = 2; ctx.setLineDash([4, 3]);
            ctx.beginPath(); ctx.arc(gx, gy, 9, 0, Math.PI * 2); ctx.stroke(); ctx.setLineDash([]);
        }
        const p = pose(r);
        if (!p) return;
        const [rx, ry] = w2s(p.x, p.y, v), rad = sel ? 10 : 7;
        if (sel) { ctx.strokeStyle = c; ctx.lineWidth = 2; ctx.globalAlpha = 0.5; ctx.beginPath(); ctx.arc(rx, ry, rad + 7, 0, Math.PI * 2); ctx.stroke(); ctx.globalAlpha = 1; }
        ctx.fillStyle = c; ctx.strokeStyle = '#0f172a'; ctx.lineWidth = 2;
        ctx.beginPath(); ctx.arc(rx, ry, rad, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
        arrow(rx, ry, p.yaw, rad + 10, c, 3);
        ctx.fillStyle = '#e2e8f0'; ctx.font = `${sel ? 'bold ' : ''}12px Inter, sans-serif`;
        ctx.fillText(displayName(r.id), rx + rad + 4, ry - rad);
    });
}
window.addEventListener('resize', draw);
setInterval(draw, 1000);   // 오래된 경로 지우기·OFFLINE 반영

// ---------- 영상 칸 ----------
function buildCams() {
    const box = document.getElementById('cams');
    box.innerHTML = '';
    Object.values(robots).forEach(r => {
        const el = document.createElement('div');
        el.className = 'cam'; el.style.setProperty('--c', r.color);
        el.innerHTML = `<div class="cam-head"><b>${displayName(r.id)}</b><span class="muted">로봇 시점</span><span class="st">--</span></div>
            <div class="cam-body"><img alt="${displayName(r.id)} 주행 영상"><span class="fps">FPS: --</span><span class="msg">영상 연결 중…</span></div>`;
        el.onclick = () => { selected = r.id; markSelected(); draw(); };
        box.appendChild(el);
        r.cam = { el, img: el.querySelector('img'), msg: el.querySelector('.msg'), fps: el.querySelector('.fps'), st: el.querySelector('.st'), times: [], last: 0 };
    });
    markSelected();
}
function markSelected() { Object.values(robots).forEach(r => r.cam && r.cam.el.classList.toggle('sel', r.id === selected)); }

function stateText(r) {
    if (r.lane && Date.now() - r.lane.t < 3000) return `LANE ${r.lane.state}${r.lane.detail ? ' · ' + r.lane.detail : ''}`;
    if (r.status && Date.now() - r.statusAt < 5000) return r.status.state + (r.status.battery_percent >= 0 ? ` · 🔋${Math.round(r.status.battery_percent)}%` : '');
    return 'OFFLINE';
}

setInterval(() => {      // 영상이 안 오면 이유를 보여 준다
    Object.values(robots).forEach(r => {
        if (!r.cam) return;
        r.cam.st.textContent = stateText(r);
        if (Date.now() - r.cam.last < 3000) return;
        r.cam.img.style.display = 'none'; r.cam.fps.textContent = 'FPS: --';
        const online = r.status && Date.now() - r.statusAt < 5000 && r.status.state !== 'OFFLINE';
        r.cam.msg.textContent = !rosOnline ? 'rosbridge(9090)에 연결되지 않았습니다'
            : !online ? '로봇이 꺼져 있거나 데이터가 오지 않습니다 (관제 GUI 에서 ON)'
            : '영상이 오지 않습니다 — 미션을 선택해 스택(Nav2 / Lane)이 켜져 있는지, 로봇 패키지가 배포됐는지(OFF→ON) 확인하세요';
    });
}, 1000);

// ---------- ROS ----------
function subscribe() {
    handles.forEach(t => { try { t.unsubscribe(); } catch (e) { /* 무시 */ } });
    handles = [];
    const sub = (name, type, cb, opt = {}) => { const t = new ROSLIB.Topic({ ros, name, messageType: type, ...opt }); t.subscribe(cb); handles.push(t); };
    sub('/fleet/robot_status', 'pinky_fms_interfaces/msg/RobotStatus', (m) => {
        const r = robots[m.robot_id];
        if (!r) return;
        r.status = m; r.statusAt = Date.now();
        if (m.localized && m.state !== 'OFFLINE' && !(r.lane && Date.now() - r.lane.t < 3000)) addTrail(r, m.x, m.y);
        draw();
    });
    Object.values(robots).forEach(r => {
        sub(`/${r.ns}/lane_status`, 'std_msgs/msg/String', (m) => {
            try { r.lane = { ...JSON.parse(m.data), t: Date.now() }; } catch (e) { return; }
            if (r.lane.pose) addTrail(r, r.lane.pose[0], r.lane.pose[1]);
            draw();
        });
        sub(`/${r.ns}/camera/rec`, 'sensor_msgs/msg/CompressedImage', (m) => {
            const c = r.cam;
            c.last = Date.now(); c.times.push(c.last); while (c.times.length > 10) c.times.shift();
            c.img.src = 'data:image/jpeg;base64,' + m.data;
            c.img.style.display = 'block'; c.msg.textContent = '';
            if (c.times.length > 1) c.fps.textContent = `FPS: ${((c.times.length - 1) * 1000 / (c.times[c.times.length - 1] - c.times[0])).toFixed(1)}`;
        }, { throttle_rate: 150, queue_length: 1 });
    });
}
function addTrail(r, x, y) {
    const t = r.trail[r.trail.length - 1];
    if (!t || Math.hypot(t[1] - x, t[2] - y) > 0.01) r.trail.push([Date.now(), x, y]);
}

function connectRos() {
    ros = new ROSLIB.Ros({ url: ROSBRIDGE_URL });
    const set = (on, txt) => { rosOnline = on; document.getElementById('ros-dot').classList.toggle('on', on); document.getElementById('ros-text').textContent = txt; };
    ros.on('connection', () => { set(true, 'rosbridge 연결됨'); subscribe(); });
    ros.on('error', () => set(false, 'rosbridge 연결 실패 — 관제 스택 확인'));
    ros.on('close', () => { set(false, 'rosbridge 끊김 — 다시 연결 중…'); setTimeout(connectRos, 3000); });
}

async function init() {
    let list = [];
    try { list = await (await fetch(`${BACKEND_URL}/robots`)).json(); }
    catch (e) { document.getElementById('cams').innerHTML = '<p class="muted">백엔드(8000)에서 로봇 목록을 받지 못했습니다</p>'; }
    list.forEach((x, i) => { robots[x.id] = { id: x.id, ns: x.namespace, color: ROBOT_COLORS[i % ROBOT_COLORS.length], status: null, statusAt: 0, lane: null, trail: [] }; });
    if (!robots[selected]) selected = list.length ? list[0].id : null;
    if (selected) document.title = `${displayName(selected)} · Pinky FMS Live View`;
    buildCams();
    loadMap();
    connectRos();
}
init();
