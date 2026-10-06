// ==========================================================
// Pinky FMS 웹 GUI
//  - 모니터링/주행 요청: rosbridge(ws://localhost:9090)로 ROS 2 와 직접 통신
//  - 로봇 ON/OFF·로그 파일: FMS 백엔드(http://localhost:8000)
// GUI 는 "운영자 입력 + 상태 표시"만 한다. 미션/조정 로직은 fleet_mission / fleet_coordinator 노드가 맡는다.
// ==========================================================
const ROSBRIDGE_URL = 'ws://localhost:9090';
const BACKEND_URL = 'http://localhost:8000';
const MAX_ROS_LOG = 300;

// 로봇 이름은 코드에 쓰지 않고 백엔드(/robots = robots.yaml)에서 읽어온다.
let netInfo = { discovery: 'multicast' };
let cfgInfo = { defaults: { user: 'pinky', mode: 'ssh' } };   // 백엔드 /config (로봇 추가 기본값)
let robotsLoaded = false;   // 백엔드 /network (robots.yaml 의 DDS 탐색 방식)
const robots = {};   // id -> { id, ns, ip, mode, proc, status, task, logs[], unreadErrors }

// ==========================================
// 1. ROS 연결 (끊기면 자동 재연결)
// ==========================================
const statusText = document.getElementById('connection-status');
const statusDot = document.getElementById('status-dot');
let ros = null;
let rosOnline = false;
let rosGeneration = 0;     // rosbridge 에 다시 연결될 때마다 증가 (재사용하는 발행자를 새로 만들기 위함)

function setRosIndicator(online, errored) {
    rosOnline = online;
    if (online) {
        statusText.textContent = 'ROS 2 Online';
        statusText.className = 'text-xs font-semibold text-emerald-400';
        statusDot.innerHTML = `<span class="animate-ping absolute inline-flex h-full w-full rounded-full bg-emerald-400 opacity-75"></span><span class="relative inline-flex rounded-full h-3 w-3 bg-emerald-500"></span>`;
    } else {
        statusText.textContent = errored ? 'Connection Error' : 'ROS Offline';
        statusText.className = 'text-xs font-semibold text-amber-400';
        statusDot.innerHTML = `<span class="relative inline-flex rounded-full h-3 w-3 bg-amber-500"></span>`;
    }
}

function connectRos() {
    ros = new ROSLIB.Ros({ url: ROSBRIDGE_URL });
    ros.on('connection', () => { rosGeneration++; setRosIndicator(true); setupRosTopics(); });
    ros.on('error', () => setRosIndicator(false, true));
    ros.on('close', () => { setRosIndicator(false); setTimeout(connectRos, 3000); });
}

// ==========================================
// 2. 공통 유틸 (모달, 토스트, HTTP)
// ==========================================
function openModal(modalId, contentId) {
    const modal = document.getElementById(modalId);
    const content = document.getElementById(contentId);
    modal.classList.remove('hidden');
    setTimeout(() => { modal.classList.remove('opacity-0'); content.classList.remove('scale-95'); }, 10);
}

function closeModal(modalId, contentId, callback) {
    const modal = document.getElementById(modalId);
    const content = document.getElementById(contentId);
    modal.classList.add('opacity-0');
    content.classList.add('scale-95');
    setTimeout(() => { modal.classList.add('hidden'); if (callback) callback(); }, 300);
}

function toast(msg, kind = 'info') {
    const colors = { info: 'border-cyan-600 text-cyan-200', ok: 'border-emerald-600 text-emerald-200', err: 'border-rose-600 text-rose-200' };
    const el = document.createElement('div');
    el.className = `bg-slate-800 border ${colors[kind]} rounded-lg px-4 py-2 text-xs shadow-xl max-w-xs`;
    el.textContent = msg;
    document.getElementById('toast-area').appendChild(el);
    setTimeout(() => el.remove(), 5000);
}

async function api(method, path, body) {
    const res = await fetch(BACKEND_URL + path, {
        method,
        headers: body ? { 'Content-Type': 'application/json' } : undefined,
        body: body ? JSON.stringify(body) : undefined,
    });
    const data = await res.json().catch(() => ({}));
    if (!res.ok) { const err = new Error(data.detail || `HTTP ${res.status}`); err.status = res.status; throw err; }
    return data;
}

const esc = (s) => String(s).replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const displayName = (id) => id.toUpperCase().replace('_', '-');

// ==========================================
// 3. ROS 구독/발행 (로봇 이름이 아니라 /fleet/* 의 robot_id 로 구분)
// ==========================================
let missionPub = null, globalCmdPub = null;
const topicHandles = [];

function setupRosTopics() {
    topicHandles.forEach(t => t.unsubscribe());
    topicHandles.length = 0;

    missionPub = new ROSLIB.Topic({ ros, name: '/fleet/mission_request', messageType: 'pinky_fms_interfaces/msg/MissionRequest' });
    globalCmdPub = new ROSLIB.Topic({ ros, name: '/fleet/global_cmd', messageType: 'std_msgs/msg/String' });

    const sub = (name, type, cb) => { const t = new ROSLIB.Topic({ ros, name, messageType: type }); t.subscribe(cb); topicHandles.push(t); };

    // 조정 층이 모아 올려주는 로봇 종합 상태 (OFFLINE / IDLE / BUSY, 배터리, 위치, heartbeat)
    sub('/fleet/robot_status', 'pinky_fms_interfaces/msg/RobotStatus', (m) => {
        const r = robots[m.robot_id];
        if (!r) return;
        r.status = m;
        r.statusAt = Date.now();
        requestMapDraw();
        updateCard(r);
        updateKpis();
    });

    // 미션 층이 올려주는 진행 상태
    sub('/fleet/mission_state', 'pinky_fms_interfaces/msg/TaskState', (m) => {
        const r = robots[m.robot_id];
        if (!r) { toast(`미션 ${m.mission_id}: ${m.state} - ${m.message}`, m.state === 'FAILED' ? 'err' : 'info'); return; }
        r.task = m;
        if (m.state === 'ASSIGNED' || m.state === 'RUNNING') r.goalPose = missionGoals[m.mission_id] || r.goalPose;
        else r.goalPose = null;
        requestMapDraw();
        if (m.state === 'FAILED') toast(`${displayName(r.id)} 미션 실패: ${m.message || '원인 불명'}`, 'err');
        if (m.state === 'SUCCEEDED') toast(`${displayName(r.id)} 목표 도착`, 'ok');
        updateCard(r);
    });

    // 로봇 로그(/rosout). logger 이름이 "<namespace>.<node>" 형태라 namespace 로 로봇을 구분한다.
    sub('/rosout', 'rcl_interfaces/msg/Log', (m) => {
        const r = Object.values(robots).find(x => m.name === x.ns || m.name.startsWith(x.ns + '.'));
        if (!r) return;
        r.logs.push({ t: Date.now(), level: m.level, name: m.name, msg: m.msg });
        if (r.logs.length > MAX_ROS_LOG) r.logs.shift();
        if (m.level >= 40) { r.unreadErrors++; updateKpis(); updateCard(r); }
        if (logRobotId === r.id && logTab === 'rosout') renderLog();
    });
}

// ==========================================
// 4. 로봇 카드 렌더링/갱신
// ==========================================
const STATE_STYLE = {
    OFFLINE: ['bg-slate-600/30 text-slate-400', 'border-slate-600'],
    STARTING: ['bg-amber-500/20 text-amber-400', 'border-amber-500'],
    IDLE: ['bg-sky-500/20 text-sky-400', 'border-sky-500'],
    BUSY: ['bg-emerald-500/20 text-emerald-400', 'border-emerald-500'],
};

function effectiveState(r) {
    const stale = !r.statusAt || Date.now() - r.statusAt > 4000;   // 조정 노드 자체가 안 오는 경우
    const st = (!stale && r.status) ? r.status.state : 'OFFLINE';
    if (st === 'OFFLINE' && r.proc) return 'STARTING';
    return st;
}

function renderFleet() {
    const container = document.getElementById('fleet-container');
    container.innerHTML = '';
    if (!Object.keys(robots).length) {
        container.innerHTML = `<li class="text-center text-slate-500 text-xs py-10 px-4 border border-dashed border-slate-700 rounded-xl">
            <i class="fa-solid fa-robot text-2xl text-slate-600 mb-3 block"></i>등록된 로봇이 없습니다.<br>
            위의 <b class="text-slate-300">로봇 추가</b>에서 ID(namespace), IP, 비밀번호를 입력하세요.</li>`;
        updateKpis();
        return;
    }
    Object.values(robots).forEach(r => {
        const n = displayName(r.id);
        container.insertAdjacentHTML('beforeend', `
        <li id="card-${r.id}" class="bg-slate-800 border-l-4 border-slate-600 rounded-xl p-3 shadow-md mb-3 transition-all">
            <div class="flex justify-between items-start mb-2">
                <h3 class="font-bold text-white text-sm">${n}
                    <span id="${r.id}-state" class="ml-2 text-[10px] px-1.5 py-0.5 rounded uppercase">--</span>
                    <span id="${r.id}-pingbadge" class="hidden ml-1 text-[10px] bg-slate-700 text-slate-300 px-1.5 py-0.5 rounded font-mono"></span>
                    <span id="${r.id}-connbadge" class="hidden ml-1 text-[10px] bg-amber-500/20 text-amber-400 px-1.5 py-0.5 rounded"></span>
                    <span id="${r.id}-errbadge" class="hidden ml-1 text-[10px] bg-rose-500/20 text-rose-400 px-1.5 py-0.5 rounded"></span>
                </h3>
                <div class="flex gap-1">
                    <button onclick="openLogModal('${r.id}')" class="bg-slate-900 hover:bg-slate-700 border border-slate-700 text-slate-400 hover:text-amber-400 px-2 py-1 rounded text-[10px] transition"><i class="fa-solid fa-terminal mr-1"></i>Log</button>
                    <button onclick="openDiagModal('${r.id}')" class="bg-slate-900 hover:bg-slate-700 border border-slate-700 text-slate-400 hover:text-cyan-400 px-2 py-1 rounded text-[10px] transition"><i class="fa-solid fa-network-wired mr-1"></i>Diag</button>
                    <button onclick="openCamModal('${n}')" class="bg-slate-900 hover:bg-slate-700 border border-slate-700 text-slate-400 hover:text-cyan-400 px-2 py-1 rounded text-[10px] transition" title="카메라 (미구현)"><i class="fa-solid fa-video"></i></button>
                    <button id="${r.id}-btn-del" onclick="deleteRobot('${r.id}')" class="hidden bg-slate-900 hover:bg-slate-700 border border-slate-700 text-slate-500 hover:text-rose-400 px-2 py-1 rounded text-[10px] transition" title="로봇 삭제 (OFF 상태에서만)"><i class="fa-solid fa-trash"></i></button>
                </div>
            </div>
            <p id="${r.id}-conn" class="text-[10px] text-slate-500 font-mono -mt-1 mb-2"></p>
            <div class="flex items-center gap-2 mb-2">
                <i id="${r.id}-battery-icon" class="fa-solid fa-battery-three-quarters text-slate-500 text-xs w-4"></i>
                <div class="flex-1 bg-slate-900 h-1.5 rounded-full overflow-hidden"><div id="${r.id}-battery-bar" class="bg-slate-500 h-full transition-all duration-500" style="width:0%"></div></div>
                <span id="${r.id}-battery-text" class="text-[10px] text-slate-400 font-mono w-8 text-right">--%</span>
            </div>
            <p id="${r.id}-pose" class="text-[10px] text-slate-500 font-mono mb-2">pose: --</p>
            <div class="bg-slate-900/50 rounded-lg p-2.5 border border-slate-700">
                <p class="text-[9px] text-slate-500 mb-1 font-bold uppercase"><i class="fa-solid fa-list-check mr-1"></i>Mission</p>
                <p id="${r.id}-task" class="text-[11px] text-slate-400">대기 중</p>
            </div>
            <div id="${r.id}-staged" class="hidden mt-2 flex items-center justify-between gap-2 text-[11px] rounded-lg px-2.5 py-1.5 border border-cyan-700/60 bg-cyan-500/10 text-cyan-300">
                <span id="${r.id}-staged-text" class="font-mono"></span>
                <button onclick="clearStaged('${r.id}')" class="text-slate-400 hover:text-rose-400 underline">해제</button>
            </div>
            <div class="mt-3 grid grid-cols-4 gap-1.5">
                <button id="${r.id}-btn-power" class="text-white text-[11px] py-1.5 rounded font-bold"></button>
                <button id="${r.id}-btn-initpose" onclick="startInitialPose('${r.id}')" class="text-[11px] py-1.5 rounded font-bold" title="맵에서 로봇의 현재 위치를 지정 (AMCL 초기 위치)"><i class="fa-solid fa-crosshairs mr-1"></i>초기위치</button>
                <button id="${r.id}-btn-goal" onclick="startGoalTool('${r.id}')" class="text-[11px] py-1.5 rounded font-bold" title="맵에서 이 로봇의 목표 지점을 지정 (출발은 '주행')"><i class="fa-solid fa-flag-checkered mr-1"></i>목표지정</button>
                <button id="${r.id}-btn-go" onclick="goOrPrompt('${r.id}')" class="text-[11px] py-1.5 rounded font-bold"><i class="fa-solid fa-play mr-1"></i>주행</button>
                <button id="${r.id}-btn-cancel" onclick="cancelMission('${r.id}')" class="hidden col-start-4 bg-rose-950/40 hover:bg-rose-900 border border-rose-900 text-rose-300 text-[11px] py-1.5 rounded font-bold"><i class="fa-solid fa-ban mr-1"></i>취소</button>
            </div>
        </li>`);
        updateCard(r);
    });
    updateKpis();
}

function updateCard(r) {
    const $ = (s) => document.getElementById(`${r.id}-${s}`);
    const card = document.getElementById(`card-${r.id}`);
    if (!card || !$('state')) return;

    const st = effectiveState(r);
    const [badge, border] = STATE_STYLE[st];
    $('state').className = `ml-2 text-[10px] px-1.5 py-0.5 rounded uppercase ${badge}`;
    $('state').textContent = st;
    card.className = card.className.replace(/border-(slate|amber|sky|emerald)-\d+/, border);

    const pb = $('pingbadge');
    if (pb) {
        const pg = r.ping;
        pb.classList.toggle('hidden', !(r.proc && pg && pg.monitoring));
        if (r.proc && pg && pg.monitoring) {
            const bad = pg.last_rtt === null || pg.last_rtt === undefined;
            pb.textContent = bad ? 'ping ✕' : `ping ${pg.last_rtt.toFixed(0)}ms`;
            pb.className = `ml-1 text-[10px] px-1.5 py-0.5 rounded font-mono ${bad ? 'bg-rose-500/20 text-rose-400' : pg.loss_pct > 0 ? 'bg-amber-500/20 text-amber-400' : 'bg-slate-700 text-slate-300'}`;
        }
    }
    $('conn').textContent = `/${r.ns}  ·  ` + (r.mode === 'local' ? 'local (가짜 로봇)' : `${r.user || '?'}@${r.ip || 'IP 없음'}`) + (r.preset ? '  ·  고정' : '');
    const connTxt = r.proc && r.authRequired ? '재접속 필요' : (r.proc && r.reachable === false ? 'SSH 끊김' : '');
    $('connbadge').classList.toggle('hidden', !connTxt);
    $('connbadge').textContent = connTxt;
    $('btn-del').classList.toggle('hidden', !!r.preset || !!r.proc);
    const eb = $('errbadge');
    eb.classList.toggle('hidden', r.unreadErrors === 0);
    eb.textContent = `ERR ${r.unreadErrors}`;

    const online = st === 'IDLE' || st === 'BUSY';
    const batt = online && r.status && r.status.battery_percent >= 0 ? Math.round(r.status.battery_percent) : null;
    if (batt === null) {
        $('battery-bar').style.width = '0%'; $('battery-text').textContent = '--%';
    } else {
        const low = batt <= 20;
        $('battery-bar').style.width = batt + '%';
        $('battery-bar').className = `${low ? 'bg-rose-500' : 'bg-emerald-500'} h-full transition-all duration-500`;
        $('battery-icon').className = `fa-solid ${low ? 'fa-battery-quarter text-rose-500' : 'fa-battery-three-quarters text-emerald-500'} text-xs w-4`;
        $('battery-text').textContent = batt + '%';
        $('battery-text').className = `text-[10px] font-mono w-8 text-right ${low ? 'text-rose-500 font-bold' : 'text-slate-400'}`;
    }
    const localized = online && r.status.localized;
    $('pose').textContent = !online ? 'pose: --'
        : localized ? `pose: x ${r.status.x.toFixed(2)}  y ${r.status.y.toFixed(2)}  yaw ${(r.status.yaw * 180 / Math.PI).toFixed(0)}°`
        : 'pose: 위치 미확인 (맵에서 초기 위치를 지정하세요)';
    $('pose').className = `text-[10px] font-mono mb-2 ${online && !localized ? 'text-amber-400' : 'text-slate-500'}`;
    const t = r.task;
    if (t && (t.state === 'ASSIGNED' || t.state === 'RUNNING')) {
        $('task').className = 'text-[11px] text-emerald-400';
        $('task').textContent = `[${t.mission_id}] ${t.state}` + (t.state === 'RUNNING' && t.distance_remaining > 0 ? ` · 남은 거리 ${t.distance_remaining.toFixed(2)} m` : '');
    } else if (t) {
        $('task').className = `text-[11px] ${t.state === 'SUCCEEDED' ? 'text-sky-400' : 'text-rose-400'}`;
        $('task').textContent = `[${t.mission_id}] ${t.state}` + (t.message ? ` - ${t.message}` : '');
    }

    const power = $('btn-power');
    if (r.proc) {
        power.className = 'flex-1 bg-slate-700 hover:bg-rose-800 text-white text-[11px] py-1.5 rounded font-bold';
        power.innerHTML = '<i class="fa-solid fa-power-off mr-1"></i>OFF';
        power.onclick = () => turnOff(r.id);
    } else {
        power.className = 'flex-1 bg-emerald-700 hover:bg-emerald-600 text-white text-[11px] py-1.5 rounded font-bold';
        power.innerHTML = '<i class="fa-solid fa-power-off mr-1"></i>ON';
        power.onclick = () => openOnModal(r.id);
    }
    const sg = r.staged;
    $('staged').classList.toggle('hidden', !sg);
    if (sg) $('staged-text').textContent = `목표 대기: x ${sg.x.toFixed(2)}, y ${sg.y.toFixed(2)}, ${(sg.yaw * 180 / Math.PI).toFixed(0)}°`;

    // 버튼 상태: 초기위치 → 목표지정 → 주행 순서로 안내한다
    const mapReady = !!mapState.meta;
    const armed = (type) => !!mapTool && mapTool.type === type && mapTool.robotId === r.id;
    const canGo = st === 'IDLE' && localized && !!sg;
    const base = 'text-[11px] py-1.5 rounded font-bold transition';
    const style = (btn, enabled, kind, armedNow) => {
        btn.disabled = !enabled;
        const color = armedNow ? 'bg-amber-500 text-slate-900'
            : kind === 'next' ? 'bg-cyan-700 hover:bg-cyan-600 text-white ring-2 ring-cyan-300'
            : kind === 'warn' ? 'bg-slate-700 hover:bg-slate-600 text-amber-300 ring-1 ring-amber-500'
            : 'bg-slate-700 hover:bg-slate-600 text-slate-200';
        btn.className = `${base} ${color} ${enabled ? '' : 'opacity-40 cursor-not-allowed'}`;
    };
    const bi = $('btn-initpose'), bg = $('btn-goal'), go = $('btn-go');
    style(bi, online && mapReady, !localized && online ? 'warn' : '', armed('initpose'));
    style(bg, online && mapReady, localized && !sg ? 'next' : '', armed('goal'));
    bi.title = !online ? '로봇이 켜져 있어야 합니다' : !mapReady ? '먼저 맵을 업로드/선택하세요' : '맵에서 로봇의 현재 위치를 지정 (AMCL 초기 위치)';
    bg.title = !online ? '로봇이 켜져 있어야 합니다' : !mapReady ? '먼저 맵을 업로드/선택하세요' : "맵에서 이 로봇의 목표 지점을 지정 (출발은 '주행')";
    go.className = `${base} ${canGo ? 'bg-cyan-700 hover:bg-cyan-600 text-white ring-2 ring-cyan-300' : 'bg-slate-700 text-slate-300 opacity-40 cursor-not-allowed'}`;
    go.disabled = !canGo;
    go.classList.toggle('hidden', st === 'BUSY');
    go.title = st !== 'IDLE' ? '대기(IDLE) 상태에서만 출발합니다' : !localized ? "먼저 '초기위치'를 지정하세요" : !sg ? "먼저 '목표지정'으로 목표를 찍으세요" : '이 로봇만 출발';
    $('btn-cancel').classList.toggle('hidden', st !== 'BUSY');
}

function updateKpis() {
    const list = Object.values(robots);
    const states = list.map(effectiveState);
    document.getElementById('kpi-total').textContent = list.length;
    document.getElementById('kpi-active').textContent = states.filter(s => s === 'BUSY').length;
    document.getElementById('kpi-idle').textContent = states.filter(s => s === 'IDLE').length;
    const alerts = list.filter(r => r.unreadErrors > 0).length;
    document.getElementById('kpi-alerts').textContent = alerts;
    document.getElementById('kpi-alerts-label').textContent = alerts ? 'Check logs' : 'Clear';
    const ages = list.filter((r, i) => (states[i] === 'IDLE' || states[i] === 'BUSY') && r.status).map(r => r.status.last_seen_sec_ago);
    document.getElementById('kpi-hb').textContent = ages.length ? (ages.reduce((a, b) => a + b, 0) / ages.length).toFixed(1) : '--';
}

// ==========================================
// 5. ON / OFF (백엔드)
// ==========================================
let onRobotId = null, onReconnect = false;

window.openOnModal = function (id, reconnect) {
    onRobotId = id;
    onReconnect = !!reconnect;
    const r = robots[id];
    document.getElementById('on-title').textContent = onReconnect ? '재접속' : '로봇 구동';
    document.getElementById('on-submit').textContent = onReconnect ? '재접속' : 'ON';
    document.getElementById('on-robot-name').textContent = displayName(id);
    document.getElementById('on-ip').value = r.ip || '';
    document.getElementById('on-user').value = r.user || cfgInfo.defaults.user || '';
    document.getElementById('on-pw').value = '';
    document.getElementById('on-key').checked = false;
    document.getElementById('on-error').classList.add('hidden');
    document.getElementById('on-local-note').classList.toggle('hidden', r.mode !== 'local');
    openModal('on-modal', 'on-modal-content');
};
window.closeOnModal = function () { closeModal('on-modal', 'on-modal-content', () => { document.getElementById('on-pw').value = ''; }); };

window.submitOn = async function () {
    const id = onRobotId, btn = document.getElementById('on-submit'), err = document.getElementById('on-error');
    const label = btn.textContent;
    btn.disabled = true; btn.textContent = '접속 중...'; err.classList.add('hidden');
    try {
        const ip = document.getElementById('on-ip').value.trim(), user = document.getElementById('on-user').value.trim();
        const res = await api('POST', `/robots/${id}/on`, {
            ip: ip || null, user: user || null,
            password: document.getElementById('on-pw').value || null,
            install_key: document.getElementById('on-key').checked,
        });
        Object.assign(robots[id], { proc: res.running, authRequired: false, reachable: true, ip: ip || robots[id].ip, user: user || robots[id].user });
        toast(`${displayName(id)}: ${res.note}`, 'ok');
        closeOnModal();
        updateCard(robots[id]); updateKpis();
    } catch (e) {
        err.textContent = e.message; err.classList.remove('hidden');
    } finally {
        document.getElementById('on-pw').value = '';
        btn.disabled = false; btn.textContent = label;
    }
};

window.turnOff = async function (id) {
    if (!confirm(`${displayName(id)} 을(를) 종료할까요?`)) return;
    try {
        const res = await api('POST', `/robots/${id}/off`);
        robots[id].proc = res.running;
        toast(`${displayName(id)} 종료`, 'ok');
        updateCard(robots[id]); updateKpis();
    } catch (e) {
        if (e.status === 401) {
            toast(`${displayName(id)}: 비밀번호가 필요합니다. 재접속한 뒤 다시 OFF 하세요`, 'err');
            openOnModal(id, true);
        } else toast(`OFF 실패: ${e.message}`, 'err');
    }
};

// ---- 로봇 추가 / 삭제 ----
function nextRobotId() {
    for (let i = 1; i < 100; i++) {
        const id = `amr_${String(i).padStart(2, '0')}`;
        if (!robots[id]) return id;
    }
    return '';
}

window.openAddModal = function () {
    const id = nextRobotId();
    document.getElementById('add-id').value = id;
    document.getElementById('add-id-preview').textContent = id || 'amr_01';
    document.getElementById('add-ip').value = '';
    document.getElementById('add-user').value = cfgInfo.defaults.user || '';
    document.getElementById('add-pw').value = '';
    document.getElementById('add-key').checked = false;
    document.getElementById('add-start').checked = true;
    document.getElementById('add-error').classList.add('hidden');
    document.getElementById('add-local-note').classList.toggle('hidden', cfgInfo.defaults.mode !== 'local');
    openModal('add-modal', 'add-modal-content');
    setTimeout(() => document.getElementById(cfgInfo.defaults.mode === 'local' ? 'add-id' : 'add-ip').focus(), 50);
};
window.closeAddModal = function () { closeModal('add-modal', 'add-modal-content', () => { document.getElementById('add-pw').value = ''; }); };
document.getElementById('add-id').addEventListener('input', (e) => {
    document.getElementById('add-id-preview').textContent = e.target.value.trim() || 'amr_01';
});

window.submitAdd = async function () {
    const btn = document.getElementById('add-submit'), err = document.getElementById('add-error');
    const id = document.getElementById('add-id').value.trim();
    btn.disabled = true; btn.textContent = '접속 확인 중...'; err.classList.add('hidden');
    try {
        const res = await api('POST', '/robots', {
            id,
            ip: document.getElementById('add-ip').value.trim() || null,
            user: document.getElementById('add-user').value.trim() || null,
            password: document.getElementById('add-pw').value || null,
            install_key: document.getElementById('add-key').checked,
            start: document.getElementById('add-start').checked,
        });
        await loadRobots();
        if (robots[id]) robots[id].proc = !!res.running;
        toast(`${displayName(id)} 추가` + (res.note ? ` · ${res.note}` : ''), 'ok');
        closeAddModal();
        if (robots[id]) { updateCard(robots[id]); updateKpis(); }
    } catch (e) {
        err.textContent = e.message; err.classList.remove('hidden');
    } finally {
        document.getElementById('add-pw').value = '';
        btn.disabled = false; btn.textContent = '추가';
    }
};

window.deleteRobot = async function (id) {
    if (!confirm(`${displayName(id)} 을(를) 목록에서 지울까요?`)) return;
    try {
        await api('DELETE', `/robots/${id}`);
    } catch (e) {
        if (e.status !== 409) { toast(`삭제 실패: ${e.message}`, 'err'); return; }
        if (!confirm(`${e.message}\n\n로봇에 접속할 수 없어 OFF 가 안 되는 경우에만 강제로 지우세요. 강제로 지울까요?`)) return;
        try { await api('DELETE', `/robots/${id}?force=true`); } catch (e2) { toast(`삭제 실패: ${e2.message}`, 'err'); return; }
    }
    toast(`${displayName(id)} 삭제`, 'ok');
    await loadRobots();
};

async function pollProcStatus() {
    const badge = document.getElementById('backend-status');
    const setBadge = (ok) => {
        badge.textContent = ok ? 'Backend: OK' : 'Backend: Offline';
        badge.className = `text-[10px] px-2 py-0.5 rounded ${ok ? 'bg-emerald-500/20 text-emerald-400' : 'bg-rose-500/20 text-rose-400'}`;
    };
    if (!await loadRobots(false)) { setBadge(false); return; }
    let ok = true;
    for (const r of Object.values(robots)) {
        try {
            const st = await api('GET', `/robots/${r.id}/status`);
            r.proc = st.running; r.authRequired = !!st.auth_required; r.reachable = st.reachable;
            r.ping = r.proc ? await api('GET', `/robots/${r.id}/ping`) : null;
            updateCard(r);
        } catch (e) {
            if (!e.status) ok = false;          // 404 등은 목록이 막 바뀐 경우라 무시
        }
    }
    setBadge(ok);
    updateKpis();
}

// ==========================================
// 6. 주행 / 취소 (미션 요청만 보낸다. 어떤 로봇이 어떻게 갈지는 하위 층이 결정)
// ==========================================
let goalRobotId = null;
let missionSeq = 0;

function sendMission(type, robotId, goal) {
    if (!missionPub || !rosOnline) { toast('ROS 에 연결되지 않았습니다 (rosbridge 확인)', 'err'); return null; }
    const id = `m${Date.now().toString(36)}${missionSeq++}`;
    missionPub.publish(new ROSLIB.Message({
        mission_id: id, type, robot_id: robotId,
        goal: goal || { header: { frame_id: 'map' }, pose: { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } },
    }));
    return id;
}

window.openGoalModal = function (id) {
    goalRobotId = id;
    document.getElementById('goal-robot-name').textContent = displayName(id);
    openModal('goal-modal', 'goal-modal-content');
};
window.closeGoalModal = function () { closeModal('goal-modal', 'goal-modal-content'); };

const missionGoals = {};   // mission_id -> {x, y, yaw}  (지도에 목표를 표시하기 위해 보관)

// 주행 요청의 단일 진입점. robotId 가 빈 문자열이면 조정 층이 로봇을 고른다.
function launchNav(robotId, x, y, yaw) {
    const mid = sendMission('NAV_GOTO', robotId, {
        header: { frame_id: 'map' },
        pose: { position: { x, y, z: 0 }, orientation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) } },
    });
    if (mid) {
        missionGoals[mid] = { x, y, yaw };
        toast(`${robotId ? displayName(robotId) : '자동 배정'} 주행 요청 (${x.toFixed(2)}, ${y.toFixed(2)})`, 'info');
    }
    return mid;
}

window.submitGoal = function () {
    const x = parseFloat(document.getElementById('goal-x').value);
    const y = parseFloat(document.getElementById('goal-y').value);
    const yaw = parseFloat(document.getElementById('goal-yaw').value || '0') * Math.PI / 180;
    if (Number.isNaN(x) || Number.isNaN(y)) { toast('x, y 를 숫자로 입력하세요', 'err'); return; }
    stageGoal(goalRobotId, x, y, yaw);
    closeGoalModal();
    setMapTool(null);
};

// ---- 목표 대기(staged) : 목표를 미리 찍어 두고 개별 또는 한꺼번에 출발 ----
function canStart(r) { return effectiveState(r) === 'IDLE' && r.status && r.status.localized; }

function refreshStartAll() {
    const staged = Object.values(robots).filter(r => r.staged);
    const ready = staged.filter(canStart);
    const btn = document.getElementById('btn-start-all');
    document.getElementById('start-all-count').textContent = `(${ready.length}${staged.length !== ready.length ? '/' + staged.length : ''})`;
    btn.disabled = ready.length === 0 || !rosOnline;
    document.getElementById('btn-clear-staged').disabled = staged.length === 0;
    requestMapDraw();
}

// 목표 지점에서 가장 가까운 벽·미확인 칸까지의 거리(m). 지도가 없으면 Infinity
function wallClearance(x, y, maxR = 0.3) {
    const m = mapState.meta;
    if (!m || !mapState.occ) return Infinity;
    const res = m.resolution, c0 = Math.floor((x - m.origin[0]) / res), r0 = mapState.h - 1 - Math.floor((y - m.origin[1]) / res);
    const R = Math.ceil(maxR / res);
    let best = Infinity;
    for (let dr = -R; dr <= R; dr++) for (let dc = -R; dc <= R; dc++) {
        const r = r0 + dr, c = c0 + dc;
        if (r < 0 || r >= mapState.h || c < 0 || c >= mapState.w || mapState.occ[r * mapState.w + c] !== 0) {
            best = Math.min(best, Math.hypot(dr, dc) * res);
        }
    }
    return best;
}
const GOAL_MIN_CLEAR = 0.12;   // 목표가 벽에서 이보다 가까우면 Nav2 가 거기까지 못 간다 (로봇 반폭 0.06 + 여유)

window.stageGoal = function (id, x, y, yaw) {
    const r = robots[id];
    if (!r) return;
    r.staged = { x, y, yaw };
    const cl = wallClearance(x, y);
    if (cl < GOAL_MIN_CLEAR) {
        toast(`${displayName(id)} 목표가 벽·장애물에서 ${(cl * 100).toFixed(0)} cm 입니다. ${(GOAL_MIN_CLEAR * 100).toFixed(0)} cm 이상 떨어진 곳을 권장합니다 (도착 실패 가능)`, 'err');
    } else
    toast(`${displayName(id)} 목표 지정(대기): ${x.toFixed(2)}, ${y.toFixed(2)}`, 'info');
    updateCard(r); refreshStartAll();
};
window.clearStaged = function (id) {
    if (!robots[id]) return;
    robots[id].staged = null;
    updateCard(robots[id]); refreshStartAll();
};
window.clearAllStaged = function () {
    Object.values(robots).forEach(r => { r.staged = null; updateCard(r); });
    refreshStartAll();
};

// 카드의 주행 버튼: 지정해 둔 목표로 이 로봇만 출발
window.goOrPrompt = function (id) {
    const r = robots[id];
    if (!r) return;
    if (!r.staged) { toast(`${displayName(id)}: 먼저 '목표지정'으로 목표를 찍으세요`, 'err'); return; }
    if (!canStart(r)) { toast(`${displayName(id)}: 대기(IDLE)이고 위치가 확인된 상태에서만 출발합니다`, 'err'); return; }
    const g = r.staged;
    if (launchNav(id, g.x, g.y, g.yaw)) { r.staged = null; updateCard(r); refreshStartAll(); }
};

// 전체 주행 시작: 대기 중인 목표를 가진 모든 로봇을 같은 순간에 출발시킨다
window.startAllStaged = function () {
    const staged = Object.values(robots).filter(r => r.staged);
    const ready = staged.filter(canStart), skipped = staged.filter(r => !canStart(r));
    if (!ready.length) { toast('출발할 수 있는 대기 목표가 없습니다', 'err'); return; }
    const lines = ready.map(r => `  ${displayName(r.id)} → (${r.staged.x.toFixed(2)}, ${r.staged.y.toFixed(2)})`).join('\n');
    const skip = skipped.length ? `\n\n제외(IDLE·위치확인 아님): ${skipped.map(r => displayName(r.id)).join(', ')}` : '';
    if (!confirm(`${ready.length}대를 동시에 출발시킬까요?\n\n${lines}${skip}`)) return;
    ready.forEach(r => {
        const g = r.staged;
        if (launchNav(r.id, g.x, g.y, g.yaw)) r.staged = null;
    });
    Object.values(robots).forEach(updateCard);
    refreshStartAll();
};

window.cancelMission = function (id) {
    if (sendMission('CANCEL', id)) toast(`${displayName(id)} 취소 요청`, 'info');
};

// ==========================================
// 7. 로그 모달 (ROS 로그 + 백엔드의 launch 출력)
// ==========================================
let logRobotId = null, logTab = 'rosout', launchText = '';
const LEVEL_NAME = { 10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL' };

window.openLogModal = function (id) {
    logRobotId = id;
    robots[id].unreadErrors = 0;       // 확인한 것으로 처리
    updateCard(robots[id]); updateKpis();
    document.getElementById('log-robot-name').textContent = displayName(id);
    setLogTab('rosout');
    openModal('log-modal', 'log-modal-content');
};
window.closeLogModal = function () { closeModal('log-modal', 'log-modal-content', () => { logRobotId = null; }); };

window.setLogTab = async function (tab) {
    logTab = tab;
    document.getElementById('tab-rosout').className = `px-3 py-1 rounded ${tab === 'rosout' ? 'bg-slate-700 text-white' : 'bg-slate-800 text-slate-400'}`;
    document.getElementById('tab-launch').className = `px-3 py-1 rounded ${tab === 'launch' ? 'bg-slate-700 text-white' : 'bg-slate-800 text-slate-400'}`;
    if (tab === 'launch') {
        try { launchText = (await api('GET', `/robots/${logRobotId}/logs?lines=300`)).log || '(출력 없음 - 아직 ON 한 적이 없거나 로그가 비어 있음)'; }
        catch (e) { launchText = `백엔드에서 로그를 가져오지 못했습니다: ${e.message}`; }
    }
    renderLog();
};

window.renderLog = function () {
    const body = document.getElementById('log-body');
    const r = robots[logRobotId];
    if (!r) return;
    if (logTab === 'launch') { body.textContent = launchText; body.scrollTop = body.scrollHeight; return; }
    const min = parseInt(document.getElementById('log-level').value, 10);
    const lines = r.logs.filter(l => l.level >= min).map(l => {
        const t = new Date(l.t).toLocaleTimeString('ko-KR', { hour12: false });
        return `${t} [${(LEVEL_NAME[l.level] || l.level).toString().padEnd(5)}] ${l.name}: ${l.msg}`;
    });
    body.textContent = lines.length ? lines.join('\n') : '(표시할 ROS 로그가 없습니다. 로봇이 /rosout 으로 보내는 로그만 보입니다)';
    body.scrollTop = body.scrollHeight;
};
window.clearRosLog = function () { if (robots[logRobotId]) { robots[logRobotId].logs = []; renderLog(); } };

// ==========================================
// 8. 카메라 모달 (미구현 - 자리만 유지)
// ==========================================
window.openCamModal = function (name) { document.getElementById('cam-robot-name').innerText = name; openModal('cam-modal', 'cam-modal-content'); };
window.closeCamModal = function () { closeModal('cam-modal', 'cam-modal-content'); };

// ==========================================
// 9. 통신 진단 모달: 실제 heartbeat(마지막 수신 후 경과 시간)
// ==========================================
let networkChart = null, chartTimer = null, diagRobotId = null;

window.openDiagModal = function (id) {
    diagRobotId = id;
    document.getElementById('diag-robot-name').innerText = displayName(id);
    document.getElementById('diag-dds').textContent = netInfo.discovery === 'unicast' ? `unicast (${netInfo.rmw || 'cyclonedds'})` : 'multicast (자동 탐색)';
    openModal('diag-modal', 'diag-modal-content');
    initChart();
};
window.closeDiagModal = function () {
    closeModal('diag-modal', 'diag-modal-content', () => {
        if (networkChart) { networkChart.destroy(); networkChart = null; }
        clearInterval(chartTimer);
    });
};

function initChart() {
    const ctx = document.getElementById('networkChart').getContext('2d');
    const N = 60;
    networkChart = new Chart(ctx, {
        type: 'line',
        data: {
            labels: Array.from({ length: N }, (_, i) => `${i - N + 1}s`),
            datasets: [
                { label: 'Ping RTT (ms)', data: Array(N).fill(null), borderColor: '#4fd1c5', backgroundColor: 'rgba(79,209,197,0.1)', borderWidth: 2, tension: 0.3, fill: true, yAxisID: 'y', spanGaps: false },
                { label: 'ROS heartbeat age (s)', data: Array(N).fill(null), borderColor: '#f59e0b', borderWidth: 2, borderDash: [5, 5], tension: 0, yAxisID: 'y1', spanGaps: false },
            ],
        },
        options: {
            responsive: true, maintainAspectRatio: false, animation: { duration: 0 },
            scales: {
                x: { grid: { color: '#334155' }, ticks: { color: '#94a3b8', maxTicksLimit: 8 } },
                y: { type: 'linear', position: 'left', title: { display: true, text: 'RTT (ms)', color: '#4fd1c5' }, grid: { color: '#334155' }, ticks: { color: '#94a3b8' }, suggestedMin: 0, suggestedMax: 20 },
                y1: { type: 'linear', position: 'right', title: { display: true, text: 'age (s)', color: '#f59e0b' }, grid: { drawOnChartArea: false }, ticks: { color: '#94a3b8' }, suggestedMin: 0, suggestedMax: 5 },
            },
            plugins: { legend: { labels: { color: '#cbd5e1' } } },
        },
    });
    clearInterval(chartTimer);
    chartTimer = setInterval(updateDiag, 1000);
    updateDiag();
}

// ping 은 백엔드, heartbeat 는 ROS 상태에서 얻는다. 둘을 같이 보면 원인을 구분할 수 있다.
function diagVerdict(r, ping, hbOk) {
    if (!r.proc) return ['neutral', '로봇이 OFF 입니다. ON 한 뒤부터 ping 을 추적합니다.'];
    if (!ping || !ping.monitoring) return ['neutral', 'ping 모니터가 아직 시작되지 않았습니다.'];
    const pingOk = ping.last_rtt !== null && ping.last_rtt !== undefined;
    if (pingOk && hbOk) return ['ok', '통신 양호: 네트워크와 ROS 모두 정상'];
    if (pingOk && !hbOk) {
        const hint = netInfo.discovery === 'unicast'
            ? 'DDS 는 유니캐스트로 설정돼 있으니, 관제PC 프로세스에 fms_env.sh 가 적용됐는지·방화벽(UDP)·로봇 launch 상태·ROS_DOMAIN_ID 를 확인하세요'
            : 'Wi-Fi 에서 멀티캐스트 탐색이 막혔을 수 있습니다. robots.yaml 의 network.discovery 를 unicast 로 바꿔 보세요 (그 외: launch 상태, ROS_DOMAIN_ID)';
        return ['warn', '네트워크는 되는데 ROS 데이터가 안 옴: ' + hint];
    }
    return ['err', '로봇 IP 로 ping 이 안 됨: Wi-Fi/공유기 연결, 로봇 전원, IP 변경 여부를 확인하세요'];
}

async function updateDiag() {
    const r = robots[diagRobotId];
    if (!r || !networkChart) return;
    let ping = null;
    try { ping = await api('GET', `/robots/${r.id}/ping`); } catch (e) { /* 백엔드 오프라인 */ }
    if (!networkChart) return;

    const online = r.status && effectiveState(r) !== 'OFFLINE';
    const age = online ? r.status.last_seen_sec_ago : null;
    document.getElementById('diag-state').textContent = effectiveState(r);
    document.getElementById('diag-avg-rtt').textContent = age === null ? '-- s' : age.toFixed(2) + ' s';

    const fmt = (v) => (v === null || v === undefined) ? '--' : v.toFixed(1);
    const ok = ping && ping.monitoring;
    document.getElementById('diag-ping').textContent = ok ? `${fmt(ping.last_rtt)} / ${fmt(ping.avg_rtt)} ms` : '-- ms';
    const lossEl = document.getElementById('diag-loss');
    lossEl.textContent = ok && ping.loss_pct !== null ? ping.loss_pct.toFixed(0) + ' %' : '--';
    lossEl.className = `text-xl font-bold mt-1 ${ok && ping.loss_pct > 20 ? 'text-rose-400' : ok && ping.loss_pct > 0 ? 'text-amber-400' : 'text-emerald-400'}`;

    const [kind, text] = diagVerdict(r, ping, online);
    const v = document.getElementById('diag-verdict');
    const cls = { ok: 'bg-emerald-500/10 border-emerald-700 text-emerald-300', warn: 'bg-amber-500/10 border-amber-700 text-amber-300', err: 'bg-rose-500/10 border-rose-700 text-rose-300', neutral: 'bg-slate-800 border-slate-700 text-slate-400' }[kind];
    v.className = `mb-4 text-xs rounded-lg px-3 py-2 border ${cls}`;
    v.textContent = '진단: ' + text;

    // RTT 는 백엔드가 보관한 최근 60초를 그대로 그리고, heartbeat 는 이 창에서 누적한다
    const rtt = networkChart.data.datasets[0].data;
    const N = rtt.length;
    const samples = ok ? ping.samples.slice(-N) : [];
    for (let i = 0; i < N; i++) rtt[i] = i >= N - samples.length ? samples[i - (N - samples.length)] : null;
    const hb = networkChart.data.datasets[1].data;
    hb.push(age); hb.shift();
    networkChart.update();
}

// ==========================================
// 10. 글로벌 커맨드
// ==========================================
document.getElementById('btn-estop')?.addEventListener('click', () => {
    if (!globalCmdPub || !rosOnline) { toast('ROS 에 연결되지 않았습니다', 'err'); return; }
    globalCmdPub.publish(new ROSLIB.Message({ data: 'E_STOP' }));
    toast('GLOBAL E-STOP: 모든 로봇의 진행 중인 주행을 취소했습니다', 'err');
});
document.getElementById('btn-return')?.addEventListener('click', () => {
    toast('RETURN DOCK 은 아직 구현되지 않았습니다 (도크 위치/미션 정의 필요)', 'info');
});

// ==========================================
// 12. 맵 뷰: 맵 표시 + 로봇 위치 + 클릭(드래그: 방향)으로 목표 지정
// ==========================================
const mapCanvas = document.getElementById('map-canvas');
const mctx = mapCanvas.getContext('2d');
const ROBOT_COLORS = ['#38bdf8', '#f59e0b', '#a78bfa', '#34d399', '#f472b6'];
const mapState = { meta: null, bmp: null, w: 0, h: 0, drag: null, hover: null, candidate: null };
let mapDrawQueued = false;
let mapTool = null;   // { type: 'initpose' | 'goal' | 'auto', robotId } — 다음 맵 입력(클릭+드래그)을 어떤 용도로 쓸지
let strayHintAt = 0;

const TOOL_TEXT = {
    initpose: ['초기 위치:', '로봇이 실제로 있는 곳을 클릭하고 보는 방향으로 드래그'],
    goal: ['목표 지점:', '가고 싶은 곳을 클릭하고 도착 방향으로 드래그 (지정만 하고 출발은 하지 않음)'],
    auto: ['자동 배정:', '목적지를 클릭하고 방향으로 드래그 → 가장 가까운 대기 로봇이 즉시 출발'],
};

function setMapTool(tool) {
    mapTool = tool;
    const c = document.getElementById('map-tool-hint');
    c.classList.toggle('hidden', !tool);
    document.getElementById('map-guide').classList.toggle('hidden', !!tool);
    if (tool) {
        document.getElementById('tool-name').textContent = (tool.robotId ? displayName(tool.robotId) + ' ' : '') + TOOL_TEXT[tool.type][0];
        document.getElementById('tool-desc').textContent = TOOL_TEXT[tool.type][1];
    }
    document.getElementById('tool-coord').classList.toggle('hidden', !(tool && (tool.type === 'goal' || tool.type === 'initpose')));
    document.getElementById('tool-coord').textContent = tool && tool.type === 'initpose' ? '벽 거리로 입력' : '좌표 입력';
    if (!tool) mapState.wallCands = null;
    mapCanvas.style.cursor = tool ? 'crosshair' : 'default';
    Object.values(robots).forEach(updateCard);     // 켜져 있는 도구의 로봇 버튼을 강조
}
function armTool(type, id) {
    if (!mapState.meta) { toast('먼저 맵을 업로드/선택하세요', 'err'); return; }
    if (mapTool && mapTool.type === type && mapTool.robotId === id) { setMapTool(null); return; }   // 같은 버튼을 다시 누르면 해제
    if (type === 'initpose' && rosOnline && robots[id]) initPoseTopic(id);     // 맵을 클릭하기 전에 미리 연결
    setMapTool({ type, robotId: id });
}
window.startInitialPose = (id) => armTool('initpose', id);
window.startGoalTool = (id) => armTool('goal', id);
window.startAutoTool = () => armTool('auto', '');
window.cancelMapTool = () => setMapTool(null);
window.openCoordInput = function () {
    if (mapTool && mapTool.type === 'goal') openGoalModal(mapTool.robotId);
    else if (mapTool && mapTool.type === 'initpose') openWallModal(mapTool.robotId);
};
document.addEventListener('keydown', (e) => { if (e.key === 'Escape') { setMapTool(null); closeGoalPicker(); } });

// ---- 로봇 선택 팝업: 맵에서 목표를 먼저 찍고, 어느 로봇의 목표로 할지 고른다 ----
function closeGoalPicker() {
    document.getElementById('goal-picker').classList.add('hidden');
    if (mapState.candidate) { mapState.candidate = null; requestMapDraw(); }
}
window.closeGoalPicker = closeGoalPicker;

function openGoalPicker(cx, cy, g) {
    const ids = Object.keys(robots);
    if (!ids.length) { toast('등록된 로봇이 없습니다', 'err'); return; }
    mapState.candidate = { x: g.x, y: g.y, yaw: g.yaw };
    requestMapDraw();
    const stateStyle = { IDLE: 'text-sky-400', BUSY: 'text-emerald-400', STARTING: 'text-amber-400', OFFLINE: 'text-slate-500' };
    document.getElementById('goal-picker-coord').textContent = `x ${g.x.toFixed(2)}, y ${g.y.toFixed(2)}, 방향 ${(g.yaw * 180 / Math.PI).toFixed(0)}°`;
    const list = document.getElementById('goal-picker-list');
    list.innerHTML = '';
    ids.forEach((id, idx) => {
        const r = robots[id], st = effectiveState(r), online = st === 'IDLE' || st === 'BUSY';
        const loc = online && r.status && r.status.localized;
        const color = ROBOT_COLORS[idx % ROBOT_COLORS.length];
        const row = document.createElement('div');
        row.className = 'flex items-center gap-2 bg-slate-800 border border-slate-700 rounded-lg px-2 py-1.5';
        const note = !online ? '꺼짐/연결 안 됨' : !loc ? '위치 미확인' : (r.staged ? '이미 대기 목표 있음(덮어씀)' : st === 'BUSY' ? '주행 중' : '준비됨');
        row.innerHTML = `<span class="w-2.5 h-2.5 rounded-full flex-shrink-0" style="background:${color}"></span>
            <div class="flex-1 min-w-0"><p class="font-bold text-white">${esc(displayName(id))} <span class="font-normal ${stateStyle[st] || ''}">${esc(st)}</span></p><p class="text-[10px] text-slate-400">${esc(note)}</p></div>`;
        const btn = (label, enabled, cls, fn, title) => {
            const b = document.createElement('button');
            b.textContent = label; b.disabled = !enabled; b.title = title || '';
            b.className = `px-2 py-1 rounded font-bold ${cls} ${enabled ? '' : 'opacity-40 cursor-not-allowed'}`;
            b.addEventListener('click', fn);
            return b;
        };
        row.appendChild(btn('목표 지정', online, 'bg-slate-600 hover:bg-slate-500 text-white', () => {
            stageGoal(id, g.x, g.y, g.yaw); closeGoalPicker();
        }, '이 로봇의 목표로 저장만 합니다 (출발은 카드의 주행)'));
        row.appendChild(btn('지정 후 주행', st === 'IDLE' && loc, 'bg-cyan-700 hover:bg-cyan-600 text-white', () => {
            stageGoal(id, g.x, g.y, g.yaw); closeGoalPicker(); goOrPrompt(id);
        }, st === 'IDLE' && loc ? '이 로봇의 목표로 지정하고 바로 출발' : '대기(IDLE)이고 위치가 확인된 로봇만 바로 출발할 수 있습니다'));
        list.appendChild(row);
    });
    // 클릭한 자리 옆에 띄우되 화면 밖으로 나가지 않게 한다
    const pk = document.getElementById('goal-picker');
    pk.classList.remove('hidden');
    const w = pk.offsetWidth, h = pk.offsetHeight;
    pk.style.left = Math.max(8, Math.min(cx + 14, window.innerWidth - w - 8)) + 'px';
    pk.style.top = Math.max(8, Math.min(cy + 14, window.innerHeight - h - 8)) + 'px';
}
// 팝업 바깥(맵 제외)을 누르면 닫는다
document.addEventListener('pointerdown', (e) => {
    const pk = document.getElementById('goal-picker');
    if (pk.classList.contains('hidden') || pk.contains(e.target) || e.target === mapCanvas) return;
    closeGoalPicker();
}, true);

// 로봇별 초기위치 발행자를 한 번만 만들어 재사용한다.
// 발행할 때마다 새로 만들면 DDS 에서 로봇의 구독자와 연결되기 전에 메시지가 먼저 나가 사라질 수 있다.
const initPoseTopics = {};
function initPoseTopic(robotId) {
    const key = `${robots[robotId].ns}|${rosGeneration}`;
    if (!initPoseTopics[robotId] || initPoseTopics[robotId].key !== key) {
        const t = new ROSLIB.Topic({ ros, name: `/${robots[robotId].ns}/initialpose`, messageType: 'geometry_msgs/msg/PoseWithCovarianceStamped' });
        t.advertise();
        initPoseTopics[robotId] = { key, topic: t };
    }
    return initPoseTopics[robotId].topic;
}

const INIT_COV_XY = 0.02, INIT_COV_YAW = 0.03;
const WHEEL_HALF_SEP = 0.048;     // pinky 휠 간격 0.0961 m 의 절반: 왼/오른쪽 벽은 휠 센터까지 잰 거리에 이만큼 더해 로봇 중심까지의 거리로 쓴다

function publishInitialPose(robotId, x, y, yaw) {
    if (!rosOnline) { toast('ROS 에 연결되지 않았습니다', 'err'); return; }
    const cov = Array(36).fill(0); cov[0] = INIT_COV_XY; cov[7] = INIT_COV_XY; cov[35] = INIT_COV_YAW;   // 표준편차 약 0.14 m / 10°. RViz 기본(0.25 → ±0.5 m)은 이 맵에서는 너무 넓다
    const msg = () => new ROSLIB.Message({
        header: { frame_id: 'map' },
        pose: { pose: { position: { x, y, z: 0 }, orientation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) } }, covariance: cov },
    });
    const t = initPoseTopic(robotId);
    t.publish(msg());
    setTimeout(() => { if (rosOnline) t.publish(msg()); }, 1000);     // 같은 위치를 한 번 더 (AMCL 은 같은 초기위치를 다시 받아도 문제없다)
    toast(`${displayName(robotId)} 초기 위치 지정 (${x.toFixed(2)}, ${y.toFixed(2)})`, 'info');
}

// ---- 초기 위치: 벽까지의 줄자 거리 → 지도 좌표 ----
let wallRobotId = null;
window.openWallModal = function (id) {
    wallRobotId = id;
    document.getElementById('wall-robot-name').textContent = displayName(id);
    document.getElementById('wall-result').innerHTML = '';
    openModal('wall-modal', 'wall-modal-content');
};
window.closeWallModal = function () { closeModal('wall-modal', 'wall-modal-content'); };
window.wallHeadingChanged = function () {
    document.getElementById('wall-heading-custom').classList.toggle('hidden', document.getElementById('wall-heading').value !== 'custom');
};

// (x, y) 에서 각도 ang(rad) 방향으로 첫 장애물 셀까지의 거리. 범위 안에 없으면 Infinity
function rayToWall(x, y, ang, maxR = 2.0) {
    const m = mapState.meta, res = m.resolution, dx = Math.cos(ang), dy = Math.sin(ang);
    for (let d = 0; d <= maxR; d += 0.005) {
        const col = Math.floor((x + dx * d - m.origin[0]) / res), row = mapState.h - 1 - Math.floor((y + dy * d - m.origin[1]) / res);
        if (col < 0 || col >= mapState.w || row < 0 || row >= mapState.h) return Infinity;
        if (mapState.occ[row * mapState.w + col] === 1) return d;
    }
    return Infinity;
}

// 로봇 중심 기준으로 입력값을 맞춘다: 좌우 벽은 휠 센터 거리에 휠 간격의 절반을 더한다
function wallSpec(side, cm, yaw) {
    const off = { front: 0, rear: Math.PI, left: Math.PI / 2, right: -Math.PI / 2 }[side];
    return { ang: yaw + off, dist: cm / 100 + ((side === 'left' || side === 'right') ? WHEEL_HALF_SEP : 0) };
}

window.solveWallPose = function () {
    const box = document.getElementById('wall-result');
    if (!mapState.meta || !mapState.occ) { box.innerHTML = '<p class="text-red-400">맵이 선택되지 않았습니다</p>'; return; }
    const hv = document.getElementById('wall-heading').value;
    const yaw = (hv === 'custom' ? parseFloat(document.getElementById('wall-heading-custom').value) : parseFloat(hv)) * Math.PI / 180;
    const s1 = document.getElementById('wall-side1').value, s2 = document.getElementById('wall-side2').value;
    const c1 = parseFloat(document.getElementById('wall-d1').value), c2 = parseFloat(document.getElementById('wall-d2').value);
    if (!isFinite(yaw) || !isFinite(c1) || !isFinite(c2) || c1 < 0 || c2 < 0) { box.innerHTML = '<p class="text-red-400">방향과 거리를 확인하세요</p>'; return; }
    const o = (s1 === s2 || (['front', 'rear'].includes(s1) === ['front', 'rear'].includes(s2)));
    if (o) { box.innerHTML = '<p class="text-red-400">두 벽은 서로 직각이어야 합니다 (앞/뒤 중 하나 + 왼/오른쪽 중 하나)</p>'; return; }
    const a = wallSpec(s1, c1, yaw), b = wallSpec(s2, c2, yaw);

    const m = mapState.meta, res = m.resolution, W = mapState.w * res, H = mapState.h * res, R = 0.06;   // R: 로봇이 벽에 최소 이만큼은 떨어져 있어야 한다 (내접 반경)
    const cost = (x, y) => {
        const r1 = rayToWall(x, y, a.ang, a.dist + 0.3), r2 = rayToWall(x, y, b.ang, b.dist + 0.3);
        return (r1 === Infinity || r2 === Infinity) ? Infinity : (r1 - a.dist) ** 2 + (r2 - b.dist) ** 2;
    };
    // 1) 거친 격자(2 cm)로 후보를 찾고  2) 후보 주변을 5 mm 격자로 다듬는다
    let seeds = [];
    for (let x = m.origin[0]; x < m.origin[0] + W; x += 0.02) for (let y = m.origin[1]; y < m.origin[1] + H; y += 0.02) {
        const col = Math.floor((x - m.origin[0]) / res), row = mapState.h - 1 - Math.floor((y - m.origin[1]) / res);
        if (mapState.occ[row * mapState.w + col] !== 0) continue;      // 이동 가능한 칸만 (장애물·미확인 제외)
        if (rayToWall(x, y, 0, R) < R || rayToWall(x, y, Math.PI / 2, R) < R || rayToWall(x, y, Math.PI, R) < R || rayToWall(x, y, -Math.PI / 2, R) < R) continue;
        const c = cost(x, y);
        if (c < 0.03 ** 2 * 2) seeds.push({ x, y, c });
    }
    seeds.sort((p, q) => p.c - q.c);
    const cands = [];
    for (const sd of seeds) {
        if (cands.some(k => Math.hypot(k.x - sd.x, k.y - sd.y) < 0.12)) continue;      // 12 cm 안의 후보는 하나로 본다
        let best = sd;
        for (let x = sd.x - 0.03; x <= sd.x + 0.03; x += 0.005) for (let y = sd.y - 0.03; y <= sd.y + 0.03; y += 0.005) {
            const c = cost(x, y);
            if (c < best.c) best = { x, y, c };
        }
        cands.push(best);
        if (cands.length >= 4) break;
    }
    mapState.wallCands = cands.map(k => ({ x: k.x, y: k.y, yaw }));
    requestMapDraw();
    if (!cands.length) { box.innerHTML = '<p class="text-amber-300">이 맵에서 입력한 거리와 맞는 위치를 찾지 못했습니다. 방향·벽 종류·거리를 확인하세요 (벽이 지도에 없거나 먼 경우도 해당).</p>'; return; }
    box.innerHTML = (cands.length > 1 ? '<p class="text-amber-300">맞는 위치가 여러 곳입니다. 지도의 점선 표시를 보고 실제 위치를 고르세요.</p>' : '<p class="text-slate-400">지도의 점선 표시가 계산된 위치입니다.</p>') +
        cands.map((k, i) => `<div class="flex items-center justify-between bg-slate-800 rounded px-3 py-1.5"><span class="font-mono text-slate-300">#${i + 1} x ${k.x.toFixed(2)}, y ${k.y.toFixed(2)} · 맞춤 오차 ${(Math.sqrt(k.c / 2) * 100).toFixed(1)} cm</span><button onclick="applyWallPose(${i})" class="px-2 py-1 rounded bg-amber-600 hover:bg-amber-500 text-white font-bold">이 위치로 지정</button></div>`).join('');
};

window.applyWallPose = function (i) {
    const k = mapState.wallCands && mapState.wallCands[i];
    if (!k || !wallRobotId) return;
    publishInitialPose(wallRobotId, k.x, k.y, k.yaw);
    mapState.wallCands = null;
    setMapTool(null);
    closeWallModal();
    requestMapDraw();
};

function parsePgm(buf) {
    const u = new Uint8Array(buf);
    let i = 0;
    const tok = () => {
        while (i < u.length) {
            if (u[i] === 35) { while (i < u.length && u[i] !== 10) i++; }      // '#' 주석
            else if (u[i] <= 32) i++;
            else break;
        }
        let t = '';
        while (i < u.length && u[i] > 32) t += String.fromCharCode(u[i++]);
        return t;
    };
    if (tok() !== 'P5') throw new Error('P5(PGM) 형식이 아님');
    const w = parseInt(tok(), 10), h = parseInt(tok(), 10), maxv = parseInt(tok(), 10);
    i++;   // 헤더 뒤 공백 1개
    if (maxv > 255) throw new Error('16bit PGM 은 지원하지 않음');
    return { w, h, data: u.subarray(i, i + w * h) };
}

// map_server 의 trinary 규칙: 점유 확률 = negate ? p/255 : (255-p)/255
function buildMapBitmap(w, h, gray, meta) {
    const off = document.createElement('canvas');
    off.width = w; off.height = h;
    const c = off.getContext('2d');
    const img = c.createImageData(w, h);
    for (let k = 0; k < w * h; k++) {
        const p = gray[k];
        const occ = meta.negate ? p / 255 : (255 - p) / 255;
        let rgb;
        if (occ > meta.occupied_thresh) rgb = [15, 23, 42];            // 장애물
        else if (occ < meta.free_thresh) rgb = [226, 232, 240];        // 이동 가능
        else rgb = [100, 116, 139];                                    // 미확인
        img.data[k * 4] = rgb[0]; img.data[k * 4 + 1] = rgb[1]; img.data[k * 4 + 2] = rgb[2]; img.data[k * 4 + 3] = 255;
    }
    c.putImageData(img, 0, 0);
    return off;
}

async function selectMap(name) {
    if (!name) return;
    try {
        const meta = await api('GET', `/maps/${name}`);
        const res = await fetch(`${BACKEND_URL}/maps/${name}/image`);
        if (!res.ok) throw new Error(`이미지 HTTP ${res.status}`);
        let w, h, gray;
        if (meta.image.toLowerCase().endsWith('.pgm')) {
            ({ w, h, data: gray } = parsePgm(await res.arrayBuffer()));
        } else {
            const bmp = await createImageBitmap(await res.blob());
            w = bmp.width; h = bmp.height;
            const t = document.createElement('canvas'); t.width = w; t.height = h;
            const tc = t.getContext('2d'); tc.drawImage(bmp, 0, 0);
            const px = tc.getImageData(0, 0, w, h).data;
            gray = new Uint8Array(w * h);
            for (let k = 0; k < w * h; k++) gray[k] = px[k * 4];
        }
        mapState.meta = meta; mapState.w = w; mapState.h = h;
        mapState.occ = new Uint8Array(w * h);        // 벽 거리 계산용
        for (let k = 0; k < w * h; k++) {
            const p = meta.negate ? gray[k] / 255 : (255 - gray[k]) / 255;
            mapState.occ[k] = p > meta.occupied_thresh ? 1 : (p < meta.free_thresh ? 0 : 2);     // 0=이동 가능, 1=장애물, 2=미확인
        }
        mapState.bmp = buildMapBitmap(w, h, gray, meta);
        document.getElementById('map-empty').classList.add('hidden');
        document.getElementById('map-select').value = name;
        requestMapDraw();
    } catch (e) { toast(`맵을 불러오지 못했습니다: ${e.message}`, 'err'); }
}

async function loadMaps() {
    try {
        const list = await api('GET', '/maps');
        const sel = document.getElementById('map-select');
        sel.innerHTML = list.map(m => `<option value="${esc(m.name)}">${esc(m.name)}</option>`).join('');
        if (list.length && !mapState.meta) await selectMap(list[0].name);
        return true;
    } catch (e) { return false; }
}

window.selectMap = selectMap;
window.uploadMap = async function (input) {
    const files = [...input.files];
    input.value = '';
    const y = files.find(f => /\.ya?ml$/i.test(f.name)), im = files.find(f => /\.(pgm|png)$/i.test(f.name));
    if (!y || !im) { toast('map.yaml 과 이미지(pgm/png)를 함께 선택하세요', 'err'); return; }
    const fd = new FormData();
    fd.append('yaml_file', y); fd.append('image_file', im);
    try {
        const res = await fetch(`${BACKEND_URL}/maps`, { method: 'POST', body: fd });
        const data = await res.json().catch(() => ({}));
        if (!res.ok) throw new Error(data.detail || `HTTP ${res.status}`);
        toast(`맵 업로드 완료: ${data.name}`, 'ok');
        await loadMaps();
        await selectMap(data.name);
    } catch (e) { toast(`업로드 실패: ${e.message}`, 'err'); }
};

// 좌표 변환: 맵 이미지 픽셀 <-> 월드(m).  이미지 row 0 이 맨 위, 월드 y 는 위쪽이 +
function mapView() {
    const cw = mapCanvas.clientWidth, ch = mapCanvas.clientHeight;
    const scale = Math.min(cw / mapState.w, ch / mapState.h) * 0.95;
    return { scale, ox: (cw - mapState.w * scale) / 2, oy: (ch - mapState.h * scale) / 2 };
}
function worldToScreen(x, y) {
    const m = mapState.meta, v = mapView();
    const col = (x - m.origin[0]) / m.resolution, row = mapState.h - (y - m.origin[1]) / m.resolution;
    return [v.ox + col * v.scale, v.oy + row * v.scale];
}
function screenToWorld(sx, sy) {
    const m = mapState.meta, v = mapView();
    const col = (sx - v.ox) / v.scale, row = (sy - v.oy) / v.scale;
    return { x: m.origin[0] + col * m.resolution, y: m.origin[1] + (mapState.h - row) * m.resolution, inside: col >= 0 && col < mapState.w && row >= 0 && row < mapState.h };
}

function drawArrow(x, y, yaw, len, color, width) {
    const ex = x + Math.cos(-yaw) * len, ey = y + Math.sin(-yaw) * len;   // 화면 y 는 아래가 + 이므로 부호 반전
    mctx.strokeStyle = color; mctx.lineWidth = width;
    mctx.beginPath(); mctx.moveTo(x, y); mctx.lineTo(ex, ey); mctx.stroke();
}

function requestMapDraw() {
    if (mapDrawQueued) return;
    mapDrawQueued = true;
    requestAnimationFrame(() => { mapDrawQueued = false; drawMap(); });
}

function drawMap() {
    const dpr = window.devicePixelRatio || 1;
    const cw = mapCanvas.clientWidth, ch = mapCanvas.clientHeight;
    if (mapCanvas.width !== Math.round(cw * dpr) || mapCanvas.height !== Math.round(ch * dpr)) {
        mapCanvas.width = Math.round(cw * dpr); mapCanvas.height = Math.round(ch * dpr);
    }
    mctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    mctx.clearRect(0, 0, cw, ch);
    if (!mapState.bmp) return;
    const v = mapView();
    mctx.imageSmoothingEnabled = false;
    mctx.drawImage(mapState.bmp, v.ox, v.oy, mapState.w * v.scale, mapState.h * v.scale);

    Object.values(robots).forEach((r, idx) => {
        const color = ROBOT_COLORS[idx % ROBOT_COLORS.length];
        if (r.staged) {       // 지정만 해 둔 목표 (점선 사각형 + 로봇 이름)
            const [sx, sy] = worldToScreen(r.staged.x, r.staged.y);
            mctx.strokeStyle = color; mctx.lineWidth = 2; mctx.setLineDash([3, 3]);
            mctx.strokeRect(sx - 8, sy - 8, 16, 16); mctx.setLineDash([]);
            drawArrow(sx, sy, r.staged.yaw, 16, color, 2);
            mctx.fillStyle = color; mctx.font = '11px Inter, sans-serif';
            mctx.fillText(displayName(r.id) + ' 대기', sx + 12, sy - 10);
        }
        if (r.goalPose) {     // 진행 중인 목표
            const [gx, gy] = worldToScreen(r.goalPose.x, r.goalPose.y);
            mctx.strokeStyle = color; mctx.lineWidth = 2; mctx.setLineDash([4, 3]);
            mctx.beginPath(); mctx.arc(gx, gy, 8, 0, Math.PI * 2); mctx.stroke(); mctx.setLineDash([]);
            drawArrow(gx, gy, r.goalPose.yaw, 14, color, 2);
        }
        const st = effectiveState(r);
        if ((st === 'IDLE' || st === 'BUSY') && r.status && r.status.localized) {
            const [rx, ry] = worldToScreen(r.status.x, r.status.y);
            mctx.fillStyle = color; mctx.strokeStyle = '#0f172a'; mctx.lineWidth = 2;
            mctx.beginPath(); mctx.arc(rx, ry, 7, 0, Math.PI * 2); mctx.fill(); mctx.stroke();
            drawArrow(rx, ry, r.status.yaw, 14, color, 3);
            mctx.fillStyle = '#e2e8f0'; mctx.font = '11px Inter, sans-serif';
            mctx.fillText(displayName(r.id), rx + 10, ry - 8);
        }
    });

    if (mapState.candidate) {     // 팝업에서 로봇을 고르기 전의 후보 목표
        const c = mapState.candidate, [px, py] = worldToScreen(c.x, c.y);
        mctx.strokeStyle = '#e2e8f0'; mctx.lineWidth = 2; mctx.setLineDash([5, 3]);
        mctx.beginPath(); mctx.arc(px, py, 9, 0, Math.PI * 2); mctx.stroke(); mctx.setLineDash([]);
        drawArrow(px, py, c.yaw, 20, '#e2e8f0', 2);
    }

    if (mapState.wallCands) {     // 벽 거리로 계산된 초기 위치 후보
        mapState.wallCands.forEach((k, i) => {
            const [px, py] = worldToScreen(k.x, k.y);
            mctx.strokeStyle = '#fbbf24'; mctx.lineWidth = 2; mctx.setLineDash([4, 3]);
            mctx.beginPath(); mctx.arc(px, py, 9, 0, Math.PI * 2); mctx.stroke(); mctx.setLineDash([]);
            drawArrow(px, py, k.yaw, 18, '#fbbf24', 2);
            mctx.fillStyle = '#fbbf24'; mctx.font = '11px Inter, sans-serif'; mctx.fillText('#' + (i + 1), px + 11, py - 9);
        });
    }

    if (mapState.drag) {      // 지금 지정 중인 목표
        const d = mapState.drag, [gx, gy] = worldToScreen(d.x, d.y);
        const pc = mapTool && mapTool.type === 'initpose' ? '#fbbf24' : '#22d3ee';
        mctx.strokeStyle = pc; mctx.lineWidth = 2;
        mctx.beginPath(); mctx.arc(gx, gy, 8, 0, Math.PI * 2); mctx.stroke();
        drawArrow(gx, gy, d.yaw, 22, pc, 3);
    }
}

function mapPointer(e) {
    const r = mapCanvas.getBoundingClientRect();
    return screenToWorld(e.clientX - r.left, e.clientY - r.top);
}

mapCanvas.addEventListener('pointerdown', (e) => {
    if (!mapState.meta) return;
    const w = mapPointer(e);
    if (!w.inside) return;
    closeGoalPicker();       // 열려 있던 팝업은 닫고 새로 찍는다
    mapCanvas.setPointerCapture(e.pointerId);
    mapState.drag = { x: w.x, y: w.y, yaw: 0, moved: false };
    requestMapDraw();
});
mapCanvas.addEventListener('pointermove', (e) => {
    if (!mapState.meta) return;
    const w = mapPointer(e);
    document.getElementById('map-cursor').textContent = `x ${w.x.toFixed(2)}, y ${w.y.toFixed(2)}`;
    const d = mapState.drag;
    if (d) {
        const dx = w.x - d.x, dy = w.y - d.y;
        if (Math.hypot(dx, dy) > 0.1) { d.yaw = Math.atan2(dy, dx); d.moved = true; }
        requestMapDraw();
    }
});
mapCanvas.addEventListener('pointerup', (e) => {
    const d = mapState.drag;
    mapState.drag = null;
    requestMapDraw();
    if (!d) return;
    if (!mapTool) { openGoalPicker(e.clientX, e.clientY, d); return; }     // 도구가 없으면 어느 로봇의 목표로 할지 고르는 팝업
    const tool = mapTool;
    setMapTool(null);                       // 한 번 쓰면 해제 (초기위치와 같은 방식)
    if (tool.type === 'initpose') publishInitialPose(tool.robotId, d.x, d.y, d.yaw);
    else if (tool.type === 'goal') stageGoal(tool.robotId, d.x, d.y, d.yaw);      // 대기 상태로만 저장, 출발은 '주행'
    else if (tool.type === 'auto') {
        if (confirm(`자동 배정(대기 중인 가장 가까운 로봇) → x ${d.x.toFixed(2)}, y ${d.y.toFixed(2)}, 방향 ${(d.yaw * 180 / Math.PI).toFixed(0)}° 로 주행할까요?`)) {
            launchNav('', d.x, d.y, d.yaw);
        }
    }
});
mapCanvas.addEventListener('pointercancel', () => { mapState.drag = null; requestMapDraw(); });
new ResizeObserver(requestMapDraw).observe(mapCanvas);

// ==========================================
// 11. 시작: 로봇 목록(백엔드: robots.yaml 고정 로봇 + GUI 에서 추가한 로봇) 로드 → 카드 렌더링 → ROS 연결
// ==========================================
// 백엔드 목록을 반영한다. 로봇이 추가·삭제됐을 때만 카드를 다시 그린다 (다른 창이나 API 에서 바뀐 것도 반영).
function applyRobotList(list) {
    const ids = new Set(list.map(x => x.id));
    let changed = list.length !== Object.keys(robots).length;
    Object.keys(robots).forEach(id => { if (!ids.has(id)) { delete robots[id]; changed = true; } });
    list.forEach(x => {
        let r = robots[x.id];
        if (!r) {
            r = robots[x.id] = { id: x.id, proc: false, status: null, statusAt: 0, task: null, goalPose: null, staged: null, logs: [], unreadErrors: 0 };
            changed = true;
        }
        Object.assign(r, { ns: x.namespace, ip: x.ip, user: x.user, mode: x.mode, preset: x.preset });
        if (x.launched) r.proc = true;
    });
    return changed;
}

async function loadRobots(force = true) {
    try {
        const changed = applyRobotList(await api('GET', '/robots'));
        if (force || changed || !robotsLoaded) { renderFleet(); }
        robotsLoaded = true;
        return true;
    } catch (e) { return false; }
}

async function init() {
    connectRos();
    try { netInfo = await api('GET', '/network'); } catch (e) { /* 백엔드가 늦게 뜨면 기본값 유지 */ }
    try { cfgInfo = await api('GET', '/config'); } catch (e) { /* 기본값 유지 */ }
    if (!await loadRobots()) toast('백엔드에 연결할 수 없습니다. 연결되면 자동으로 로봇 목록을 불러옵니다.', 'err');
    loadMaps();
    pollProcStatus();
    setInterval(pollProcStatus, 2000);
    setInterval(() => { if (!mapState.meta) loadMaps(); }, 5000);
    setInterval(() => { Object.values(robots).forEach(updateCard); updateKpis(); refreshStartAll(); }, 1000);   // 상태 메시지가 끊겨도 OFFLINE 으로 전환
}
init();
