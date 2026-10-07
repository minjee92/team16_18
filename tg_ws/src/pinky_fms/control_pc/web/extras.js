// ==========================================================
// Pinky FMS 웹 GUI 추가 기능 (app.js 다음에 로드. app.js 의 전역 함수·상태를 그대로 쓴다)
//  1. RETURN DOCK: 로봇을 마지막으로 지정한 Init Pose 위치(= 도크)로 돌려보낸다
//  2. REC: 주행 기록 — 백엔드가 관제PC 로컬(~/pinky/logs/rec_*)에 rosbag + 분석용 텍스트를 저장
// ==========================================================

// ---------- 1. RETURN DOCK ----------
// 도크 = 로봇별 마지막 Init Pose. 브라우저를 새로 고쳐도 남도록 지도별로 저장한다
const HOME_KEY = (id) => `fms.home.${mapName() || '-'}.${id}`;
const DOCK_IDLE_WAIT_MS = 12000;      // 주행 중이면 취소한 뒤 IDLE 이 될 때까지 기다리는 최대 시간

function saveHome(r, x, y, yaw) {
    r.home = { x, y, yaw };
    try { localStorage.setItem(HOME_KEY(r.id), JSON.stringify(r.home)); } catch (e) { /* 저장소 없음: 이번 화면에서만 기억 */ }
}

function getHome(r) {
    try {
        const h = JSON.parse(localStorage.getItem(HOME_KEY(r.id)) || 'null');
        if (h && Number.isFinite(h.x) && Number.isFinite(h.y)) return { yaw: 0, ...h };
    } catch (e) { /* 무시 */ }
    return r.home || null;
}

// Init Pose 를 보낼 때마다 그 위치를 도크로 기억한다
const _publishInitialPose = window.publishInitialPose;
window.publishInitialPose = function (robotId, x, y, yaw) {
    if (robots[robotId]) saveHome(robots[robotId], x, y, yaw);
    return _publishInitialPose.apply(this, arguments);
};

function waitIdle(r, ms) {
    return new Promise((resolve) => {
        const t0 = Date.now();
        const tick = () => {
            if (effectiveState(r) === 'IDLE') return resolve(true);
            if (Date.now() - t0 > ms) return resolve(false);
            setTimeout(tick, 300);
        };
        tick();
    });
}

// 한 로봇 복귀. 반환: 복귀 명령을 보냈으면 true
async function dockOne(r, quiet) {
    const n = displayName(r.id), h = getHome(r), t0 = Date.now();
    const fail = (msg) => { if (!quiet) toast(`${n}: ${msg}`, 'err'); return false; };
    if (!h) return fail('도크(초기 위치)가 없습니다. 먼저 Init Pose 를 지정하세요');
    if (!r.proc || effectiveState(r) === 'OFFLINE') return fail('로봇이 꺼져 있습니다');
    r.staged = null;

    if (r.stack === 'lane') {               // 차선 스택: 차선을 따라 편도로 도크까지 (fms_lane_mission 'home')
        if (!sendLaneCmd(r, 'home', { x: h.x, y: h.y })) return false;
        r.goalPose = { x: h.x, y: h.y, yaw: h.yaw };
        toast(`${n} RETURN DOCK: 차선을 따라 (${h.x.toFixed(2)}, ${h.y.toFixed(2)}) 로 복귀`, 'info');
        updateCard(r); refreshStartAll();
        return true;
    }

    if (navNeedsStart(r)) return fail('Nav2 가 꺼져 있습니다. Mission 을 Goal/Fleet 으로 고르면 켜집니다');
    if (r.needInitPose || !(r.status && r.status.localized)) return fail('위치(AMCL)를 모릅니다. 먼저 Init Pose 를 지정하세요');
    if (effectiveState(r) === 'BUSY') {     // 조정 층은 IDLE 로봇만 받는다 → 진행 중인 미션을 취소하고 기다린다
        sendMission('CANCEL', r.id);
        toast(`${n} RETURN DOCK: 진행 중인 미션을 취소하고 복귀합니다`, 'info');
        if (!await waitIdle(r, DOCK_IDLE_WAIT_MS)) return fail(`취소 후 ${DOCK_IDLE_WAIT_MS / 1000}초 안에 IDLE 이 되지 않아 복귀하지 못했습니다`);
        if (estopAt >= t0) return fail('E-STOP 으로 복귀를 중단했습니다');
    }
    if (!canStart(r)) return fail('대기(IDLE)이고 위치가 확인된 상태에서만 복귀합니다');
    const ok = !!launchNav(r.id, h.x, h.y, h.yaw);
    updateCard(r); refreshStartAll();
    return ok;
}

window.returnDock = function (id) {
    const r = robots[id];
    if (!r) return;
    const h = getHome(r);
    if (h && !confirm(`${displayName(id)} 를 도크(초기 위치 ${h.x.toFixed(2)}, ${h.y.toFixed(2)})로 복귀시킬까요?\n진행 중인 미션은 취소됩니다.`)) return;
    dockOne(r, false);
};

async function returnAllDock() {
    const on = Object.values(robots).filter(r => r.proc && effectiveState(r) !== 'OFFLINE');
    if (!on.length) { toast('켜진 로봇이 없습니다', 'err'); return; }
    const withHome = on.filter(r => getHome(r)), noHome = on.filter(r => !getHome(r));
    if (!withHome.length) { toast('도크(초기 위치)가 지정된 로봇이 없습니다. 먼저 Init Pose 를 지정하세요', 'err'); return; }
    const lines = withHome.map(r => { const h = getHome(r); return `  ${displayName(r.id)} → (${h.x.toFixed(2)}, ${h.y.toFixed(2)})`; }).join('\n');
    const skip = noHome.length ? `\n\n제외(Init Pose 없음): ${noHome.map(r => displayName(r.id)).join(', ')}` : '';
    if (!confirm(`${withHome.length}대를 도크(초기 위치)로 복귀시킬까요? 진행 중인 미션은 취소됩니다.\n\n${lines}${skip}`)) return;
    const res = await Promise.all(withHome.map(r => dockOne(r, false)));
    const n = res.filter(Boolean).length;
    toast(`RETURN DOCK: ${n}/${withHome.length}대 복귀 출발${n < withHome.length ? ' (실패한 로봇은 알림 확인)' : ''}`, n ? 'ok' : 'err');
}
document.getElementById('btn-return')?.addEventListener('click', returnAllDock);

// ---------- GLOBAL E-STOP 보강 ----------
// app.js 의 버튼 핸들러는 /fleet/global_cmd 'E_STOP' 만 보낸다 (조정 층이 Nav2 미션·후진·빠져나오기를 멈춘다).
// 여기서 더 하는 것: 차선 로봇 정지(lane 미션은 조정 층을 거치지 않는다), GUI 의 자동 출발 예약 해제, 조정 층 처리 확인
let estopAt = 0;
const ESTOP_ACK_MS = 2000;
let estopAckTopic = null, estopAckKey = null, estopAcked = 0;

function ensureEstopAck() {
    if (!ros) return;
    const key = `ack|${rosGeneration}`;
    if (estopAckKey === key) return;
    estopAckTopic = new ROSLIB.Topic({ ros, name: '/fleet/global_cmd_ack', messageType: 'std_msgs/msg/String' });
    estopAckTopic.subscribe((m) => {
        let d;
        try { d = JSON.parse(m.data); } catch (e) { return; }
        if (d.cmd !== 'E_STOP') return;
        estopAcked = Date.now();
        const ids = (d.stopped || []).map(displayName);
        recEvent('ros', `E_STOP ack: ${ids.length ? ids.join(', ') + ' 정지' : '진행 중인 주행 없음'}`);
    });
    estopAckKey = key;
}

document.getElementById('btn-estop')?.addEventListener('click', () => {
    if (!rosOnline) return;                             // app.js 가 이미 '연결 안 됨' 을 알렸다
    estopAt = Date.now();
    ensureEstopAck();
    const lanes = [];
    Object.values(robots).forEach(r => {
        r.pendingGo = false;                            // Init Pose 를 받으면 자동 출발하던 예약
        r.staged = null;                                // 찍어 둔 목표도 지운다 (E-STOP 뒤 실수로 Start All 방지)
        if (r.proc && r.stack === 'lane') {             // 차선 노드에 직접 정지 명령 (수신 확인까지 재전송)
            sendLaneCmd(r, 'cancel');
            r.goalPose = null;
            lanes.push(displayName(r.id));
        }
        updateCard(r);
    });
    refreshStartAll();
    if (lanes.length) toast(`E-STOP: 차선 주행 정지 명령 → ${lanes.join(', ')}`, 'err');
    const sentAt = estopAt;
    setTimeout(() => {
        if (estopAcked < sentAt) toast('E-STOP 확인 응답이 없습니다! 조정 노드(traffic_core)가 꺼져 있을 수 있습니다 — 로봇을 직접 확인하세요', 'err');
    }, ESTOP_ACK_MS);
});

// ---------- 2. REC: 주행 기록 ----------
let recState = { active: false };
const recBtn = document.getElementById('btn-rec');

function fmtElapsed(s) {
    const m = Math.floor(s / 60), ss = String(s % 60).padStart(2, '0');
    return m >= 60 ? `${Math.floor(m / 60)}:${String(m % 60).padStart(2, '0')}:${ss}` : `${m}:${ss}`;
}

function renderRec() {
    if (!recBtn) return;
    const on = recState.active;
    recBtn.className = `flex-1 ${on ? 'bg-rose-900/60 border-rose-600 text-rose-200' : 'bg-slate-700 hover:bg-slate-600 border-slate-700 text-white'} border py-2 rounded-lg font-bold text-xs transition-all flex items-center justify-center gap-1.5`;
    const mb = recState.bytes ? ` · ${(recState.bytes / 1048576).toFixed(0)} MB` : '';
    recBtn.innerHTML = on
        ? `<i class="fa-solid fa-circle text-rose-500 animate-pulse"></i> STOP REC ${fmtElapsed(recState.elapsed || 0)}${mb}`
        : `<i class="fa-solid fa-circle-dot"></i> REC LOG`;
    recBtn.title = on ? `기록 중: ${recState.dir}\n누르면 기록을 끝내고 분석용 텍스트(issues.log, summary.txt)를 만듭니다`
        : '주행 기록 시작: 관제·로봇 토픽, ROS 로그, GUI 조작을 관제PC 의 ~/pinky/logs/rec_<시각>/ 에 저장';
}

async function pollRec() {
    try {
        const prev = recState;
        recState = await api('GET', '/recording/status');
        if (recState.active && recState.recorder_alive === false && prev.recorder_alive !== false) toast('기록 프로세스(ros2 bag)가 종료됐습니다. record.log 를 확인하세요', 'err');
        if (!recState.active && prev.last && prev.last.finalizing && recState.last && !recState.last.finalizing) {
            toast(`기록 정리 완료: ${recState.last.dir}${recState.last.error ? ' (일부 오류: summary.txt 참고)' : ''}`, recState.last.error ? 'err' : 'ok');
        }
    } catch (e) { recState = { active: false, offline: true }; }
    renderRec();
}

recBtn?.addEventListener('click', async () => {
    try {
        if (recState.active) {
            recState = await api('POST', '/recording/stop');
            toast(`기록 종료: ${recState.last.dir} — 분석용 텍스트를 만드는 중 (issues.log, summary.txt)`, 'info');
        } else {
            const note = prompt('기록 메모 (선택): 시험 내용·조건을 적어 두면 분석할 때 도움이 됩니다', '');
            if (note === null) return;
            recState = await api('POST', '/recording/start', { note });
            toast(`기록 시작: ${recState.dir}`, 'ok');
        }
    } catch (e) { toast(`기록 ${recState.active ? '종료' : '시작'} 실패: ${e.message}`, 'err'); }
    renderRec();
});

// 기록 중이면 GUI 이벤트(알림·조작)를 events.log 로 보낸다. 순서가 뒤바뀌지 않게 하나씩 차례로 보낸다
let recQueue = Promise.resolve();
function recEvent(kind, msg) {
    if (!recState.active) return;
    const body = JSON.stringify({ t: Date.now(), kind, msg });
    recQueue = recQueue.then(() => fetch(BACKEND_URL + '/recording/event', {
        method: 'POST', headers: { 'Content-Type': 'application/json' }, body,
    })).catch(() => { /* 기록 실패는 무시 */ });
}

// app.js 의 전역 함수를 감싸 조작을 기록한다 (원래 동작은 그대로)
function wrapGlobal(name, describe) {
    const orig = window[name];
    if (typeof orig !== 'function') return;
    window[name] = function (...args) {
        try { recEvent('ui', describe(...args)); } catch (e) { /* 기록 실패는 무시 */ }
        return orig.apply(this, args);
    };
}
const f2 = (v) => (typeof v === 'number' ? v.toFixed(2) : v);
wrapGlobal('toast', (msg, kind) => `[${kind || 'info'}] ${msg}`);
wrapGlobal('publishInitialPose', (id, x, y, yaw) => `Init Pose ${id} (${f2(x)}, ${f2(y)}, yaw ${f2(yaw)})`);
wrapGlobal('stageGoal', (id, x, y, yaw) => `Set Goal ${id} (${f2(x)}, ${f2(y)}, yaw ${f2(yaw)})`);
wrapGlobal('goOrPrompt', (id) => `Go ${id}`);
wrapGlobal('startAllStaged', () => 'Start All');
wrapGlobal('cancelMission', (id) => `Cancel ${id}`);
wrapGlobal('returnDock', (id) => `Return Dock ${id}`);
wrapGlobal('setMissionType', (t) => `Mission ${t}`);
wrapGlobal('setTrafficMode', (m) => `Traffic Management ${m}`);
wrapGlobal('sendMission', (type, id, goal) => `mission_request ${type} ${id || '(자동 배정)'}${goal ? ` (${f2(goal.pose.position.x)}, ${f2(goal.pose.position.y)})` : ''}`);
wrapGlobal('sendLaneCmd', (r, cmd, extra) => `lane_cmd ${r.id} ${cmd}${extra ? ' ' + JSON.stringify(extra) : ''}`);
wrapGlobal('turnOff', (id) => `OFF ${id}`);
document.getElementById('btn-estop')?.addEventListener('click', () => recEvent('ui', 'GLOBAL E-STOP'));
document.getElementById('btn-return')?.addEventListener('click', () => recEvent('ui', 'RETURN DOCK (전체)'));

pollRec();
setInterval(() => { pollRec(); if (rosOnline) ensureEstopAck(); }, 2000);
