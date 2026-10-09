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

// ---- Lane 로봇 도크: 차선 추종 대신 잠시 Nav2 로 바꿔 복귀하고, 끝나면 차선 스택으로 되돌린다 ----
// (차선 'home' 편도 복귀는 갈림길 교착에서 돌아오지 못함: 2026-10-08 23:44 실물)
const DOCK_NAV_READY_MS = 90000;      // Nav2 가 켜지고 위치가 잡힐 때까지 기다리는 최대 시간
const DOCK_LANE_STOP_MS = 8000;       // 차선 미션 cancel 뒤 멈출 때까지 기다리는 최대 시간
let dockMapName = null;               // Nav2 도크용 지도: 차선 지도는 차선 밖이 미확인이라 Nav2 가 경로를 못 낸다 → *_nolanes*

async function nolanesMap() {
    if (dockMapName) return dockMapName;
    try { const m = (await api('GET', '/maps')).find(x => /_nolanes(_|$)/.test(x.name)); if (m) dockMapName = m.name; } catch (e) { /* 아래에서 현재 지도 */ }
    return dockMapName;
}

function sleep(ms) { return new Promise(res => setTimeout(res, ms)); }
async function waitFor(cond, ms) {
    const t0 = Date.now();
    while (Date.now() - t0 < ms) { if (cond()) return true; await sleep(300); }
    return cond();
}

// 지금 로봇이 믿는 위치: 차선 노드가 보낸 pose(5 s 안) → 조정 층의 AMCL 위치
function currentPose(r) {
    if (r.lane && Array.isArray(r.lane.pose) && Date.now() - r.lane.t < 5000) { const [x, y, yaw] = r.lane.pose; return { x, y, yaw }; }
    if (r.status && r.status.localized && Date.now() - (r.statusAt || 0) < 5000) return { x: r.status.x, y: r.status.y, yaw: r.status.yaw };
    return null;
}

// Nav2 도크가 끝나면(도착·실패·취소·E-STOP) 미션이 아직 Lane 이면 차선 스택으로 되돌리고 위치를 자동으로 넘긴다
async function restoreLane(r, why) {
    const p = currentPose(r);
    r.stackOverride = r.stackMapOverride = null;
    r.docking = null;
    if (missionType !== 'lane' || !r.proc) { updateCard(r); return; }
    const started = await prepareStack(r, true);           // nav → lane (백엔드가 nav 를 끄고 lane 을 띄운다)
    if (started && p) {
        _publishInitialPose(r.id, p.x, p.y, p.yaw);        // 원래 함수: 도크(home)를 덮어쓰지 않는다. Nav2 워밍업 대기는 이 함수가 한다
        toast(`${displayName(r.id)}: ${why} → 차선 스택으로 되돌리고 현재 위치(${p.x.toFixed(2)}, ${p.y.toFixed(2)})를 Init Pose 로 보냅니다`, 'info');
    } else if (started) {
        toast(`${displayName(r.id)}: ${why} → 차선 스택으로 되돌렸습니다. 위치를 몰라 Init Pose 를 직접 지정하세요`, 'err');
    }
}

async function dockLaneViaNav(r, h, t0, fail) {
    const n = displayName(r.id);
    const p = currentPose(r);
    if (!p) return fail('위치를 모릅니다 – 먼저 Init Pose 를 지정하세요');
    const aborted = () => estopAt >= t0 || !r.proc;
    r.docking = { t0 };
    sendLaneCmd(r, 'cancel');                              // 차선 주행부터 멈춘다
    toast(`${n} RETURN DOCK: 차선 주행을 멈추고 Nav2 로 도크(${h.x.toFixed(2)}, ${h.y.toFixed(2)})까지 복귀합니다 — 도크 중 다른 로봇과의 간섭 주의 (차선 주행 로봇과는 서로 피하지 않음)`, 'info');
    if (!await waitFor(() => !laneActive(r) || aborted(), DOCK_LANE_STOP_MS) || aborted()) {
        r.docking = null;
        return fail(aborted() ? 'E-STOP 으로 복귀를 중단했습니다' : '차선 주행이 멈추지 않아 복귀하지 못했습니다');
    }
    r.stackOverride = 'nav';
    r.stackMapOverride = (await nolanesMap()) || mapName();
    if (!await prepareStack(r, true)) {                    // lane → nav (실패 이유는 prepareStack 이 알린다)
        await restoreLane(r, 'Nav2 시작 실패');
        return false;
    }
    _publishInitialPose(r.id, p.x, p.y, p.yaw);            // Nav2 는 60 s 안에 초기 위치가 없으면 활성화를 포기한다 → 자동으로 보낸다
    const ready = () => r.ready && Date.now() - r.ready.t < 3000 && r.ready.nav === 'active' && canStart(r);
    if (!await waitFor(() => ready() || aborted(), DOCK_NAV_READY_MS) || aborted()) {
        await restoreLane(r, aborted() ? 'E-STOP' : 'Nav2 준비 시간 초과');
        return fail(aborted() ? 'E-STOP 으로 복귀를 중단했습니다' : `Nav2 가 ${DOCK_NAV_READY_MS / 1000}초 안에 준비되지 않아 복귀하지 못했습니다`);
    }
    const mid = launchNav(r.id, h.x, h.y, h.yaw);
    if (!mid) { await restoreLane(r, '복귀 요청 실패'); return false; }
    r.docking = { t0, mid };
    updateCard(r); refreshStartAll();
    // 이 도크 미션이 끝나면 차선 스택으로 되돌린다
    (async () => {
        const end = () => r.task && r.task.mission_id === mid && ['SUCCEEDED', 'FAILED', 'CANCELED'].includes(r.task.state);
        await waitFor(() => end() || !r.proc || (r.docking && r.docking.mid !== mid), 30 * 60000);
        if (!r.docking || r.docking.mid !== mid) return;
        const st = r.task && r.task.mission_id === mid ? r.task.state : '종료';
        await restoreLane(r, st === 'SUCCEEDED' ? '도크 도착' : `도크 ${st}`);
    })();
    return mid;
}

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

// 한 로봇 복귀. 반환: 보낸 도크 미션 id (실패하면 false). 순서는 아래 도크 큐가 정한다
async function dockOne(r, quiet) {
    const n = displayName(r.id), h = getHome(r), t0 = Date.now();
    const fail = (msg) => { if (!quiet) toast(`${n}: ${msg}`, 'err'); return false; };
    if (!h) return fail('도크(초기 위치)가 없습니다. 먼저 Init Pose 를 지정하세요');
    if (!r.proc || effectiveState(r) === 'OFFLINE') return fail('로봇이 꺼져 있습니다');
    if (r.docking) return fail('이미 도크로 복귀 중입니다 (멈추려면 Cancel)');
    r.staged = null;

    if (r.stack === 'lane') return dockLaneViaNav(r, h, t0, fail);    // 차선 로봇: 잠시 Nav2 로 바꿔 복귀

    if (navNeedsStart(r)) return fail('Nav2 가 꺼져 있습니다. Mission 을 Goal/Fleet 으로 고르면 켜집니다');
    if (r.needInitPose || !(r.status && r.status.localized)) return fail('위치(AMCL)를 모릅니다. 먼저 Init Pose 를 지정하세요');
    if (effectiveState(r) === 'BUSY') {     // 조정 층은 IDLE 로봇만 받는다 → 진행 중인 미션을 취소하고 기다린다
        sendMission('CANCEL', r.id);
        toast(`${n} RETURN DOCK: 진행 중인 미션을 취소하고 복귀합니다`, 'info');
        if (!await waitIdle(r, DOCK_IDLE_WAIT_MS)) return fail(`취소 후 ${DOCK_IDLE_WAIT_MS / 1000}초 안에 IDLE 이 되지 않아 복귀하지 못했습니다`);
        if (estopAt >= t0) return fail('E-STOP 으로 복귀를 중단했습니다');
    }
    if (!canStart(r)) return fail('대기(IDLE)이고 위치가 확인된 상태에서만 복귀합니다');
    const mid = launchNav(r.id, h.x, h.y, h.yaw);
    updateCard(r); refreshStartAll();
    return mid || false;
}

// ---- 도크 큐: 한 대씩 순서대로 (동시에 보내면 좁은 트랙에서 경로가 엇갈려 마주침 양보에 걸린다: 2026-10-09 00:34 실물) ----
const DOCK_STEP_MS = 90000;           // 앞 로봇이 출발한 뒤 이 시간이 지나면 끝나지 않았어도 다음 로봇을 보낸다
const dockQueue = [];                 // 기다리는 로봇 id (앞에서부터)
let dockActive = null, dockRunning = false;

function dockDist(r) {
    const p = currentPose(r), h = getHome(r);
    return p && h ? Math.hypot(p.x - h.x, p.y - h.y) : Infinity;
}

// 기다리는 동안은 그 자리에 세워 둔다 (차선 주행은 cancel, Nav2 미션은 취소)
function holdForDock(r) {
    if (laneActive(r)) sendLaneCmd(r, 'cancel');
    else if (effectiveState(r) === 'BUSY') sendMission('CANCEL', r.id);
    r.staged = null;
}

function enqueueDock(list) {
    const added = list.filter(r => r.id !== dockActive && !dockQueue.includes(r.id));
    added.forEach(r => { dockQueue.push(r.id); if (dockActive || dockQueue[0] !== r.id) holdForDock(r); });
    Object.values(robots).forEach(updateCard);
    runDockQueue();
    return added;
}

async function runDockQueue() {
    if (dockRunning) return;
    dockRunning = true;
    try {
        while (dockQueue.length) {
            const id = dockQueue.shift(), r = robots[id];
            if (!r) continue;
            dockActive = id;
            Object.values(robots).forEach(updateCard);
            const t0 = Date.now();
            const mid = await dockOne(r, false);
            if (mid && estopAt < t0) {
                const end = () => r.task && r.task.mission_id === mid && ['SUCCEEDED', 'FAILED', 'CANCELED'].includes(r.task.state);
                const done = await waitFor(() => end() || estopAt >= t0 || !r.proc, DOCK_STEP_MS);
                if (!done && dockQueue.length) toast(`${displayName(id)} 도크가 ${DOCK_STEP_MS / 1000}초 안에 끝나지 않아 다음 로봇을 보냅니다`, 'err');
            }
            dockActive = null;
            if (estopAt >= t0) {                       // E-STOP: 남은 도크는 모두 취소
                if (dockQueue.length) toast(`E-STOP: 도크 대기 ${dockQueue.length}대 취소 (${dockQueue.map(displayName).join(', ')})`, 'err');
                dockQueue.length = 0;
            }
        }
    } finally {
        dockActive = null; dockRunning = false;
        Object.values(robots).forEach(updateCard);
    }
}

window.returnDock = function (id) {
    const r = robots[id];
    if (!r) return;
    if (id === dockActive || dockQueue.includes(id)) { toast(`${displayName(id)}: 이미 도크 ${id === dockActive ? '중' : '대기 중'}입니다 (멈추려면 Cancel)`, 'info'); return; }
    const h = getHome(r);
    if (!h) { toast(`${displayName(id)}: 도크(초기 위치)가 없습니다. 먼저 Init Pose 를 지정하세요`, 'err'); return; }
    const laneNote = r.stack === 'lane' ? '\n차선 주행 중이면 Nav2 로 잠시 바꿔 복귀한 뒤 차선 스택으로 되돌립니다. 도크 중 다른 로봇과의 간섭에 주의하세요.' : '';
    const busy = dockActive || dockQueue.length;
    const queueNote = busy ? `\n다른 로봇이 도크 중이라 대기열 ${dockQueue.length + 1}번째로 들어가고, 차례가 올 때까지 그 자리에 멈춰 있습니다.` : '';
    if (!confirm(`${displayName(id)} 를 도크(초기 위치 ${h.x.toFixed(2)}, ${h.y.toFixed(2)})로 복귀시킬까요?\n진행 중인 미션은 취소됩니다.${laneNote}${queueNote}`)) return;
    enqueueDock([r]);
    if (busy && dockQueue.includes(id)) toast(`${displayName(id)} 도크 대기 (${dockQueue.indexOf(id) + 1 + (dockActive ? 1 : 0)}번째): 앞 로봇의 도크가 끝나면 출발합니다`, 'info');
};

async function returnAllDock() {
    const on = Object.values(robots).filter(r => r.proc && effectiveState(r) !== 'OFFLINE');
    if (!on.length) { toast('켜진 로봇이 없습니다', 'err'); return; }
    const withHome = on.filter(r => getHome(r)).sort((a, b) => dockDist(a) - dockDist(b));     // 도크에 가까운 로봇부터
    const noHome = on.filter(r => !getHome(r));
    if (!withHome.length) { toast('도크(초기 위치)가 지정된 로봇이 없습니다. 먼저 Init Pose 를 지정하세요', 'err'); return; }
    const lines = withHome.map((r, i) => { const h = getHome(r); return `  ${i + 1}. ${displayName(r.id)} → (${h.x.toFixed(2)}, ${h.y.toFixed(2)})`; }).join('\n');
    const skip = noHome.length ? `\n\n제외(Init Pose 없음): ${noHome.map(r => displayName(r.id)).join(', ')}` : '';
    const laneNote = withHome.some(r => r.stack === 'lane') ? '\n차선 로봇은 Nav2 로 잠시 바꿔 복귀합니다.' : '';
    if (!confirm(`${withHome.length}대를 한 대씩 순서대로 도크(초기 위치)로 복귀시킬까요? 진행 중인 미션은 취소되고, 차례를 기다리는 로봇은 그 자리에 멈춰 있습니다.${laneNote}\n\n${lines}${skip}`)) return;
    const added = enqueueDock(withHome);
    toast(`RETURN DOCK: ${added.length}대를 순서대로 복귀시킵니다 (${withHome.map(r => displayName(r.id)).join(' → ')})`, 'info');
}
document.getElementById('btn-return')?.addEventListener('click', returnAllDock);

// ---------- GLOBAL E-STOP 보강 ----------
// app.js 의 버튼 핸들러는 /fleet/global_cmd 'E_STOP' 만 보낸다 (조정 층이 Nav2 미션·후진·빠져나오기를 멈춘다).
// 여기서 더 하는 것: 차선 로봇 정지(lane 미션은 조정 층을 거치지 않는다), GUI 의 자동 출발 예약 해제, 조정 층 처리 확인
let estopAt = 0;
const ESTOP_ACK_MS = 2000;
let estopAckTopic = null, estopAckKey = null, estopAcked = 0, estopAckShownFor = 0, estopLaneSent = [];

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
        const lane = new Set([...(d.lane || []).map(displayName), ...estopLaneSent]);     // 조정 층이 본 차선 로봇 + GUI 가 cancel 을 보낸 로봇
        const parts = [];
        if (ids.length) parts.push(`Nav2 주행 ${ids.length}대 정지 (${ids.join(', ')})`);
        if (lane.size) parts.push(`차선 주행 ${lane.size}대 정지 요청 (${[...lane].join(', ')})`);
        const msg = `E-STOP 확인: ${parts.length ? parts.join(' · ') : '진행 중인 주행 없음'}`;
        if (estopAckShownFor !== estopAt) { estopAckShownFor = estopAt; toast(msg, 'info'); }   // ack 가 여러 번 와도 한 번만
        else recEvent('ros', msg);
    });
    estopAckKey = key;
}

document.getElementById('btn-estop')?.addEventListener('click', () => {
    if (!rosOnline) return;                             // app.js 가 이미 '연결 안 됨' 을 알렸다
    estopAt = Date.now();
    ensureEstopAck();
    if (dockQueue.length) { toast(`E-STOP: 도크 대기 ${dockQueue.length}대 취소 (${dockQueue.map(displayName).join(', ')})`, 'err'); dockQueue.length = 0; }
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
    estopLaneSent = lanes;
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
        : '주행 기록 시작: 관제·로봇 토픽, ROS 로그, GUI 조작, 주행 영상(차선 주행 중)을 관제PC 의 ~/pinky/logs/rec_<시각>/ 에 저장';
}

async function pollRec() {
    try {
        const prev = recState;
        recState = await api('GET', '/recording/status');
        if (recState.active && recState.recorder_alive === false && prev.recorder_alive !== false) toast('기록 프로세스(ros2 bag)가 종료됐습니다. record.log 를 확인하세요', 'err');
        if (!recState.active && prev.last && prev.last.finalizing && recState.last && !recState.last.finalizing) {
            const v = recState.last.videos || [];
            const vid = v.length ? ` · 영상 ${v.join(', ')}` : ' · 영상 없음 (차선 주행 스택이 떠 있을 때만 녹화)';
            toast(`기록 정리 완료: ${recState.last.dir}${vid}${recState.last.error ? ' (일부 오류: summary.txt 참고)' : ''}`, recState.last.error ? 'err' : 'ok');
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
const _cancelMission = window.cancelMission;
window.cancelMission = function (id) {
    const i = dockQueue.indexOf(id);
    if (i >= 0) { dockQueue.splice(i, 1); toast(`${displayName(id)} 도크 대기 취소`, 'info'); Object.values(robots).forEach(updateCard); }
    return _cancelMission.apply(this, arguments);
};
wrapGlobal('returnDock', (id) => `Return Dock ${id}`);
wrapGlobal('setMissionType', (t) => `Mission ${t}`);
wrapGlobal('setTrafficMode', (m) => `Traffic Management ${m}`);
wrapGlobal('sendMission', (type, id, goal) => `mission_request ${type} ${id || '(자동 배정)'}${goal ? ` (${f2(goal.pose.position.x)}, ${f2(goal.pose.position.y)})` : ''}`);
wrapGlobal('sendLaneCmd', (r, cmd, extra) => `lane_cmd ${r.id} ${cmd}${extra ? ' ' + JSON.stringify(extra) : ''}`);
wrapGlobal('turnOff', (id) => `OFF ${id}`);
document.getElementById('btn-estop')?.addEventListener('click', () => recEvent('ui', 'GLOBAL E-STOP'));
document.getElementById('btn-return')?.addEventListener('click', () => recEvent('ui', 'RETURN DOCK (전체)'));

// ---------- 차선 이탈 경고: /fleet/lane_traffic_state 의 alerts [{robot, reason: off_lane|lost, pose}] ----------
const ALERT_TEXT = { off_lane: '차선 이탈', lost: '차선 인식 실패' };
let laneStateKey = null;
function ensureLaneState() {
    if (!ros) return;
    const key = `lts|${rosGeneration}`;
    if (laneStateKey === key) return;
    const t = new ROSLIB.Topic({ ros, name: '/fleet/lane_traffic_state', messageType: 'std_msgs/msg/String' });
    t.subscribe((m) => {
        let d;
        try { d = JSON.parse(m.data); } catch (e) { return; }
        const now = Date.now(), seen = new Set();
        (Array.isArray(d.alerts) ? d.alerts : []).forEach(a => {
            const r = robots[a && a.robot];
            if (!r) return;
            seen.add(r.id);
            const reason = ALERT_TEXT[a.reason] || a.reason || '이상';
            if (!r.laneAlert || r.laneAlert.reason !== reason) {
                toast(`${displayName(r.id)}: ${reason} – 수동 개입 필요`, 'err');
            }
            r.laneAlert = { reason, t: now };
            updateCard(r);
        });
        Object.values(robots).forEach(r => { if (r.laneAlert && !seen.has(r.id)) { r.laneAlert = null; updateCard(r); } });
    });
    laneStateKey = key;
}

// 카드에 차선 이탈 배지 (app.js 카드 템플릿은 그대로 두고 상태 배지 뒤에 붙인다)
const _updateCard = window.updateCard;
window.updateCard = function (r) {
    const ret = _updateCard.apply(this, arguments);
    const anchor = document.getElementById(`${r.id}-errbadge`);
    if (anchor) {
        let b = document.getElementById(`${r.id}-lanealert`);
        if (!b) {
            b = document.createElement('span');
            b.id = `${r.id}-lanealert`;
            b.className = 'hidden ml-1 text-[10px] bg-rose-600 text-white px-1.5 py-0.5 rounded font-bold animate-pulse';
            anchor.after(b);
        }
        const a = r.laneAlert && Date.now() - r.laneAlert.t < 3000 ? r.laneAlert : null;     // 상태가 끊기면 3초 뒤 지운다
        b.classList.toggle('hidden', !a);
        if (a) { b.textContent = `${a.reason} – 수동 개입 필요`; b.title = '로봇을 차선 위로 옮긴 뒤 Init Pose 를 다시 지정하세요'; }
        let q = document.getElementById(`${r.id}-dockq`);
        if (!q) {
            q = document.createElement('span');
            q.id = `${r.id}-dockq`;
            q.className = 'hidden ml-1 text-[10px] bg-emerald-500/20 text-emerald-300 px-1.5 py-0.5 rounded font-bold';
            b.after(q);
        }
        const qi = dockQueue.indexOf(r.id);
        const label = r.id === dockActive ? '도크 중' : qi >= 0 ? `도크 대기 (${qi + 1 + (dockActive ? 1 : 0)}번째)` : '';
        q.classList.toggle('hidden', !label);
        q.textContent = label;
    }
    return ret;
};

pollRec();
setInterval(() => { pollRec(); if (rosOnline) { ensureEstopAck(); ensureLaneState(); } }, 2000);
