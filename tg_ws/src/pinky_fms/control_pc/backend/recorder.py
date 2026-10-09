"""주행 기록 (GUI 의 REC 버튼): 시험 중 관제·로봇 토픽을 관제PC 로컬에 저장하고, 끝나면 분석용 텍스트로 정리한다.

  POST /recording/start  {note}   → ~/pinky/logs/rec_<시각>/ 에 rosbag(mcap) 기록 시작
  POST /recording/event  {t, kind, msg}  → GUI 이벤트(버튼 조작·알림)를 events.log 에 추가
  POST /recording/stop            → 기록 종료 + rosout/미션 상태를 텍스트로 변환 + 로봇 launch 로그 수집
  GET  /recording/status

저장 폴더 내용:
  bag/            rosbag2 (mcap, 10분마다 분할) — 위치·경로·속도·스캔·BT·관제 토픽·/rosout
  events.log      GUI 에서 한 조작과 화면에 뜬 알림 (시각순)
  rosout.log      모든 ROS 로그 (stop 후 변환)
  issues.log      WARN 이상 ROS 로그 + 실패한 미션 (분석은 여기부터)
  missions.log    /fleet/mission_state 변화
  launch/<로봇>_<스택>.log   로봇에서 실행한 launch 출력 끝부분 (stop 때 SSH 로 1회 수집)
  video_<로봇>.mp4  주행 영상 (camera/rec, 차선 주행 스택이 떠 있을 때만 나온다. 실제 시각 기준 5 fps)
  summary.txt     기간·로그 수준별 개수·노드별 경고 수·미션 결과·영상
ROS 통신은 별도 프로세스(ros2 bag, 변환 스크립트)에서 한다. 백엔드 자체는 ROS 를 import 하지 않는다.
"""
import json
import os
import shlex
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

HERE = Path(__file__).resolve().parent
LOG_ROOT = Path(os.environ.get('FMS_REC_DIR', Path.home() / 'pinky/logs'))
ROS_SETUP = os.environ.get('FMS_ROS_SETUP', '/opt/ros/jazzy/setup.bash')
WS_SETUP = os.environ.get('FMS_WS_SETUP', str(Path.home() / 'pinky/install/setup.bash'))
ROBOTS_FILE = Path(os.environ.get(
    'FMS_ROBOTS_FILE', HERE.parents[0] / 'ros/pinky_fms_core/config/robots.yaml'))
ENV_SCRIPT = HERE.parents[0] / 'fms_env.sh'
# 기록할 토픽: 관제 토픽 전부 + 로봇 namespace 아래 분석에 쓰는 것.
# 영상은 차선 노드의 기록용 camera/rec(JPEG 320×240, 5 Hz)만. 원본 camera/compressed 는 크고 로봇 부하가 커서 제외
TOPICS = (r'^/fleet/.*|^/rosout$|^/[a-z][a-z0-9_]*/(amcl_pose|plan|received_global_plan|cmd_vel|cmd_vel_nav|odom|scan'
          r'|scan_robotfree|behavior_tree_log|other_robots|fms_status|lane_status|lane_cmd|lane_traffic_cmd|escape_status|escape_cmd'
          r'|initialpose|goal_pose|tf|tf_static|battery_state|camera/rec|navigate_to_pose/_action/status'
          r'|navigate_through_poses/_action/status|backup/_action/status|drive_on_heading/_action/status)$')
MAX_EVENTS = 20000
ACTIVE_FILE = LOG_ROOT / '.fms_rec_active.json'     # 백엔드가 기록 중에 재시작돼도 남은 ros2 bag 을 찾아 끝내기 위함

router = APIRouter()
_lock = threading.Lock()
_rec = None          # {'dir', 'proc', 'started', 'note', 'events'}
_last = None         # 마지막으로 끝난 기록 {'dir', 'stopped', 'finalizing', 'error'}


def _env_prefix():
    return (f'source {shlex.quote(ROS_SETUP)} && '
            f'{{ test -f {shlex.quote(WS_SETUP)} && source {shlex.quote(WS_SETUP)} || true; }} && '
            f'source {shlex.quote(str(ENV_SCRIPT))} {shlex.quote(str(ROBOTS_FILE))} >/dev/null 2>&1')


def _alive(p):
    return p is not None and p.poll() is None


def _size(d):
    try:
        return sum(f.stat().st_size for f in Path(d).rglob('*') if f.is_file())
    except OSError:
        return 0


def _status():
    if _rec:
        return {'active': True, 'dir': str(_rec['dir']), 'started': _rec['started'], 'note': _rec['note'],
                'elapsed': round(time.time() - _rec['started']), 'bytes': _size(_rec['dir']),
                'recorder_alive': _alive(_rec['proc'])}
    return {'active': False, 'last': _last}


class StartBody(BaseModel):
    note: str = ''


class EventBody(BaseModel):
    t: float = 0          # 브라우저 시각 (ms, epoch)
    kind: str = 'info'
    msg: str = ''


@router.get('/recording/status')
def rec_status():
    with _lock:
        if _rec and not _alive(_rec['proc']):
            _rec['error'] = 'ros2 bag 이 종료됨 (record.log 확인)'
        return _status()


@router.post('/recording/start')
def rec_start(body: StartBody):
    global _rec
    with _lock:
        if _rec:
            raise HTTPException(409, f'이미 기록 중: {_rec["dir"]}')
        if _last and _last.get('finalizing'):
            raise HTTPException(409, '이전 기록을 정리하는 중입니다. 잠시 뒤 다시 누르세요')
        d = LOG_ROOT / f'rec_{datetime.now():%Y%m%d_%H%M%S}'
        d.mkdir(parents=True, exist_ok=False)
        note = body.note.strip()[:500]
        (d / 'note.txt').write_text(note + '\n', encoding='utf-8')
        cmd = (f'{_env_prefix()} && echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION" && '
               f'exec ros2 bag record -o {shlex.quote(str(d / "bag"))} --storage mcap --max-bag-duration 600 '
               f'--include-hidden-topics -e {shlex.quote(TOPICS)}')
        log = open(d / 'record.log', 'w')
        # 새 프로세스 그룹: 종료 때 그룹 전체에 SIGINT (ros2 bag 이 mcap 을 마무리하도록)
        proc = subprocess.Popen(['bash', '-c', cmd], stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                start_new_session=True)
        log.close()
        ACTIVE_FILE.write_text(json.dumps({'pid': proc.pid, 'dir': str(d)}), encoding='utf-8')
        _rec = {'dir': d, 'proc': proc, 'started': time.time(), 'note': note, 'events': 0}
        _event(_rec, time.time() * 1000, 'rec', f'기록 시작 {note}')
        return _status()


def _event(rec, t_ms, kind, msg):
    if rec['events'] >= MAX_EVENTS:
        return
    rec['events'] += 1
    ts = datetime.fromtimestamp((t_ms or time.time() * 1000) / 1000).strftime('%H:%M:%S.%f')[:-3]
    line = f'{ts} [{kind[:12]:<5}] {" ".join(msg.split())[:2000]}\n'
    with open(rec['dir'] / 'events.log', 'a', encoding='utf-8') as f:
        f.write(line)


@router.post('/recording/event')
def rec_event(body: EventBody):
    with _lock:
        if not _rec:
            return {'active': False}
        _event(_rec, body.t, body.kind, body.msg)
        return {'active': True}


@router.post('/recording/stop')
def rec_stop():
    global _rec, _last
    with _lock:
        if not _rec:
            raise HTTPException(409, '기록 중이 아닙니다')
        rec, _rec = _rec, None
        _event(rec, time.time() * 1000, 'rec', '기록 종료')
        _last = {'dir': str(rec['dir']), 'stopped': time.time(), 'finalizing': True, 'error': None}
    threading.Thread(target=_finalize, args=(rec,), daemon=True).start()
    return {'active': False, 'last': _last}


def _stop_proc(p):
    if not _alive(p):
        return
    for sig, wait in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
        try:
            os.killpg(p.pid, sig)
        except ProcessLookupError:
            return
        try:
            p.wait(wait)
            return
        except subprocess.TimeoutExpired:
            pass


def _finalize(rec):
    d, errors = rec['dir'], []
    _stop_proc(rec['proc'])
    ACTIVE_FILE.unlink(missing_ok=True)
    try:
        _collect_launch_logs(d)
    except Exception as e:                      # 로봇이 꺼져 있거나 SSH 실패: 기록 자체는 유지
        errors.append(f'launch 로그 수집 실패: {type(e).__name__}: {e}')
    if (d / 'bag').exists():
        cmd = (f'{_env_prefix()} && exec python3 {shlex.quote(str(HERE / "rec_extract.py"))} '
               f'{shlex.quote(str(d))} {rec["started"]:.3f}')
        try:
            r = subprocess.run(['bash', '-c', cmd], capture_output=True, text=True, timeout=900)
            if r.returncode != 0:
                errors.append(f'로그 변환 실패: {(r.stderr or r.stdout).strip()[-500:]}')
        except subprocess.TimeoutExpired:
            errors.append('로그 변환 시간 초과 (900 s)')
    else:
        errors.append('bag 폴더가 없음: ros2 bag 이 시작되지 못함 (record.log 확인)')
    if errors:
        with open(d / 'summary.txt', 'a', encoding='utf-8') as f:
            f.write('\n[기록 처리 오류]\n' + '\n'.join(errors) + '\n')
    with _lock:
        if _last and _last['dir'] == str(d):
            _last.update(finalizing=False, error='; '.join(errors) or None, bytes=_size(d),
                         videos=sorted(f.name for f in d.glob('video_*.mp4')))


def _collect_launch_logs(d):
    """켜져 있는 로봇마다 bringup 과 주행 스택의 launch 출력 끝부분을 가져온다 (SSH 1회/로봇)"""
    app = sys.modules.get('app')
    if app is None:
        return
    launched = app._read('launched.json', {})
    if not launched:
        return
    out_dir = d / 'launch'
    out_dir.mkdir(exist_ok=True)
    for rid in launched:
        try:
            _, r = app.get_robot(rid)
            run = app.connect(rid, r)
            files = [('bringup', app.paths(rid)[1])] + [(s, app.stack_paths(rid, s)[1]) for s in app.stacks_of(r)]
            for name, path in files:
                _, out, _ = run.run(f'tail -n 3000 {shlex.quote(path)} 2>/dev/null', timeout=20)
                if out.strip():
                    (out_dir / f'{rid}_{name}.log').write_text(out, encoding='utf-8')
        except Exception as e:
            (out_dir / f'{rid}_ERROR.txt').write_text(f'{type(e).__name__}: {e}\n', encoding='utf-8')


def _stop_leftover():
    """이전 백엔드 프로세스가 기록 중에 끝났으면 그 ros2 bag(프로세스 그룹)에 SIGINT 를 보내 마무리시킨다"""
    try:
        info = json.loads(ACTIVE_FILE.read_text(encoding='utf-8'))
        os.killpg(int(info['pid']), signal.SIGINT)
        with open(Path(info['dir']) / 'events.log', 'a', encoding='utf-8') as f:
            f.write(f'{datetime.now():%H:%M:%S.000} [rec  ] 백엔드 재시작으로 기록 중단 (텍스트 변환 안 됨)\n')
    except (OSError, ValueError, KeyError):
        pass
    ACTIVE_FILE.unlink(missing_ok=True)


_stop_leftover()
