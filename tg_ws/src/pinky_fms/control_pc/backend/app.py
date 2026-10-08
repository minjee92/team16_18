"""FMS 백엔드: 로봇 등록(추가/삭제), 로봇 프로세스 켜기·끄기(SSH), launch 출력, ping, 맵. ROS 통신은 하지 않는다.

  .venv/bin/uvicorn app:app --host 127.0.0.1 --port 8000
- 127.0.0.1 에만 바인딩한다.
- 비밀번호는 파일·로그에 남기지 않는다. 로봇이 켜져 있는 동안 재접속용으로 메모리에만 두고, OFF/삭제 때 지운다.
- 로봇에서 실행하는 명령은 robots.yaml(defaults, robots)에 정의한 것뿐이다. GUI 는 임의 명령을 보낼 수 없다.
- 로봇 ID 가 곧 ROS namespace 다 (/amr_01/...). 조정 노드는 토픽 이름으로 로봇을 자동 발견한다.
"""
import base64
import json
import io
import hashlib
import os
import re
import shlex
import socket
import subprocess
import threading
import time
from collections import deque
from pathlib import Path

import paramiko
import yaml
from fastapi import FastAPI, File, HTTPException, UploadFile
from fastapi.responses import FileResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

import netconf

ROBOTS_FILE = Path(os.environ.get(
    'FMS_ROBOTS_FILE', Path(__file__).resolve().parents[1] / 'ros/pinky_fms_core/config/robots.yaml'))
KEY_PATH = Path(os.environ.get('FMS_SSH_KEY', Path.home() / '.ssh/fms_ed25519'))
# 실행 중 상태(추가한 로봇, 마지막 접속 정보, 켠 로봇). 설정 파일마다 따로 둔다 (데모와 실물이 섞이지 않게)
STATE_DIR = Path(os.environ.get('FMS_STATE_DIR', Path(__file__).resolve().parent / 'state' / ROBOTS_FILE.stem))

app = FastAPI(title='FMS backend')
app.add_middleware(CORSMiddleware, allow_origins=['*'], allow_methods=['*'], allow_headers=['*'])
from recorder import router as recording_router   # 주행 기록 (REC 버튼): /recording/*
app.include_router(recording_router)
ID_RE = re.compile(r'^[A-Za-z0-9_]+$')                 # 맵 이름
NS_RE = re.compile(r'^[a-z][a-z0-9_]{0,31}$')          # 로봇 ID = ROS namespace
HOST_RE = re.compile(r'^[0-9A-Za-z.\-]{1,253}$')
USER_RE = re.compile(r'^[a-z_][a-z0-9_\-]{0,31}$')
PKG_RE = re.compile(r'ros2\s+launch\s+(\S+)')
STACK_RE = re.compile(r'^[a-z_]{1,16}$')           # 주행 스택 이름 (robots.yaml defaults.stacks 의 키)


# ---------------- 상태 파일 (robots.yaml 은 수정하지 않는다) ----------------
_state_lock = threading.Lock()


def _read(name, default):
    try:
        return json.loads((STATE_DIR / name).read_text(encoding='utf-8'))
    except Exception:
        return default


def _update(name, fn):
    """읽고-고치고-쓰기를 잠금 안에서 한다. 임시 파일에 쓴 뒤 교체하므로, 동시에 읽는 쪽은 항상 완전한 파일을 본다."""
    with _state_lock:
        d = _read(name, {})
        fn(d)
        STATE_DIR.mkdir(parents=True, exist_ok=True)
        tmp = STATE_DIR / f'.{name}.tmp'
        tmp.write_text(json.dumps(d, indent=1, ensure_ascii=False), encoding='utf-8')
        os.replace(tmp, STATE_DIR / name)


# ---------------- 로봇 목록: robots.yaml 의 고정 로봇 + GUI 에서 추가한 로봇 ----------------
def load():
    return yaml.safe_load(ROBOTS_FILE.read_text(encoding='utf-8')) or {}


def defaults(cfg):
    return {'user': 'pinky', 'mode': 'ssh', **(cfg.get('defaults') or {})}


def registry():
    cfg = load()
    base = defaults(cfg)
    reg = {}
    for rid, r in (cfg.get('robots') or {}).items():
        reg[str(rid)] = {**base, **(r or {}), 'preset': True}
    for rid, r in _read('added.json', {}).items():
        reg.setdefault(rid, {**base, **r, 'preset': False})
    for rid, c in _read('last_conn.json', {}).items():
        if rid in reg:
            reg[rid].update({k: v for k, v in c.items() if v})
    for rid, r in reg.items():
        r['id'] = rid
    return cfg, reg


def get_robot(rid):
    cfg, reg = registry()
    r = reg.get(rid)
    if r is None:
        raise HTTPException(404, f'등록되지 않은 로봇: {rid}')
    for k in ('ws_setup', 'launch_cmd'):
        if not r.get(k):
            raise HTTPException(500, f'robots.yaml 에 {k} 가 없습니다 (defaults 또는 로봇 항목에 지정)')
    return cfg, r


def local_ip_for(remote_ip):
    """remote_ip 로 나가는 경로에서 관제PC 가 쓰는 IP. 네트워크(직결/공유기/핫스팟)가 바뀌어도 자동으로 따라간다."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((remote_ip, 9))      # UDP 라서 실제 패킷은 보내지 않는다
        return s.getsockname()[0]
    finally:
        s.close()


def dds_setup(cfg, rid, r, run, ip):
    """유니캐스트 모드면 로봇에 Cyclone DDS 설정 파일을 만들고, launch 앞에 붙일 export 문을 돌려준다."""
    net = netconf.network_cfg(cfg)
    if net['discovery'] != 'unicast':
        return '', net
    local = r.get('mode', 'ssh') == 'local'
    pc_ip = '127.0.0.1' if local else local_ip_for(ip)
    path = f'/tmp/fms_cyclonedds_{rid}.xml'
    b64 = base64.b64encode(netconf.robot_xml(net, pc_ip, '' if local else ip).encode()).decode()
    code, _, err = run.run(f'echo {b64} | base64 -d > {path}')
    if code != 0:
        raise HTTPException(502, f'로봇에 DDS 설정 파일을 만들지 못함: {err.strip()[:120]}')
    return f'export RMW_IMPLEMENTATION={netconf.RMW} CYCLONEDDS_URI=file://{path} && ', net


# ---------------- ping 모니터: ON 이후부터 OFF 까지 1초마다 로봇 IP 로 ping ----------------
PING_WINDOW = 60
_RTT_RE = re.compile(r'time[=<]([\d.]+)\s*ms')


def probe(ip):
    """RTT(ms) 를 돌려준다. 응답이 없으면 None. ICMP ping 이 없으면 SSH 포트(22) TCP 연결 시간으로 대체."""
    try:
        p = subprocess.run(['ping', '-c', '1', '-W', '1', ip], capture_output=True, text=True, timeout=3)
        m = _RTT_RE.search(p.stdout)
        return float(m.group(1)) if (p.returncode == 0 and m) else None
    except FileNotFoundError:
        t0 = time.monotonic()
        try:
            with socket.create_connection((ip, 22), timeout=1):
                return (time.monotonic() - t0) * 1000
        except OSError:
            return None
    except subprocess.TimeoutExpired:
        return None


class PingMonitor(threading.Thread):
    def __init__(self, ip):
        super().__init__(daemon=True)
        self.ip = ip
        self.samples = deque(maxlen=PING_WINDOW)   # RTT(ms) 또는 None(손실)
        self.stop_ev = threading.Event()

    def run(self):
        while not self.stop_ev.is_set():
            t0 = time.monotonic()
            self.samples.append(probe(self.ip))
            self.stop_ev.wait(max(0.0, 1.0 - (time.monotonic() - t0)))

    def stop(self):
        self.stop_ev.set()


monitors = {}   # rid -> PingMonitor
_mon_lock = threading.Lock()


def start_monitor(rid, ip):
    with _mon_lock:
        old = monitors.get(rid)
        if old and old.ip == ip and old.is_alive():
            return
        if old:
            old.stop()
        m = PingMonitor(ip)
        monitors[rid] = m
        m.start()


def stop_monitor(rid):
    with _mon_lock:
        m = monitors.pop(rid, None)
    if m:
        m.stop()


# ---------------- 실행기: SSH(연결 재사용) / 로컬 ----------------
class LocalRunner:
    def run(self, cmd, timeout=20):
        p = subprocess.run(['bash', '-lc', cmd], capture_output=True, text=True, timeout=timeout)
        return p.returncode, p.stdout, p.stderr


class SSHConn:
    def __init__(self, ip, user, password=None, timeout=6):
        self.ip, self.user = ip, user
        self.c = paramiko.SSHClient()
        self.c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        kw = dict(hostname=ip, username=user, timeout=timeout, banner_timeout=timeout, auth_timeout=timeout)
        if password:
            kw.update(password=password, look_for_keys=False, allow_agent=False)
        elif KEY_PATH.exists():
            kw.update(key_filename=str(KEY_PATH))
        try:
            self.c.connect(**kw)
        except paramiko.AuthenticationException:
            self.c.close()
            raise HTTPException(401, 'SSH 인증 실패: 비밀번호를 확인하세요' if password
                                else '비밀번호가 필요합니다 (이 로봇에 등록된 SSH 키 없음)')
        except Exception as e:
            self.c.close()
            raise HTTPException(502, f'SSH 접속 실패 ({ip}): {type(e).__name__}')

    def alive(self):
        t = self.c.get_transport()
        return t is not None and t.is_active()

    def run(self, cmd, timeout=20):
        try:
            _, out, err = self.c.exec_command(f'bash -lc {shlex.quote(cmd)}', timeout=timeout)
            o, e = out.read().decode(errors='replace'), err.read().decode(errors='replace')
            return out.channel.recv_exit_status(), o, e
        except (paramiko.SSHException, OSError, EOFError) as e:
            self.close()
            raise HTTPException(502, f'SSH 연결이 끊김 ({self.ip}): {type(e).__name__}')

    def install_key(self):
        """이 PC 의 공개키를 로봇에 등록해 다음부터 비밀번호 없이 접속하게 한다 (GUI 체크박스로 선택)."""
        if not KEY_PATH.exists():
            subprocess.run(['ssh-keygen', '-t', 'ed25519', '-N', '', '-f', str(KEY_PATH), '-q'], check=True)
        pub = KEY_PATH.with_suffix('.pub').read_text().strip()
        self.run('mkdir -p ~/.ssh && chmod 700 ~/.ssh && touch ~/.ssh/authorized_keys && '
                 f'grep -qxF {shlex.quote(pub)} ~/.ssh/authorized_keys || echo {shlex.quote(pub)} >> ~/.ssh/authorized_keys '
                 '&& chmod 600 ~/.ssh/authorized_keys')

    def close(self):
        self.c.close()


_conns = {}         # rid -> SSHConn   (켜져 있는 동안 재사용)
_secrets = {}       # rid -> 비밀번호 (메모리에만. OFF/삭제 때 지운다)
_needs_auth = set()  # 인증 실패한 로봇: 비밀번호를 다시 받을 때까지 자동 재접속하지 않는다
_retry_at = {}      # rid -> 네트워크 실패 후 다음 자동 재시도 시각
_locks = {}
_locks_guard = threading.Lock()


_op_locks = {}


def _op_lock(rid):
    """로봇별 동작 잠금 (ON·OFF·스택 전환). GUI 가 여러 탭에서 같은 요청을 동시에 보내면 스택을 두 번 띄우거나,
    한쪽의 '남은 노드 정리'가 다른 쪽이 방금 띄운 차선 노드를 죽인다 (2026-10-09 실물: 두 로봇 차선 노드 exit -9)"""
    with _locks_guard:
        return _op_locks.setdefault(rid, threading.RLock())


def _lock(rid):
    with _locks_guard:
        return _locks.setdefault(rid, threading.Lock())


def connect(rid, r, ip=None, user=None, password=None, quick=False):
    """로봇 실행기를 돌려준다. ssh 는 연결을 재사용한다. quick=True(주기적인 상태 조회)면 실패 직후 재시도하지 않는다."""
    if r.get('mode', 'ssh') == 'local':
        return LocalRunner()
    ip, user = ip or r.get('ip'), user or r.get('user')
    if not ip or not HOST_RE.match(ip):
        raise HTTPException(400, '로봇 IP 가 없거나 형식이 잘못되었습니다')
    if not USER_RE.match(user or ''):
        raise HTTPException(400, 'SSH 사용자 이름 형식이 잘못되었습니다')
    with _lock(rid):
        c = _conns.get(rid)
        if c and c.alive() and c.ip == ip and c.user == user and not password:
            return c
        if quick and rid in _needs_auth:
            raise HTTPException(401, '비밀번호가 필요합니다')
        if quick and time.monotonic() < _retry_at.get(rid, 0):
            raise HTTPException(502, '재접속 대기 중')
        if c:
            c.close()
            _conns.pop(rid, None)
        try:
            c = SSHConn(ip, user, password or _secrets.get(rid), timeout=3 if quick else 6)
        except HTTPException as e:
            if e.status_code == 401:
                _needs_auth.add(rid)
            else:
                _retry_at[rid] = time.monotonic() + 10
            raise
        _needs_auth.discard(rid)
        _retry_at.pop(rid, None)
        _conns[rid] = c
        if password:
            _secrets[rid] = password
        return c


def forget(rid):
    """연결과 메모리의 비밀번호를 지운다."""
    with _lock(rid):
        c = _conns.pop(rid, None)
        if c:
            c.close()
        _secrets.pop(rid, None)
        _needs_auth.discard(rid)
        _retry_at.pop(rid, None)


def paths(rid):
    return f'/tmp/fms_{rid}.pid', f'/tmp/fms_{rid}.log'


def is_running(run, rid):
    pid, _ = paths(rid)
    code, _, _ = run.run(f'test -f {pid} && kill -0 -$(cat {pid}) 2>/dev/null')
    return code == 0


# ---------------- 로봇 FMS 패키지 자동 배포: ON 할 때 관제PC 의 robot/pinky_fms_bringup 과 다르면 복사·빌드 ----------------
FMS_PKG_SRC = Path(__file__).resolve().parents[1].parent / 'robot' / 'pinky_fms_bringup'
FMS_PKG_PARTS = ('CMakeLists.txt', 'package.xml', 'launch', 'params', 'scripts')


def _fms_pkg_files():
    out = {}
    for part in FMS_PKG_PARTS:
        p = FMS_PKG_SRC / part
        for f in ([p] if p.is_file() else sorted(p.rglob('*'))):
            if f.is_file() and '__pycache__' not in f.parts and not f.name.endswith('.pyc'):
                out[str(f.relative_to(FMS_PKG_SRC))] = f.read_bytes()
    return out


def auto_deploy(r, run):
    """robots.yaml 의 auto_deploy(기본 켜짐): 로봇의 pinky_fms_bringup 이 관제PC 것과 다르면 SFTP 로 복사하고 빌드한다.
    같은 내용(해시)이면 아무것도 하지 않는다. 반환: 안내 문구('' = 이미 최신)"""
    if isinstance(run, LocalRunner) or not r.get('auto_deploy', True) or not FMS_PKG_SRC.is_dir():
        return ''
    files = _fms_pkg_files()
    h = hashlib.sha256()
    for k in sorted(files):
        h.update(k.encode()); h.update(b'\0'); h.update(files[k]); h.update(b'\0')
    want = h.hexdigest()[:16]
    ws = r.get('ws_dir', '~/pinky_pro')
    _, home, _ = run.run('echo $HOME')
    ws = ws.replace('~', home.strip(), 1)
    dst = f'{ws}/src/pinky_fms/pinky_fms_bringup'
    marker = f'{dst}/.fms_deploy_hash'
    _, cur, _ = run.run(f'cat {shlex.quote(marker)} 2>/dev/null; test -d {shlex.quote(ws)}/install/pinky_fms_bringup && echo INSTALLED')
    if want in cur and 'INSTALLED' in cur:
        return ''
    code, _, _ = run.run(f'test -d {shlex.quote(ws)}/src')
    if code != 0:
        raise HTTPException(500, f'로봇에 colcon 작업공간({ws}/src)이 없어 FMS 패키지를 자동 배포할 수 없습니다')
    t0 = time.monotonic()
    dirs = sorted({str(Path(k).parent) for k in files if '/' in k})
    run.run(f"pkill -KILL -f '[f]ms_lane_mission.py'; pkill -KILL -f '[r]os2cli.daemon.daemonize'; "
            + ' '.join(f'mkdir -p {shlex.quote(dst + "/" + d)};' for d in [''] + dirs) + ' true')
    try:
        sftp = run.c.open_sftp()
        try:
            for k, data in files.items():
                sftp.putfo(io.BytesIO(data), f'{dst}/{k}')
        finally:
            sftp.close()
    except (paramiko.SSHException, OSError) as e:
        raise HTTPException(502, f'로봇에 FMS 패키지 복사 실패: {type(e).__name__}')
    code, out, err = run.run(f"chmod +x {shlex.quote(dst)}/scripts/*.py; cd {shlex.quote(ws)} && source /opt/ros/jazzy/setup.bash && "
                             f"colcon build --packages-select pinky_fms_bringup 2>&1 | tail -3", timeout=300)
    if code != 0 or 'Summary: 1 package finished' not in out:
        raise HTTPException(500, f'로봇에서 FMS 패키지 빌드 실패: {(out + err).strip()[-300:]}')
    run.run(f'echo {want} > {shlex.quote(marker)}')
    return f'FMS 패키지 자동 배포 완료 ({len(files)}개 파일, {time.monotonic() - t0:.0f} s)'


def start_robot(cfg, rid, r, run, ip):
    """로봇에서 launch 를 시작하고(이미 실행 중이면 그대로), 접속 정보 기억과 ping 추적을 시작한다."""
    note = '이미 실행 중'
    deployed = ''
    if not is_running(run, rid):
        deployed = auto_deploy(r, run)          # bringup 을 띄우기 전에 로봇 패키지를 최신으로
        pid, log = paths(rid)
        launch = r['launch_cmd'].format(namespace=rid)
        dds_export, net = dds_setup(cfg, rid, r, run, ip)
        # setsid 로 새 세션(프로세스 그룹)을 만들어 SSH 가 끊겨도 살아 있게 하고, exec 로 PID == 그룹 ID 가 되게 한다.
        # 관제PC 와 같은 RMW(Cyclone DDS)로 맞춘다. 서로 다른 RMW 는 액션/QoS 에서 불안정할 수 있다.
        inner = (f"{r['ws_setup']} && unset ROS_LOCALHOST_ONLY && export ROS_DOMAIN_ID={cfg['domain_id']} "
                 f'RMW_IMPLEMENTATION={netconf.RMW} && {dds_export}exec {launch}')
        run.run(f'setsid bash -c {shlex.quote(inner)} > {log} 2>&1 < /dev/null & echo $! > {pid}')
        note = '시작 요청 완료' + (' (DDS 유니캐스트)' if net['discovery'] == 'unicast' else '') + (f' · {deployed}' if deployed else '')
    _update('launched.json', lambda d: d.__setitem__(rid, ip))
    _update('last_conn.json', lambda d: d.__setitem__(rid, {'ip': ip, 'user': r.get('user')}))
    start_monitor(rid, ip)
    return {'id': rid, 'running': True, 'note': note}


# ---------------- 주행 스택: bringup 위에 미션에 맞는 것 하나만 (nav = Nav2, lane = 차선 추종) ----------------
def stack_paths(rid, s):
    return f'/tmp/fms_{rid}_{s}.pid', f'/tmp/fms_{rid}_{s}.log'


def stacks_of(r):
    return {k: v for k, v in (r.get('stacks') or {}).items() if STACK_RE.match(str(k)) and v}


def stack_sig(r, rid, s):
    """로봇 터미널에서 손으로 띄운 같은 스택도 알아보기 위한 pgrep 패턴 (launch 파일 + namespace).
    첫 글자를 [x] 로 감싸 이 확인 명령 자신은 걸리지 않게 한다."""
    m = re.search(r'(\S+\.launch\.(?:xml|py))', stacks_of(r)[s])
    if not m:
        return None
    f = m.group(1)
    return f"[{f[0]}]{f[1:]} namespace:={rid}"


def proc_state(run, rid, r):
    """(bringup 실행 중?, 실행 중인 스택 목록) 을 SSH 한 번으로 확인. 백엔드가 띄운 것(pid 파일)과 손으로 띄운 것 모두"""
    pid, _ = paths(rid)
    checks = [f'test -f {pid} && kill -0 -$(cat {pid}) 2>/dev/null && echo bringup']
    for s in stacks_of(r):
        sp, _ = stack_paths(rid, s)
        sig = stack_sig(r, rid, s)
        alt = f' || pgrep -f {shlex.quote(sig)} >/dev/null' if sig else ''
        checks.append(f'{{ test -f {sp} && kill -0 -$(cat {sp}) 2>/dev/null{alt}; }} && echo {s}')
    _, out, _ = run.run('; '.join(f'( {c} )' for c in checks) + '; true')
    names = out.split()
    return 'bringup' in names, [n for n in names if n != 'bringup']


STACK_NODES = {'lane': ('fms_lane_mission.py',), 'nav': ('fms_escape.py', 'fms_camera_stream.py')}   # 정리 안 되고 남으면 안 되는 스택 노드


def orphan_sweep(rid, s):
    """이 로봇의 스택 노드 중 launch 없이 남은 것을 TERM → KILL 하는 셸 명령 ([x] 는 이 명령 자신 제외용)"""
    orphans = ' '.join(shlex.quote(f'[{n[0]}]{n[1:]} .*__ns:=/{rid}( |$)') for n in STACK_NODES.get(s, ()))
    if not orphans:
        return ''
    return (f'for p in {orphans}; do pkill -TERM -f "$p"; done; sleep 2; '
            f'for p in {orphans}; do pkill -KILL -f "$p"; done; ')


def stop_stack(run, rid, s, r=None):
    sp, _ = stack_paths(rid, s)
    sig = stack_sig(r, rid, s) if r else None
    manual = f'pkill -INT -f {shlex.quote(sig)}; ' if sig else ''      # 손으로 띄운 ros2 launch 에도 SIGINT (launch 가 하위 노드를 정리)
    manual_term = f'pkill -TERM -f {shlex.quote(sig)}; ' if sig else ''      # SIGINT 를 무시하는 경우(백그라운드 실행) 대비
    # 노드가 종료 처리(카메라 close 등)에서 멈추면 launch 만 끝나고 노드가 남는다. 남은 노드는 옛 상태(예: ARRIVED)를
    # 계속 발행해 다음 미션을 '이미 도착'으로 만들므로 마지막에 KILL 로 확실히 정리한다 ([x] 는 이 명령 자신 제외용)
    sweep = orphan_sweep(rid, s)
    run.run(f'test -f {sp} && kill -INT -$(cat {sp}) 2>/dev/null; {manual}sleep 3; {manual_term}'
            f'test -f {sp} && kill -0 -$(cat {sp}) 2>/dev/null && kill -TERM -$(cat {sp}) && sleep 3; '
            f'test -f {sp} && kill -KILL -$(cat {sp}) 2>/dev/null; {sweep}rm -f {sp} {sp}.cmd; true', timeout=25)


STACK_ARGS = ('goal_x', 'goal_y')              # 스택 명령에 넣을 수 있는 미션 인자 (숫자만)


def stack_command(r, rid, s, args, map_path=None):
    vals = {'namespace': rid, 'map': r.get('robot_map', ''),
            'lane_map': r.get('lane_map') or r.get('robot_map', '')}     # 차선 미션용 지도 (없으면 robot_map)
    if map_path:                    # GUI 에서 고른 지도 (로봇에 복사한 경로): 어느 스택이든 이 지도를 쓴다
        vals['map'] = vals['lane_map'] = map_path
    for k, v in (args or {}).items():
        if k not in STACK_ARGS:
            raise HTTPException(400, f'허용되지 않은 미션 인자: {k}')
        try:
            f = float(v)
        except (TypeError, ValueError):
            raise HTTPException(400, f'{k} 는 숫자여야 합니다')
        if f != f or abs(f) > 1000:
            raise HTTPException(400, f'{k} 값이 범위를 벗어났습니다')
        vals[k] = f'{f:.3f}'
    try:
        return stacks_of(r)[s].format(**vals)
    except KeyError as e:
        raise HTTPException(400, f'{s} 스택에는 미션 인자 {e} 가 필요합니다 (예: 목적지)')


def start_stack(cfg, rid, r, run, ip, s, args=None, map_path=None):
    sp, log = stack_paths(rid, s)
    launch = stack_command(r, rid, s, args, map_path)
    dds_export, _ = dds_setup(cfg, rid, r, run, ip)
    inner = (f"{r['ws_setup']} && unset ROS_LOCALHOST_ONLY && export ROS_DOMAIN_ID={cfg['domain_id']} "
             f'RMW_IMPLEMENTATION={netconf.RMW} && {dds_export}exec {launch}')
    # 이전 실행에서 남은 노드가 있으면 먼저 정리 (같은 토픽에 옛 상태를 계속 발행하고 카메라를 잡고 있다)
    run.run(f'{orphan_sweep(rid, s)}echo {shlex.quote(launch)} > {sp}.cmd; '
            f'setsid bash -c {shlex.quote(inner)} > {log} 2>&1 < /dev/null & echo $! > {sp}', timeout=15)


def check_robot(run, r):
    """추가하기 전에 로봇에서 launch 패키지를 찾을 수 있는지 확인한다 (ws_setup 경로, 빌드 여부)."""
    m = PKG_RE.search(r['launch_cmd'])
    if not m:
        return
    pkg = m.group(1)
    code, _, err = run.run(f"{r['ws_setup']} && ros2 pkg prefix {shlex.quote(pkg)}", timeout=40)
    if code != 0:
        raise HTTPException(400, f'로봇에서 {pkg} 패키지를 찾지 못했습니다. 로봇에 빌드했는지, '
                                 f'robots.yaml 의 ws_setup 경로가 맞는지 확인하세요. ({err.strip()[-120:]})')
    code, _, _ = run.run(f"{r['ws_setup']} && ros2 pkg prefix {netconf.RMW}", timeout=40)
    if code != 0:
        raise HTTPException(400, f'로봇에 {netconf.RMW} 가 설치돼 있지 않습니다. 로봇에서 '
                                 f'sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp 를 실행하세요 '
                                 f'(관제PC 와 같은 RMW 를 써야 통신이 안정적입니다).')


# ---------------- API ----------------
class AddBody(BaseModel):
    id: str                        # = ROS namespace (예: amr_01)
    ip: str | None = None
    user: str | None = None
    password: str | None = None    # 파일에 저장하지 않는다
    install_key: bool = False      # 다음부터 비밀번호 없이 접속 (이 PC 의 SSH 키를 로봇에 등록)
    start: bool = True             # 추가하자마자 ON


class OnBody(BaseModel):
    ip: str | None = None
    user: str | None = None
    password: str | None = None
    install_key: bool = False


@app.get('/network')
def network_info():
    n = netconf.network_cfg(load())
    return {'discovery': n['discovery'], 'interface': n['interface'], 'rmw': netconf.RMW if n['discovery'] == 'unicast' else None}


@app.get('/config')
def config_info():
    d = defaults(load())
    return {'defaults': {'user': d['user'], 'mode': d['mode']}, 'key_installed_on_pc': KEY_PATH.exists(),
            'stacks': sorted(stacks_of(d))}


@app.get('/robots')
def list_robots():
    _, reg = registry()
    launched = _read('launched.json', {})
    return [{'id': rid, 'namespace': rid, 'ip': r.get('ip'), 'user': r.get('user'), 'mode': r.get('mode', 'ssh'),
             'preset': r['preset'], 'launched': rid in launched} for rid, r in reg.items()]


@app.post('/robots')
def add_robot(body: AddBody):
    cfg, reg = registry()
    rid = body.id.strip()
    if not NS_RE.match(rid):
        raise HTTPException(400, 'ID(namespace)는 영문 소문자로 시작하고 소문자·숫자·_ 만 쓸 수 있습니다 (예: amr_01)')
    if rid in reg:
        raise HTTPException(409, f'이미 있는 ID 입니다: {rid}')
    base = defaults(cfg)
    r = {**base, 'id': rid, 'ip': (body.ip or '').strip() or ('127.0.0.1' if base['mode'] == 'local' else None),
         'user': (body.user or '').strip() or base['user']}
    for k in ('ws_setup', 'launch_cmd'):
        if not r.get(k):
            raise HTTPException(500, f'robots.yaml 의 defaults 에 {k} 가 없습니다')
    run = connect(rid, r, r['ip'], r['user'], body.password)
    try:
        check_robot(run, r)
        if isinstance(run, SSHConn) and body.password and body.install_key:
            run.install_key()
    except HTTPException:
        forget(rid)
        raise
    _update('added.json', lambda d: d.__setitem__(rid, {'ip': r['ip'], 'user': r['user']}))
    _update('last_conn.json', lambda d: d.__setitem__(rid, {'ip': r['ip'], 'user': r['user']}))
    if body.start:
        return {**start_robot(cfg, rid, r, run, r['ip']), 'added': True}
    return {'id': rid, 'added': True, 'running': False}


@app.delete('/robots/{rid}')
def delete_robot(rid: str, force: bool = False):
    _, reg = registry()
    r = reg.get(rid)
    if r is None:
        raise HTTPException(404, f'등록되지 않은 로봇: {rid}')
    if r['preset']:
        raise HTTPException(400, 'robots.yaml 에 적힌 로봇은 GUI 에서 지울 수 없습니다 (파일에서 지우세요)')
    if rid in _read('launched.json', {}) and not force:
        raise HTTPException(409, '실행 중인 로봇입니다. 먼저 OFF 하세요')
    for name in ('added.json', 'last_conn.json', 'launched.json'):
        _update(name, lambda d: d.pop(rid, None))
    stop_monitor(rid)
    forget(rid)
    return {'id': rid, 'deleted': True}


@app.get('/robots/{rid}/status')
def status(rid: str):
    """2초마다 불린다. 우리가 켜지 않은 ssh 로봇은 SSH 로 확인하지 않는다 (불필요한 접속을 만들지 않기 위해)."""
    _, r = get_robot(rid)
    local = r.get('mode', 'ssh') == 'local'
    launched = _read('launched.json', {})
    if not local and rid not in launched:
        return {'id': rid, 'running': False, 'reachable': None, 'auth_required': False}
    try:
        running, stacks = proc_state(connect(rid, r, quick=True), rid, r)
    except HTTPException as e:
        # 확인할 수 없으면 마지막으로 아는 상태(실행 중)를 유지하고 이유를 알린다
        return {'id': rid, 'running': True, 'reachable': e.status_code == 401, 'auth_required': e.status_code == 401}
    if running and rid not in monitors:
        start_monitor(rid, launched.get(rid) or r.get('ip') or '127.0.0.1')
    if not running and rid in launched:          # 로봇이 재부팅됐거나 launch 가 스스로 종료됨
        _update('launched.json', lambda d: d.pop(rid, None))
        stop_monitor(rid)
    return {'id': rid, 'running': running, 'reachable': True, 'auth_required': False, 'stack': stacks[0] if stacks else None}


@app.post('/robots/{rid}/on')
def turn_on(rid: str, body: OnBody):
    """켜기. 이미 켜져 있으면 다시 접속만 한다 (비밀번호가 필요해진 경우의 재접속에도 쓴다)."""
    cfg, r = get_robot(rid)
    local = r.get('mode', 'ssh') == 'local'
    ip = (body.ip or '').strip() or r.get('ip') or ('127.0.0.1' if local else None)
    user = (body.user or '').strip() or r.get('user')
    run = connect(rid, r, ip, user, body.password)
    if isinstance(run, SSHConn) and body.password and body.install_key:
        run.install_key()
    r = {**r, 'ip': ip, 'user': user}
    if not r['preset']:
        _update('added.json', lambda d: d.__setitem__(rid, {'ip': ip, 'user': user}))
    with _op_lock(rid):
        return start_robot(cfg, rid, r, run, ip)


@app.post('/robots/{rid}/off')
def turn_off(rid: str):
    _, r = get_robot(rid)
    run = connect(rid, r)          # 401 이면 GUI 가 비밀번호를 다시 받아 재접속한다
    with _op_lock(rid):
        for s in proc_state(run, rid, r)[1]:      # 주행 스택(Nav2·차선 추종)을 먼저 끈다
            stop_stack(run, rid, s, r)
    pid, _ = paths(rid)
    # ros2 launch 가 정리할 수 있도록 SIGINT 를 프로세스 그룹에 보낸다
    run.run(f'test -f {pid} && kill -INT -$(cat {pid}) 2>/dev/null; sleep 3; '
            f'kill -0 -$(cat {pid}) 2>/dev/null && kill -TERM -$(cat {pid}); rm -f {pid}', timeout=15)
    _update('launched.json', lambda d: d.pop(rid, None))
    stop_monitor(rid)
    forget(rid)                    # 비밀번호와 연결을 지운다: 다음 ON 때 다시 입력
    return {'id': rid, 'running': False}


@app.get('/robots/{rid}/ping')
def ping_stats(rid: str):
    get_robot(rid)
    m = monitors.get(rid)
    if m is None:
        return {'id': rid, 'monitoring': False}
    samples = list(m.samples)
    ok = sorted(x for x in samples if x is not None)
    return {
        'id': rid, 'monitoring': True, 'ip': m.ip, 'samples': samples,
        'last_rtt': samples[-1] if samples else None,
        'avg_rtt': sum(ok) / len(ok) if ok else None,
        'p95_rtt': ok[min(len(ok) - 1, int(len(ok) * 0.95))] if ok else None,
        'loss_pct': 100.0 * (len(samples) - len(ok)) / len(samples) if samples else None,
    }


class StackBody(BaseModel):
    stack: str = ''                # '' 또는 'none' = 주행 스택 모두 끄기
    args: dict | None = None       # 미션 인자 (예: lane 의 goal_x, goal_y). 주어지면 같은 스택이 실행 중이어도 다시 띄운다
    map: str | None = None         # GUI 에서 고른 지도 이름 (maps/<name>). 로봇에 복사해 그 지도로 띄운다


@app.post('/robots/{rid}/stack')
def set_stack(rid: str, body: StackBody):
    """미션에 맞는 주행 스택을 켠다 (다른 스택은 끈다). 실행할 명령은 robots.yaml 에 정해진 것만 쓴다."""
    cfg, r = get_robot(rid)
    want = (body.stack or '').strip()
    want = '' if want == 'none' else want
    if want and want not in stacks_of(r):
        raise HTTPException(400, f'robots.yaml 의 stacks 에 없는 주행 스택: {want}')
    if body.map:
        map_meta(body.map)                           # 없는 지도면 404
    if want:
        stack_command(r, rid, want, body.args)       # 인자를 먼저 검사한다 (잘못되면 아무것도 끄지 않음)
    launched = _read('launched.json', {})
    ip = launched.get(rid) or r.get('ip') or '127.0.0.1'
    run = connect(rid, r)
    with _op_lock(rid):          # 같은 로봇의 스택 요청은 한 번에 하나씩 (여러 GUI 탭 대비)
        bringup, running = proc_state(run, rid, r)
        if want and not bringup:
            raise HTTPException(409, '로봇이 꺼져 있습니다. 먼저 ON 하세요')
        map_path = push_map(run, body.map) if want and body.map else None
        restart = bool(body.args) and want in running    # 미션 인자(목적지)가 바뀌면 새로 띄운다
        if want in running and not restart:
            # 백엔드가 띄운 스택이 다른 지도(명령)로 떠 있으면 새로 띄운다. 손으로 띄운 것(.cmd 없음)은 그대로 둔다
            sp, _ = stack_paths(rid, want)
            _, cur, _ = run.run(f'cat {sp}.cmd 2>/dev/null')
            restart = bool(cur.strip()) and cur.strip() != stack_command(r, rid, want, body.args, map_path)
        for s in running:
            if s != want or restart:
                stop_stack(run, rid, s, r)
        note, started = ('이미 실행 중' if want in running and not restart else ''), False
        if want and (want not in running or restart):
            start_stack(cfg, rid, r, run, ip, want, body.args, map_path)
            note, started = '시작 요청 완료', True
        return {'id': rid, 'stack': want or None, 'note': note, 'started': started, 'map': map_path}


def push_map(run, name):
    """GUI 지도(maps/<name>)를 로봇의 ~/fms_maps/<name>/ 에 복사하고 map.yaml 경로를 돌려준다. 같으면(md5) 건너뛴다."""
    m = map_meta(name)
    d = MAPS_DIR / name
    if isinstance(run, LocalRunner):
        return str(d / 'map.yaml')
    files = {f: (d / f).read_bytes() for f in ('map.yaml', m['image'])}
    _, home, _ = run.run('echo $HOME')
    rdir = f'{home.strip()}/fms_maps/{name}'
    _, out, _ = run.run(f'mkdir -p {shlex.quote(rdir)} && cd {shlex.quote(rdir)} && md5sum {" ".join(map(shlex.quote, files))} 2>/dev/null')
    have = {ln.split()[1]: ln.split()[0] for ln in out.splitlines() if len(ln.split()) == 2}
    todo = [f for f, b in files.items() if have.get(f) != hashlib.md5(b).hexdigest()]
    if todo:
        try:
            sftp = run.c.open_sftp()
            try:
                for f in todo:
                    sftp.putfo(io.BytesIO(files[f]), f'{rdir}/{f}')
            finally:
                sftp.close()
        except (paramiko.SSHException, OSError) as e:
            raise HTTPException(502, f'로봇에 지도 복사 실패: {type(e).__name__}')
    return f'{rdir}/map.yaml'


@app.get('/robots/{rid}/logs')
def logs(rid: str, lines: int = 200, stack: str = ''):
    _, r = get_robot(rid)
    run = connect(rid, r)
    if stack and stack not in stacks_of(r):
        raise HTTPException(400, f'알 수 없는 주행 스택: {stack}')
    _, log = stack_paths(rid, stack) if stack else paths(rid)
    _, out, _ = run.run(f'tail -n {max(1, min(lines, 2000))} {log} 2>/dev/null')
    return {'id': rid, 'log': out}


# ---------------- 맵 (업로드 / 목록 / 제공) ----------------
# maps/<이름>/map.yaml + 이미지(pgm/png). 형식은 nav2 map_server 의 map.yaml 과 같다.
MAPS_DIR = Path(os.environ.get('FMS_MAPS_DIR', Path(__file__).resolve().parent / 'maps'))
MAX_MAP_BYTES = 10 * 1024 * 1024
IMG_EXT = {'.pgm': 'image/x-portable-graymap', '.png': 'image/png'}


def map_meta(name):
    if not ID_RE.match(name) or not (MAPS_DIR / name / 'map.yaml').exists():
        raise HTTPException(404, f'맵 없음: {name}')
    d = MAPS_DIR / name
    y = yaml.safe_load((d / 'map.yaml').read_text(encoding='utf-8'))
    img = Path(str(y.get('image', ''))).name
    if Path(img).suffix.lower() not in IMG_EXT or not (d / img).exists():
        raise HTTPException(500, f'맵 이미지 파일이 없음: {img}')
    return {
        'name': name, 'image': img,
        'resolution': float(y['resolution']), 'origin': [float(v) for v in y['origin'][:3]],
        'negate': int(y.get('negate', 0)),
        'occupied_thresh': float(y.get('occupied_thresh', 0.65)), 'free_thresh': float(y.get('free_thresh', 0.196)),
    }


@app.get('/maps')
def list_maps():
    out = []
    for d in sorted(MAPS_DIR.glob('*/map.yaml')):
        try:
            out.append(map_meta(d.parent.name))
        except HTTPException:
            pass
    return out


@app.get('/maps/{name}')
def get_map(name: str):
    return map_meta(name)


@app.get('/maps/{name}/image')
def get_map_image(name: str):
    m = map_meta(name)
    f = MAPS_DIR / name / m['image']
    return FileResponse(f, media_type=IMG_EXT[f.suffix.lower()])


@app.post('/maps')
async def upload_map(yaml_file: UploadFile = File(...), image_file: UploadFile = File(...)):
    """map.yaml 과 이미지(pgm/png)를 함께 올린다. 맵 이름은 yaml 파일 이름(확장자 제외)."""
    name = Path(yaml_file.filename or '').stem
    if name == 'map':
        name = Path(image_file.filename or '').stem
    if not ID_RE.match(name):
        raise HTTPException(400, '맵 이름은 영문/숫자/_ 만 가능 (yaml 파일 이름에서 가져옴)')
    ydata, idata = await yaml_file.read(), await image_file.read()
    if len(ydata) > 64 * 1024 or len(idata) > MAX_MAP_BYTES:
        raise HTTPException(413, '파일이 너무 큼')
    try:
        y = yaml.safe_load(ydata)
        res, origin = float(y['resolution']), [float(v) for v in y['origin'][:3]]
    except Exception:
        raise HTTPException(400, 'map.yaml 형식 오류 (resolution, origin 필요)')
    if res <= 0 or len(origin) != 3:
        raise HTTPException(400, 'resolution/origin 값 오류')
    ext = Path(image_file.filename or '').suffix.lower()
    if ext not in IMG_EXT:
        raise HTTPException(400, '이미지는 .pgm 또는 .png 만 지원')
    magic_ok = idata[:2] == b'P5' if ext == '.pgm' else idata[:8] == b'\x89PNG\r\n\x1a\n'
    if not magic_ok:
        raise HTTPException(400, '이미지 파일 내용이 확장자와 다름')
    d = MAPS_DIR / name
    d.mkdir(parents=True, exist_ok=True)
    img_name = f'{name}{ext}'
    y['image'] = img_name
    (d / img_name).write_bytes(idata)
    (d / 'map.yaml').write_text(yaml.safe_dump(y, allow_unicode=True), encoding='utf-8')
    return map_meta(name)
