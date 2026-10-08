"""차선 주행(Vision-based Lane Following) 로봇끼리의 마주침 관제 (fleet_traffic 과 별개 노드).

fleet_traffic 은 차선 미션 중인 로봇을 관제 대상에서 뺀다(lane_active). 이 노드는 그 차선 로봇들만 본다.
  1) 1차선에서 두 로봇이 서로를 향해 meet_dist 안에 들어오면 (또는 둘 다 '앞 사물'로 멈춰 교착이면)
  2) 둘 다 일시정지(hold)
  3) 비켜서기 쉬운 쪽이 양보: 차선 밖 비켜설 자리로 이동(yield) → 도착(YIELDED)하면 우선권 로봇 재개(resume priority)
  4) 우선권 로봇이 복귀 자리를 지나가면 양보 로봇 재개(resume yielder): 떠난 자리·방향으로 돌아와 차선 추종 재개

  갈림길 선착순(사용자 승인 2026-10-08): 갈림길(차선 지도에서 자동 추출)에 먼저 정지한 로봇(detail 'junction…')이 우선.
  나중에 갈림길 junction_radius 안으로 다가오는 로봇은 hold, 우선 로봇이 정지 자리에서 junction_exit 이상 빠져나가면 resume.
  우선 로봇이 'junction busy' 로 대기 로봇에 막혀 있으면(대기 로봇이 우선 로봇의 출구 쪽) 대기 로봇을 차선 밖으로 비켜서게 한다.
  서 있는 로봇(IDLE/ARRIVED, 차선 위)이 주행 로봇 앞을 2 s 넘게 막으면 서 있는 로봇을 비켜서게 했다가 제자리로 돌린다
  (로봇 노드는 미션 없이도 yield/resume 을 따른다, pinky-96 2026-10-08).
  알림: 미션 중 차선 밖에 멈춘 로봇, LOST 로 10 s 넘게 멈춘 로봇 → /fleet/lane_traffic_state 의 alerts.
로봇 쪽 실행은 fms_lane_mission(pinky_fms_bringup)이 한다.
  명령 /<ns>/lane_traffic_cmd (String JSON): {"id","cmd":"hold"} | {"id","cmd":"yield","x","y"} | {"id","cmd":"resume","role"}
    진행 중인 명령은 keepalive 로 1 s 마다 같은 id 로 다시 보낸다 (로봇은 10 s 동안 못 받으면 resume 으로 간주).
  상태 /<ns>/lane_status 의 pose, traffic(HOLD|YIELDING|YIELDED|REJOINING|YIELD_FAILED|null), traffic_id
GUI/기록용 요약: /fleet/lane_traffic_state (String JSON, 2 Hz). GLOBAL E-STOP(/fleet/global_cmd) 이면 진행 중 조정을 모두 버린다
(로봇 차선 노드는 E-STOP 을 직접 받아 멈춘다).
"""
import json
import math
import re
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from .lane_geom import LaneGeometry, ang_diff, choose_yielder, headon, passed, path_blocked, rejoin_point, retreat_then_escape

LANE_STATUS_RE = re.compile(r'^/([^/]+)/lane_status$')
INACTIVE = ('ARRIVED', 'FAILED', 'IDLE')


class LaneRobot:
    def __init__(self, rid):
        self.id = rid
        self.st = None              # 마지막 lane_status (dict)
        self.t = 0.0                # 받은 시각
        self.pub = None
        self.cmd = None             # 지금 보내고 있는 명령 dict (keepalive)
        self.cmd_t = 0.0
        self.jn_since = None        # 갈림길 정지(detail 'junction…')를 처음 본 시각
        self.still_pose = None      # 마지막으로 움직였다고 본 위치와 그 시각 (멈춤 알림·막힘 판단)
        self.still_t = 0.0
        self.block_t = None         # 서 있는 로봇에 막혀 'obstacle ahead' 로 멈추기 시작한 시각

    def still_for(self, now):
        return now - self.still_t if self.still_pose is not None else 0.0

    def fresh(self, now, stale=1.5):
        return self.st is not None and now - self.t < stale

    @property
    def pose(self):
        p = self.st.get('pose') if self.st else None
        return tuple(p) if p and len(p) >= 3 else None

    @property
    def traffic(self):
        return self.st.get('traffic') if self.st else None

    def acked(self):
        return self.cmd is not None and self.st is not None and self.st.get('traffic_id') == self.cmd['id']

    def active(self, now):
        """차선 미션 수행 중 (목적지가 있고 위치를 안다)"""
        return self.fresh(now) and self.st.get('state') not in INACTIVE and self.pose is not None


class Encounter:
    def __init__(self, a, b, why):
        self.a, self.b = a, b       # LaneRobot
        self.phase = 'HOLD'         # HOLD → YIELD → PASS → REJOIN → (끝)
        self.t0 = time.monotonic()
        self.why = why
        self.yielder = self.priority = None
        self.escape = self.rejoin = None
        self.failed = set()         # 비켜서기에 실패한 로봇 id
        self.force_priority = None  # 갈림길 우선 로봇처럼 우선권이 정해진 경우
        self.parked = None          # 서 있는 로봇(미션 없음)이 길을 막는 경우 그 로봇 — 미션이 없어도 '끝남'으로 보지 않는다
        self.pending_escape = None  # 뒤로 물러난 다음 이어서 비켜설 자리 (바로 비켜설 자리가 없을 때)
        self.rejoin_tries = 0       # 복귀가 막혀 복귀 자리를 다시 고른 횟수
        self.prio_path = None       # 우선권 로봇이 지나갈 중심선 점·진행 방향 (양보 로봇이 통로를 벗어났는지 판단)
        self.blocked_t = None       # PASS: 우선권 로봇이 양보 로봇 때문에 멈춰 있기 시작한 시각
        self.reyields = 0           # 우선권 로봇을 막아 다시 비켜선 횟수
        self.info = {}

    def other(self, r):
        return self.b if r is self.a else self.a

    def summary(self):
        return {'robots': [self.a.id, self.b.id], 'phase': self.phase, 'why': self.why,
                'priority': self.priority.id if self.priority else None,
                'yielder': self.yielder.id if self.yielder else None,
                'escape': None if self.escape is None else [round(v, 2) for v in self.escape],
                'rejoin': None if self.rejoin is None else [round(v, 2) for v in self.rejoin[:2]],
                'age': round(time.monotonic() - self.t0, 1), 'info': self.info}


class LaneTraffic(Node):
    def __init__(self):
        super().__init__('lane_traffic')
        self.declare_parameter('lanes_yaml', '')        # 차선만 빈 지도 (mission4_3_lanes_1cm)
        self.declare_parameter('floor_yaml', '')        # 차선 지운 지도 (mission4_3_nolanes_1cm)
        self.declare_parameter('meet_dist', 0.6)        # 마주 보고 이 거리 안이면 정지·조정 (중심 간)
        self.declare_parameter('deadlock_dist', 0.45)   # 마주 보는 판단이 안 돼도 둘 다 '앞 사물'로 멈춰 있고 이 안이면 교착으로 본다
        self.declare_parameter('escape_max', 0.45)      # 비켜설 자리 최대 거리 (없으면 0.7 m 까지 넓혀 다시 찾는다)
        self.declare_parameter('pass_dist', 0.45)       # 우선권 로봇이 복귀 자리에서 이만큼 멀어지고 등 뒤가 되면 양보 로봇 재개
        self.declare_parameter('hold_wait', 1.5)        # hold 확인을 기다리는 최대 시간 (s)
        self.declare_parameter('yield_timeout', 25.0)
        self.declare_parameter('pass_timeout', 90.0)    # 우선권 로봇이 이 시간 안에 못 지나가면 그래도 양보 로봇을 재개
        self.declare_parameter('rejoin_timeout', 30.0)
        self.declare_parameter('cooldown', 5.0)         # 같은 두 로봇을 다시 조정하기 전 대기 (s)
        self.declare_parameter('junction_radius', 0.6)  # 갈림길 중심에서 이 안으로 다가오는 비우선 로봇은 hold
        self.declare_parameter('junction_at', 0.35)     # 갈림길에 '정지한' 로봇으로 볼 중심 거리
        self.declare_parameter('junction_exit', 0.4)    # 우선 로봇이 정지 자리에서 이만큼 빠져나가면 대기 로봇 resume
        self.declare_parameter('junction_exit_radius', 0.75)  # 우선 로봇의 출구 차선 위 로봇은 이 안이면 hold 대신 바로 양보 (2026-10-08 23:44 실물 교착)
        self.declare_parameter('junction_busy_wait', 2.0)  # 우선 로봇이 대기 로봇 때문에 'junction busy' 로 이 시간 넘게 서 있으면 대기 로봇 양보
        gp = lambda n: self.get_parameter(n).value
        self.meet_dist, self.deadlock_dist = gp('meet_dist'), gp('deadlock_dist')
        self.escape_max, self.pass_dist = gp('escape_max'), gp('pass_dist')
        self.hold_wait, self.yield_timeout = gp('hold_wait'), gp('yield_timeout')
        self.pass_timeout, self.rejoin_timeout, self.cooldown = gp('pass_timeout'), gp('rejoin_timeout'), gp('cooldown')
        self.jn_radius, self.jn_at, self.jn_exit, self.jn_busy_wait = (gp('junction_radius'), gp('junction_at'),
                                                                      gp('junction_exit'), gp('junction_busy_wait'))
        self.jn_exit_radius = gp('junction_exit_radius')
        self.geo = None
        if gp('lanes_yaml') and gp('floor_yaml'):
            try:
                self.geo = LaneGeometry(gp('lanes_yaml'), gp('floor_yaml'))
            except Exception as e:      # 지도가 없거나 형식이 다르면 조정 없이 상태만 발행
                self.get_logger().error(f'지도 읽기 실패: {e}')
        if self.geo is None:
            self.get_logger().warn('lanes_yaml/floor_yaml 이 없어 차선 마주침 조정 꺼짐')
        # 갈림길: {'c': (x, y), 'owner': LaneRobot|None, 'owner_pose', 'held': set(LaneRobot), 'busy_t'}
        self.junctions = [] if self.geo is None else [
            {'c': c, 'owner': None, 'owner_pose': None, 'held': set(), 'busy_t': None, 'exit': None} for c in self.geo.find_junctions()]
        if self.geo is not None:
            self.get_logger().info(f'갈림길 {len(self.junctions)}곳: ' + ', '.join(f'({j["c"][0]:.2f}, {j["c"][1]:.2f})' for j in self.junctions))
        self.robots = {}
        self.encounters = []
        self.cool = {}              # frozenset(id 쌍) → 다시 조정할 수 있는 시각
        self.seq = int(time.time() * 1000) % 1_000_000_000
        self.state_pub = self.create_publisher(String, '/fleet/lane_traffic_state', 10)
        self.create_subscription(String, '/fleet/global_cmd', self._on_global_cmd, 10)
        self.create_timer(2.0, self._discover)
        self.create_timer(0.1, self._tick)
        self.create_timer(0.5, self._publish_state)
        self._discover()
        self.get_logger().info(f'lane_traffic ready (조정 {"켜짐" if self.geo else "꺼짐"})')

    # ---------- 로봇 발견·상태 ----------
    def _discover(self):
        for name, _ in self.get_topic_names_and_types():
            m = LANE_STATUS_RE.match(name)
            if m and m.group(1) not in self.robots:
                rid = m.group(1)
                r = self.robots[rid] = LaneRobot(rid)
                r.pub = self.create_publisher(String, f'/{rid}/lane_traffic_cmd', 10)
                self.create_subscription(String, name, lambda msg, r=r: self._on_status(r, msg), 10)
                self.get_logger().info(f'차선 로봇 발견: /{rid}')

    def _on_status(self, r, msg):
        try:
            r.st, r.t = json.loads(msg.data), time.monotonic()
        except ValueError:
            pass

    # ---------- 명령 ----------
    def _send(self, r, cmd, **kw):
        self.seq += 1
        r.cmd = dict(id=self.seq, cmd=cmd, **kw)
        r.cmd_t = 0.0                   # 바로 보낸다
        self.get_logger().info(f'→ {r.id}: {cmd} {kw if kw else ""}')

    def _pump(self, now):
        """진행 중인 명령을 1 s 마다 같은 id 로 재전송 (처음 확인 전에는 0.3 s 마다)"""
        for r in self.robots.values():
            if r.cmd is None:
                continue
            period = 1.0 if r.acked() else 0.3
            if now - r.cmd_t >= period:
                r.pub.publish(String(data=json.dumps(r.cmd)))
                r.cmd_t = now

    def _release(self, r, role):
        self._send(r, 'resume', role=role)

    def _release_yielder(self, e, skip=0.0):
        """양보 로봇 복귀. 원래 자리가 다른 로봇(도착해 서 있는 우선권 로봇 등)에 막혀 있으면 진행 방향 쪽 빈 자리로"""
        y = e.yielder
        blockers = [o.pose for o in self.robots.values() if o is not y and o.fresh(time.monotonic(), 3.0)]
        # 미션 없이 서 있던 로봇은 원래 자리가 제자리다: 실제로 부딪힐 만큼(0.2 m) 가까울 때만 다른 자리로
        clear = 0.2 if e.parked is y else 0.35
        rj = rejoin_point(self.geo, e.rejoin, y.pose[:2] if y.pose else e.rejoin[:2], blockers, clear=clear, skip=skip)
        if rj is None:
            self.get_logger().warn(f'{y.id}: 비어 있는 복귀 자리를 찾지 못함 → 원래 자리로 복귀 시도')
            return self._send(y, 'resume', role='yielder')
        if skip <= 0.0 and tuple(rj) == tuple(e.rejoin[:3]):
            return self._send(y, 'resume', role='yielder')
        self.get_logger().info(f'{y.id}: 원래 자리가 막혀 복귀 자리 변경 → ({rj[0]:.2f}, {rj[1]:.2f}, {math.degrees(rj[2]):.0f}°)')
        self._send(y, 'resume', role='yielder', x=round(rj[0], 3), y=round(rj[1], 3), yaw=round(rj[2], 3))

    def _finish(self, e, msg):
        self.get_logger().info(f'조정 끝 {e.a.id}↔{e.b.id}: {msg}')
        self.encounters.remove(e)
        self.cool[frozenset((e.a.id, e.b.id))] = time.monotonic() + self.cooldown
        for r in (e.a, e.b):            # resume 이 확인될 때까지만 더 보낸다
            if r.cmd is not None and r.cmd['cmd'] != 'resume':
                r.cmd = None

    # ---------- 주기 판단 ----------
    def _tick(self):
        now = time.monotonic()
        for r in self.robots.values():           # 움직임 추적 (2 cm 이상 움직이면 갱신)
            p = r.pose if r.fresh(now) else None
            if p is None:
                r.still_pose = None
            elif r.still_pose is None or math.hypot(p[0] - r.still_pose[0], p[1] - r.still_pose[1]) > 0.02:
                r.still_pose, r.still_t = p, now
        if self.geo is not None:
            self._junction_tick(now)
            self._detect(now)
            self._detect_parked(now)
            for e in list(self.encounters):
                self._step(e, now)
        for r in self.robots.values():       # resume 이 확인됐거나 로봇이 사라지면 keepalive 중단
            if r.cmd is not None and r.cmd['cmd'] == 'resume' and not any(r in (e.a, e.b) for e in self.encounters):
                if r.acked() or not r.fresh(now, 5.0):
                    r.cmd = None
        self._pump(now)

    def _busy(self, r):
        return any(r in (e.a, e.b) for e in self.encounters)

    # ---------- 갈림길 선착순 ----------
    def _jn_dist(self, r, j):
        return math.hypot(r.pose[0] - j['c'][0], r.pose[1] - j['c'][1])

    def _jn_eligible(self, r, j):
        """갈림길 역할(우선·대기·양보)을 줄 수 있는 로봇: 차선 위에 있고 갈림길 중심과 사이에 지도 벽이 없다.
        차선 밖에 멈춘 로봇(차선 이탈)이나 벽 너머 가까이 있는 로봇은 갈림길을 쓰는 로봇이 아니다 (rec_20261008_220315)."""
        return self.geo.on_lane(r.pose) and not self.geo.wall_between(r.pose, j['c'])

    def _junction_tick(self, now):
        for r in self.robots.values():           # 갈림길 정지를 처음 본 시각 (선착순 기준)
            at_jn = r.active(now) and str(r.st.get('detail', '')).startswith('junction')
            r.jn_since = (r.jn_since or now) if at_jn else None
        for j in self.junctions:
            o = j['owner']
            # 우선 로봇 해제: 정지 자리에서 빠져나갔거나 미션이 끝남
            if o is not None:
                left = (o.pose is not None and not str(o.st.get('detail', '')).startswith('junction')
                        and math.hypot(o.pose[0] - j['owner_pose'][0], o.pose[1] - j['owner_pose'][1]) >= self.jn_exit)
                if left or not o.active(now):
                    self.get_logger().info(f'🚥 갈림길 ({j["c"][0]:.2f},{j["c"][1]:.2f}) {o.id} 통과 → 대기 로봇 출발 {[r.id for r in j["held"]]}')
                    for r in j['held']:
                        if self._busy(r):
                            continue
                        # 우선 로봇이 대기 로봇 쪽으로 나가는 중이면(같은 띠에서 정면) 풀어 주면 바로 마주친다 (실물 10-09 00:12)
                        if o.active(now) and self._in_front(o, r, ahead=1.0, half=0.25) and self._jn_eligible(r, j):
                            self._junction_yield(j, o, r, 'owner heading to held robot')
                        else:
                            self._release(r, 'junction')
                    j.update(owner=None, owner_pose=None, held=set(), busy_t=None, exit=None)
                    o = None
            # 우선 로봇 정하기: 갈림길에 정지한 로봇 중 먼저 선 쪽
            if o is None:
                at = [r for r in self.robots.values() if r.jn_since is not None and r.traffic is None
                      and not self._busy(r) and self._jn_dist(r, j) <= self.jn_at and self._jn_eligible(r, j)]
                if at:
                    o = min(at, key=lambda r: r.jn_since)
                    j.update(owner=o, owner_pose=o.pose, busy_t=None, exit=None)
                    self.get_logger().info(f'🚥 갈림길 ({j["c"][0]:.2f},{j["c"][1]:.2f}) 우선: {o.id} (먼저 정지)')
            if o is None:
                continue
            # 우선 로봇이 고른 출구 → 출구 차선 방향. lane_status 'exit' 필드(2026-10-08 추가)를 먼저 쓰고, 없으면(배포 전 노드)
            # detail 'junction check (left)' 등에서 읽는다. 한 번 보면 우선권이 풀릴 때까지 기억한다 (출구 확인을 통과하면
            # detail 에서 출구가 사라진다). 방향 기준은 갈림길에 정지했을 때의 방향(owner_pose) — 회전 중에 읽어도 틀어지지 않게.
            if j['exit'] is None and j['owner_pose'] is not None:
                ex = o.st.get('exit')
                if ex not in ('left', 'right', 'straight', 'back'):
                    m = re.search(r'\((left|right|straight|back)\)', str(o.st.get('detail', '')))
                    ex = m.group(1) if m and str(o.st.get('detail', '')).startswith('junction') else None
                if ex:
                    off = {'left': math.pi / 2, 'right': -math.pi / 2, 'straight': 0.0, 'back': math.pi}[ex]
                    j['exit'] = (ex, j['owner_pose'][2] + off)
                    self.get_logger().info(f'🚥 갈림길 {o.id} 출구: {ex}')
            # 실제로 움직인 쪽으로 출구 보정: 정지 자리에서 0.15 m 넘게 움직였고 중심에서 0.2 m 넘게 벗어났으면 그 방향이 출구
            # (보고된 방향·yaw 가 틀려도 따라잡는다 — 실물 10-09 00:12 에서 출구 차선 위 로봇을 hold 함)
            if o.pose is not None and j['owner_pose'] is not None:
                mv = math.hypot(o.pose[0] - j['owner_pose'][0], o.pose[1] - j['owner_pose'][1])
                dc = self._jn_dist(o, j)
                if mv > 0.15 and dc > 0.2:
                    obs = math.atan2(o.pose[1] - j['c'][1], o.pose[0] - j['c'][0])
                    if j['exit'] is None or ang_diff(obs, j['exit'][1]) > math.radians(45):
                        self.get_logger().info(f'🚥 갈림길 {o.id} 실제 출구 방향 {math.degrees(obs):.0f}° 로 보정')
                        j['exit'] = ('observed', obs)
            # 출구 차선 위 로봇: 우선 로봇이 곧 그쪽으로 나간다 → 세우면 교착 (23:44 실물) → hold 대신 바로 양보
            if j['exit'] is not None:
                for r in list(self.robots.values()):
                    if r is o or self._busy(r) or not r.active(now) or r.traffic not in (None, 'HOLD'):
                        continue
                    if not self._on_exit(r, j) or not self._jn_eligible(r, j):
                        continue
                    j['held'].discard(r)
                    self._junction_yield(j, o, r, 'on owner exit')
                    if self._busy(o):
                        break
            if self._busy(o):
                continue
            # 나중에 다가오는 로봇 hold (갈림길 쪽으로 오는 중이거나 갈림길에 서 있는 로봇)
            for r in self.robots.values():
                if r is o or r in j['held'] or self._busy(r) or not r.active(now) or r.traffic is not None:
                    continue
                d = self._jn_dist(r, j)
                if d > self.jn_radius or not self._jn_eligible(r, j):
                    continue
                toward = (j['c'][0] - r.pose[0]) * math.cos(r.pose[2]) + (j['c'][1] - r.pose[1]) * math.sin(r.pose[2]) > 0
                if toward or r.jn_since is not None:
                    j['held'].add(r)
                    self.get_logger().info(f'✋ {r.id} 갈림길 대기 ({d:.2f} m): {o.id} 가 먼저 정지')
                    self._send(r, 'hold')
            j['held'] = {r for r in j['held'] if r.active(now) and not self._busy(r)}
            # 교착: 우선 로봇이 대기 로봇 때문에 출구 확인을 못 하거나(junction busy), 출구로 나가다 대기 로봇 앞에서
            # 막힘(obstacle ahead, 2026-10-08 23:44 실물) → 대기 로봇을 비켜서게
            det = str(o.st.get('detail', ''))
            ahead = [r for r in j['held'] if self._in_front(o, r)] if det == 'obstacle ahead' else []
            busy = det.startswith('junction busy') or bool(ahead)
            j['busy_t'] = (j['busy_t'] or now) if busy else None
            if busy and now - j['busy_t'] >= self.jn_busy_wait and j['held']:
                y = (ahead or sorted(j['held'], key=lambda r: math.hypot(r.pose[0] - o.pose[0], r.pose[1] - o.pose[1])))[0]
                # 벽 확인은 로봇끼리가 아니라 각자와 갈림길 중심 사이로 (두 로봇을 잇는 직선은 칸막이 끝을 스칠 수 있다)
                if (math.hypot(y.pose[0] - o.pose[0], y.pose[1] - o.pose[1]) <= 1.0
                        and self._jn_eligible(y, j) and self._jn_eligible(o, j)):
                    j['held'].discard(y)
                    j['busy_t'] = None
                    self._junction_yield(j, o, y, det)

    def _on_exit(self, r, j):
        """r 이 우선 로봇이 고른 출구 차선 위(갈림길 중심에서 출구 방향 ±45°, junction_exit_radius 안)에서 갈림길 쪽으로
        오고 있는가. 같은 출구로 앞서 빠져나가는(멀어지는) 로봇은 막는 로봇이 아니다 (follow 시나리오)."""
        dx, dy = r.pose[0] - j['c'][0], r.pose[1] - j['c'][1]
        d = math.hypot(dx, dy)
        toward = -dx * math.cos(r.pose[2]) - dy * math.sin(r.pose[2]) > 0
        return (0.05 < d <= self.jn_exit_radius and toward
                and ang_diff(math.atan2(dy, dx), j['exit'][1]) <= math.radians(45))

    def _in_front(self, o, r, ahead=0.45, half=0.20):
        dx, dy = r.pose[0] - o.pose[0], r.pose[1] - o.pose[1]
        fx = dx * math.cos(o.pose[2]) + dy * math.sin(o.pose[2])
        fy = -dx * math.sin(o.pose[2]) + dy * math.cos(o.pose[2])
        return 0.0 < fx <= ahead and abs(fy) <= half

    def _junction_yield(self, j, o, y, why):
        e = Encounter(o, y, f'junction: {why}')
        e.force_priority = o
        self.encounters.append(e)
        self.get_logger().info(f'⚠️ 갈림길 ({j["c"][0]:.2f},{j["c"][1]:.2f}): {y.id} 가 {o.id} 출구를 막음 ({why}) → {y.id} 양보')
        self._send(o, 'hold')

    def _detect(self, now):
        live = [r for r in self.robots.values() if r.active(now) and r.traffic is None and not self._busy(r)]
        for i, a in enumerate(live):
            for b in live[i + 1:]:
                if self._busy(a) or self._busy(b) or self.cool.get(frozenset((a.id, b.id)), 0) > now:
                    continue
                pa, pb = a.pose, b.pose
                why = None
                if headon(self.geo, pa, pb, self.meet_dist):
                    why = 'head-on'
                elif (a.st.get('detail') == 'obstacle ahead' and b.st.get('detail') == 'obstacle ahead'
                      and headon(self.geo, pa, pb, self.deadlock_dist, math.radians(100))):
                    why = 'deadlock'
                if why:
                    e = Encounter(a, b, why)
                    self.encounters.append(e)
                    self.get_logger().info(f'⚠️ 차선 마주침 {a.id}↔{b.id} ({why}, {math.hypot(pb[0] - pa[0], pb[1] - pa[1]):.2f} m): 둘 다 정지')
                    self._send(a, 'hold')
                    self._send(b, 'hold')

    def _detect_parked(self, now):
        """서 있는 로봇(미션 없음·도착, 차선 위)이 주행 로봇 바로 앞을 막고 있으면(주행 로봇 'obstacle ahead' 2 s 이상)
        서 있는 로봇이 비켜선다. 주행 로봇이 우선 (원래 목표로 계속)."""
        for r in self.robots.values():
            if not (r.active(now) and r.traffic is None and not self._busy(r) and r.st.get('detail') == 'obstacle ahead'):
                r.block_t = None
                continue
            blocker = None
            for o in self.robots.values():
                if o is r or not o.fresh(now) or o.pose is None or o.st.get('state') not in INACTIVE:
                    continue
                if self._busy(o) or o.traffic is not None or not self.geo.on_lane(o.pose):
                    continue
                d = math.hypot(o.pose[0] - r.pose[0], o.pose[1] - r.pose[1])
                ab = math.atan2(o.pose[1] - r.pose[1], o.pose[0] - r.pose[0])
                if d <= 0.45 and ang_diff(r.pose[2], ab) <= math.radians(50) and not self.geo.wall_between(r.pose, o.pose):
                    blocker = o
                    break
            if blocker is None:
                r.block_t = None
                continue
            r.block_t = r.block_t or now
            if now - r.block_t < 2.0 or self.cool.get(frozenset((r.id, blocker.id)), 0) > now:
                continue
            r.block_t = None
            e = Encounter(r, blocker, 'parked robot blocking')
            e.force_priority, e.parked = r, blocker
            self.encounters.append(e)
            self.get_logger().info(f'⚠️ 서 있는 {blocker.id} 가 {r.id} 앞을 막음 ({math.hypot(blocker.pose[0] - r.pose[0], blocker.pose[1] - r.pose[1]):.2f} m) → {blocker.id} 비켜서기')
            self._send(r, 'hold')

    def _alerts(self, now):
        out = []
        for r in self.robots.values():
            if not r.active(now) or r.traffic is not None:
                continue
            stopped = r.still_for(now)
            lost = 'lost' in str(r.st.get('detail', '')).lower()
            if stopped >= 10.0 and not self.geo.on_lane(r.pose, 0.08):
                out.append({'robot': r.id, 'reason': 'off_lane', 'pose': [round(v, 2) for v in r.pose], 'for': round(stopped)})
            elif stopped >= 10.0 and lost:
                out.append({'robot': r.id, 'reason': 'lost', 'pose': [round(v, 2) for v in r.pose], 'for': round(stopped)})
        new = {(a['robot'], a['reason']) for a in out} - getattr(self, '_alert_keys', set())
        for k in new:
            self.get_logger().warn(f'🔔 {k[0]}: {"차선 밖에 멈춤" if k[1] == "off_lane" else "차선을 놓치고 멈춤"} (10 s 이상)')
        self._alert_keys = {(a['robot'], a['reason']) for a in out}
        return out

    def _step(self, e, now):
        age = now - e.t0
        # 어느 한쪽이 미션을 그만두면(취소·E-STOP·오프라인·도착) 남은 쪽을 풀어 주고 끝낸다
        # (우선권 로봇이 도착해 끝난 경우: 양보 로봇은 REJOIN 단계로 넘겨 복귀를 끝까지 지켜본다)
        gone = [r for r in (e.a, e.b) if not r.fresh(now, 3.0)
                or (r.st.get('state') in INACTIVE and r is not e.parked)]
        if gone and e.phase in ('PASS', 'REJOIN') and gone == [e.priority]:
            if e.phase == 'PASS':
                self.get_logger().info(f'↩️ {e.priority.id} 미션 종료 → {e.yielder.id} 차선 복귀')
                e.phase, e.t0 = 'REJOIN', now
                self._release_yielder(e)
            gone = []
        if gone:
            for r in (e.a, e.b):
                if r not in gone and r.fresh(now, 3.0):
                    if r is e.yielder and e.phase in ('YIELD', 'PASS', 'REJOIN'):
                        self._release_yielder(e)
                    else:
                        self._release(r, 'priority')
            return self._finish(e, f'{gone[0].id} 미션 종료/연결 끊김')

        if e.phase == 'HOLD':
            if e.parked is not None:
                if not (e.force_priority.acked() or age > self.hold_wait):
                    return
            elif not ((e.a.acked() and e.b.acked()) or age > self.hold_wait):
                return
            self._choose(e, now)
        elif e.phase == 'YIELD':
            y = e.yielder
            # 사용자 사양: 양보 로봇이 우선권 로봇의 경로 점유 공간(정면 감시 상자)을 벗어나면 YIELDED 를 기다리지 않고 바로 출발
            out = (y.acked() and y.traffic in ('YIELDING', 'YIELDED') and y.pose is not None
                   and math.hypot(y.pose[0] - e.rejoin[0], y.pose[1] - e.rejoin[1]) > 0.05
                   and not self.geo.in_corridor(y.pose[:2], e.prio_path)[0])
            if e.pending_escape is not None and y.acked() and y.traffic == 'YIELDED':
                esc, e.pending_escape = e.pending_escape, None
                e.escape, e.t0 = esc, now
                self.get_logger().info(f'↩️ {y.id} 뒤로 물러남 → 이어서 비켜서기 ({esc[0]:.2f}, {esc[1]:.2f})')
                self._send(y, 'yield', x=round(esc[0], 3), y=round(esc[1], 3))
                return
            if e.pending_escape is not None:
                out = False                 # 물러나는 중에는 아직 차선 위
            if out or (y.acked() and y.traffic == 'YIELDED'):
                e.phase, e.t0 = 'PASS', now
                self.get_logger().info(f'✅ {y.id} {"비켜섬" if y.traffic == "YIELDED" else "통로 벗어남"} → {e.priority.id} 우선 통과')
                self._release(e.priority, 'priority')
            elif (y.acked() and y.traffic == 'YIELD_FAILED') or age > self.yield_timeout:
                self.get_logger().warn(f'❌ {y.id} 비켜서기 실패 ({y.traffic}, {age:.0f}s)')
                e.failed.add(y.id)
                self._choose(e, now)
        elif e.phase == 'PASS':
            p = e.priority
            self._reyield_if_blocking(e, now)
            others = [o.pose for o in self.robots.values() if o is not e.yielder and o.fresh(now, 3.0) and o.pose]
            ret_clear = e.yielder.pose is None or not path_blocked(e.yielder.pose[:2], e.rejoin[:2], others)[0]
            if p.pose is not None and passed(p.pose, e.rejoin, self.pass_dist) and ret_clear:
                why = '통과 완료'
            elif age > self.pass_timeout:
                why = f'{self.pass_timeout:.0f}s 안에 통과 못 함'
            else:
                return
            self.get_logger().info(f'↩️ {p.id} {why} → {e.yielder.id} 차선 복귀')
            e.phase, e.t0 = 'REJOIN', now
            self._release_yielder(e)
        elif e.phase == 'REJOIN':
            y = e.yielder
            if y.acked() and y.traffic is None:
                self._finish(e, f'{y.id} 차선 복귀 완료')
            elif y.acked() and y.traffic == 'YIELD_FAILED' and e.rejoin_tries < 3:
                e.rejoin_tries += 1
                e.t0 = now
                self.get_logger().warn(f'❌ {y.id} 복귀 막힘 → 더 먼 복귀 자리로 재시도 {e.rejoin_tries}/3')
                self._release_yielder(e, skip=0.15 * e.rejoin_tries)
            elif (y.acked() and y.traffic == 'YIELD_FAILED') or age > self.rejoin_timeout:
                e.phase, e.t0 = 'STUCK', now
                self.get_logger().error(f'🆘 {y.id}: 차선 복귀 실패 — 정지 유지 (사람 확인 필요)')
        elif e.phase == 'STUCK':
            if (e.yielder is not None and e.yielder.acked() and e.yielder.traffic is None) or \
                    all(r.traffic is None for r in (e.a, e.b)):
                self._finish(e, '정지 해소 (사람이 처리)')

    def _reyield_if_blocking(self, e, now):
        """우선권 로봇이 출발한 뒤 양보 로봇 때문에 '앞 사물'로 2 s 넘게 서 있으면, 우선권 로봇의 실제 위치·방향 기준
        정면 상자 밖으로 양보 로봇을 한 번 더 비켜서게 한다 (곡선에서 차선 안쪽으로 깎아 돌면 비켜선 자리가 정면에 걸린다)."""
        p, y = e.priority, e.yielder
        blocking = (p.pose is not None and y.pose is not None and p.st.get('detail') == 'obstacle ahead'
                    and math.hypot(p.pose[0] - y.pose[0], p.pose[1] - y.pose[1]) < 0.45
                    and y.traffic in ('YIELDED', 'YIELDING', 'YIELD_FAILED'))
        if not blocking:
            e.blocked_t = None
            return
        e.blocked_t = e.blocked_t or now
        if now - e.blocked_t < 2.0 or e.reyields >= 2:
            return
        e.reyields += 1
        e.blocked_t = None
        found = None
        for max_d in (self.escape_max, 0.7):
            found = self.geo.escape_for(y.pose, p.pose[:2], max_dist=max_d, other_pose=p.pose)
            if found is not None:
                break
        if found is None:
            self.get_logger().warn(f'{y.id} 이 {p.id} 앞을 막고 있으나 더 비켜설 자리가 없음')
            return
        e.escape = found[1]
        self.get_logger().info(f'↪️ {y.id} 이 {p.id} 정면에 걸림 → 다시 비켜서기 ({found[1][0]:.2f}, {found[1][1]:.2f}) {e.reyields}/2')
        self._send(y, 'yield', x=round(found[1][0], 3), y=round(found[1][1], 3))

    def _choose(self, e, now):
        """우선권·양보 로봇과 비켜설 자리를 정하고 yield 를 보낸다 (실패한 로봇은 다시 고르지 않는다)"""
        cands = [r for r in (e.a, e.b) if r.id not in e.failed]
        if not cands:
            e.phase, e.t0 = 'STUCK', now
            self.get_logger().error(f'🆘 {e.a.id}↔{e.b.id}: 둘 다 비켜서지 못함 — 정지 유지 (사람 확인 필요)')
            return
        def rec(r):
            return {'id': r.id, 'pose': r.pose, 'remaining': r.st.get('dist'), 'robot': r}
        if e.force_priority is not None and len(cands) == 2:
            cands = [e.other(e.force_priority)]      # 우선권이 정해져 있으면 상대만 양보 후보
        res = None
        for max_d in (self.escape_max, 0.7):
            if len(cands) == 2:
                res = choose_yielder(self.geo, rec(e.a), rec(e.b), max_dist=max_d)
                if res is not None:
                    y, p, esc, info = res[0]['robot'], res[1]['robot'], res[2], res[3]
                    break
            else:
                y = cands[0]
                found = None
                if e.force_priority is not None:
                    # 갈림길: 우선 로봇의 출구 확인(jn_wait)은 출구 경로 ±0.20 m 의 다른 로봇을 보므로 그보다 멀리 (없으면 0.19)
                    found = self.geo.escape_for(y.pose, e.other(y).pose[:2], max_dist=max_d, center_clear=0.25)
                found = found or self.geo.escape_for(y.pose, e.other(y).pose[:2], max_dist=max_d)
                if found is not None:
                    why = ('parked robot yields' if e.parked is not None else 'junction first-come') \
                        if e.force_priority is not None and not e.failed else f'{e.other(y).id} failed'
                    p, esc, info = e.other(y), found[1], {'why': why, y.id: round(found[0], 3)}
                    res = True
                    break
        if res is None:
            # 바로 비켜설 자리가 없으면 차선을 따라 뒤로 물러난 다음 비켜선다 (main brain 제안 2026-10-09)
            order = cands if e.force_priority is not None else sorted(cands, key=lambda r: -(r.st.get('dist') or 0.0))
            for y in order:
                rt = retreat_then_escape(self.geo, y.pose, e.other(y).pose[:2])
                if rt is not None:
                    p, esc, info = e.other(y), rt[0], {'why': 'retreat on lane, then escape', 'then': [round(v, 2) for v in rt[1]]}
                    e.pending_escape = rt[1]
                    res = True
                    break
        if res is None:
            e.phase, e.t0 = 'STUCK', now
            self.get_logger().error(f'🆘 {e.a.id}↔{e.b.id}: 비켜설 자리도, 물러날 자리도 찾지 못함 — 정지 유지 (사람 확인 필요)')
            return
        e.yielder, e.priority, e.escape, e.info = y, p, esc, info
        e.rejoin = y.pose
        e.prio_path = self.geo.priority_path(p.pose[:2], y.pose[:2])
        e.phase, e.t0 = 'YIELD', now
        self.get_logger().info(f'🚦 우선권 {p.id}, 양보 {y.id} → 비켜설 자리 ({esc[0]:.2f}, {esc[1]:.2f}) {info}')
        # 앞서 비켜서다 실패한 로봇은 hold 를 보내지 않는다: 로봇이 HOLD 로 바뀌면 나중 resume 때 복귀(rejoin) 없이
        # 차선 밖에서 바로 주행을 이어 간다. 지난 yield 명령을 keepalive 로 계속 보내 YIELD_FAILED(정지)로 둔다.
        if p.id not in e.failed and (p.cmd is None or p.cmd['cmd'] != 'hold'):
            self._send(p, 'hold')
        self._send(y, 'yield', x=round(esc[0], 3), y=round(esc[1], 3))

    def _on_global_cmd(self, m):
        if m.data.strip().upper() != 'E_STOP':
            return
        if self.encounters:
            self.get_logger().warn(f'🛑 GLOBAL E-STOP: 차선 마주침 조정 {len(self.encounters)}건 취소')
        self.encounters.clear()
        for j in self.junctions:
            j.update(owner=None, owner_pose=None, held=set(), busy_t=None, exit=None)
        for r in self.robots.values():
            r.cmd = None                # 로봇 차선 노드는 E-STOP 을 직접 받아 미션을 끝낸다

    def _publish_state(self):
        now = time.monotonic()
        robots = {r.id: {'traffic': r.traffic, 'active': r.active(now),
                         'cmd': r.cmd['cmd'] if r.cmd else None, 'acked': r.acked()}
                  for r in self.robots.values()}
        self.state_pub.publish(String(data=json.dumps(
            {'enabled': self.geo is not None, 'encounters': [e.summary() for e in self.encounters], 'robots': robots,
             'alerts': self._alerts(now) if self.geo is not None else [],
             'junctions': [{'c': [round(v, 2) for v in j['c']], 'owner': j['owner'].id if j['owner'] else None,
                            'exit': j['exit'][0] if j['exit'] else None,
                            'held': sorted(r.id for r in j['held'])} for j in self.junctions]})))


def main():
    rclpy.init()
    node = LaneTraffic()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):     # Ctrl+C / launch 종료
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
