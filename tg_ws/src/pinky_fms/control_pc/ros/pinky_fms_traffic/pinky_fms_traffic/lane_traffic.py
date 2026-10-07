"""차선 주행(Vision-based Lane Following) 로봇끼리의 마주침 관제 (fleet_traffic 과 별개 노드).

fleet_traffic 은 차선 미션 중인 로봇을 관제 대상에서 뺀다(lane_active). 이 노드는 그 차선 로봇들만 본다.
  1) 1차선에서 두 로봇이 서로를 향해 meet_dist 안에 들어오면 (또는 둘 다 '앞 사물'로 멈춰 교착이면)
  2) 둘 다 일시정지(hold)
  3) 비켜서기 쉬운 쪽이 양보: 차선 밖 비켜설 자리로 이동(yield) → 도착(YIELDED)하면 우선권 로봇 재개(resume priority)
  4) 우선권 로봇이 복귀 자리를 지나가면 양보 로봇 재개(resume yielder): 떠난 자리·방향으로 돌아와 차선 추종 재개

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
from rclpy.node import Node
from std_msgs.msg import String

from .lane_geom import LaneGeometry, choose_yielder, headon, passed, rejoin_point

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
        gp = lambda n: self.get_parameter(n).value
        self.meet_dist, self.deadlock_dist = gp('meet_dist'), gp('deadlock_dist')
        self.escape_max, self.pass_dist = gp('escape_max'), gp('pass_dist')
        self.hold_wait, self.yield_timeout = gp('hold_wait'), gp('yield_timeout')
        self.pass_timeout, self.rejoin_timeout, self.cooldown = gp('pass_timeout'), gp('rejoin_timeout'), gp('cooldown')
        self.geo = None
        if gp('lanes_yaml') and gp('floor_yaml'):
            try:
                self.geo = LaneGeometry(gp('lanes_yaml'), gp('floor_yaml'))
            except Exception as e:      # 지도가 없거나 형식이 다르면 조정 없이 상태만 발행
                self.get_logger().error(f'지도 읽기 실패: {e}')
        if self.geo is None:
            self.get_logger().warn('lanes_yaml/floor_yaml 이 없어 차선 마주침 조정 꺼짐')
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
        rj = rejoin_point(self.geo, e.rejoin, y.pose[:2] if y.pose else e.rejoin[:2], blockers, skip=skip)
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
        if self.geo is not None:
            self._detect(now)
            for e in list(self.encounters):
                self._step(e, now)
        for r in self.robots.values():       # resume 이 확인됐거나 로봇이 사라지면 keepalive 중단
            if r.cmd is not None and r.cmd['cmd'] == 'resume' and not any(r in (e.a, e.b) for e in self.encounters):
                if r.acked() or not r.fresh(now, 5.0):
                    r.cmd = None
        self._pump(now)

    def _busy(self, r):
        return any(r in (e.a, e.b) for e in self.encounters)

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

    def _step(self, e, now):
        age = now - e.t0
        # 어느 한쪽이 미션을 그만두면(취소·E-STOP·오프라인·도착) 남은 쪽을 풀어 주고 끝낸다
        # (우선권 로봇이 도착해 끝난 경우: 양보 로봇은 REJOIN 단계로 넘겨 복귀를 끝까지 지켜본다)
        gone = [r for r in (e.a, e.b) if not r.fresh(now, 3.0) or r.st.get('state') in INACTIVE]
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
            if not ((e.a.acked() and e.b.acked()) or age > self.hold_wait):
                return
            self._choose(e, now)
        elif e.phase == 'YIELD':
            y = e.yielder
            # 사용자 사양: 양보 로봇이 우선권 로봇의 경로 점유 공간(정면 감시 상자)을 벗어나면 YIELDED 를 기다리지 않고 바로 출발
            out = (y.acked() and y.traffic in ('YIELDING', 'YIELDED') and y.pose is not None
                   and math.hypot(y.pose[0] - e.rejoin[0], y.pose[1] - e.rejoin[1]) > 0.05
                   and not self.geo.in_corridor(y.pose[:2], e.prio_path)[0])
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
            if p.pose is not None and passed(p.pose, e.rejoin, self.pass_dist):
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
        res = None
        for max_d in (self.escape_max, 0.7):
            if len(cands) == 2:
                res = choose_yielder(self.geo, rec(e.a), rec(e.b), max_dist=max_d)
                if res is not None:
                    y, p, esc, info = res[0]['robot'], res[1]['robot'], res[2], res[3]
                    break
            else:
                y = cands[0]
                found = self.geo.escape_for(y.pose, e.other(y).pose[:2], max_dist=max_d)
                if found is not None:
                    p, esc, info = e.other(y), found[1], {'why': f'{e.other(y).id} failed', y.id: round(found[0], 3)}
                    res = True
                    break
        if res is None:
            e.phase, e.t0 = 'STUCK', now
            self.get_logger().error(f'🆘 {e.a.id}↔{e.b.id}: 비켜설 자리를 찾지 못함 — 정지 유지 (사람 확인 필요)')
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
        for r in self.robots.values():
            r.cmd = None                # 로봇 차선 노드는 E-STOP 을 직접 받아 미션을 끝낸다

    def _publish_state(self):
        now = time.monotonic()
        robots = {r.id: {'traffic': r.traffic, 'active': r.active(now),
                         'cmd': r.cmd['cmd'] if r.cmd else None, 'acked': r.acked()}
                  for r in self.robots.values()}
        self.state_pub.publish(String(data=json.dumps(
            {'enabled': self.geo is not None, 'encounters': [e.summary() for e in self.encounters], 'robots': robots})))


def main():
    rclpy.init()
    node = LaneTraffic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
